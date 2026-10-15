// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file cttc-nr-v2x-nr-sl-mpquic.cc
 * @brief NR Uu (5G cellular) + NR SL PC5 (3GPP sidelink) + MPQUIC heterogeneous uplink.
 *
 * Replaces the WiFi 802.11p PC5 approximation in cttc-nr-v2x-wifi-mpquic.cc with
 * real 3GPP Rel.16 NR-V2X sidelink (PC5 Mode 2, sensing-based resource allocation).
 *
 * Topology:
 *
 *   [gNB (0,0,10)]
 *       | NR Uu 3.5 GHz
 *   [Vehicle (50,0,1.5)] ---(NR SL PC5, 5.89 GHz, Mode 2 SPS)---> [RSU SL UE (300,0,5)]
 *       |                                                                  |
 *       | path 0: Vehicle(7.0.0.2) → NR Uu → EPC/PGW → remoteHost:4000  | P2P backhaul
 *       | path 1: Vehicle(7.0.0.3) → NR SL → RSU(7.0.0.4) → remoteHost:4000
 *       +--------------------------------------------------------------------------+
 *                                                              [remoteHost]
 *
 * MPQUIC paths:
 *   Path 0 (Uu):  Vehicle(7.0.0.2) → gNB → EPC → PGW(1.0.0.1) → P2P → remoteHost(1.0.0.2):4000
 *   Path 1 (NR SL): Vehicle(7.0.0.3) → NR SL → RSU(7.0.0.4) → P2P → remoteHost(10.10.2.2):4000
 *
 * TFT design:
 *   Vehicle TRANSMIT TFT: remote = remoteHost SL IP (10.10.2.2) → SL bearer for PC5 path
 *   RSU BIDIRECTIONAL TFT: remote = vehicle SL IP (7.0.0.3) → SL bearer for ACKs
 *
 * Routing:
 *   remoteHost: 7.0.0.0/8 via PGW (Uu ACKs); /32 host route 7.0.0.3 via RSU backhaul (SL ACKs)
 *   Vehicle: default via EPC gateway (Uu); host route 10.10.2.2 via RSU SL IP, SL iface
 *   RSU: connected 7.0.0.0/8 on SL iface; IP forwarding enabled (auto in ns-3)
 *
 * Telemetry CSV:
 *   time_s, uu_rsrp_dBm, uu_sinr_dB, rsu_dist_m, rsu_angle_deg,
 *   queue_depth, total_rx_pkts, gamma0
 *
 * Usage:
 *   ./ns3 run "cttc-nr-v2x-nr-sl-mpquic"
 *   ./ns3 run "cttc-nr-v2x-nr-sl-mpquic --simTime=10 --schedulerType=MinRtt --logging=true"
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mpquic-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrV2xNrSlMpquic");

// ---------------------------------------------------------------------------
// NR Uu PHY trace callbacks
// ---------------------------------------------------------------------------
double g_rsrpDbm{-140.0};
double g_sinrDb{-10.0};

void CbRsrp(uint16_t, uint16_t, uint16_t, double rsrp, uint8_t) { g_rsrpDbm = rsrp; }
void CbSinr(uint16_t, uint16_t, double sinr, uint16_t)
{
    g_sinrDb = (sinr > 0.0) ? 10.0 * std::log10(sinr) : -100.0;
}

// ---------------------------------------------------------------------------
// Periodic telemetry snapshot
// ---------------------------------------------------------------------------
std::ofstream g_csv;

void
TakeSnapshot(Ptr<MpQuicSocketBase>  mpquic,
             Ptr<MpQuicReceiverApp> rx,
             Ptr<MobilityModel>     vehMob,
             Ptr<MobilityModel>     rsuMob,
             Time                   interval,
             Time                   simEnd)
{
    if (Simulator::Now() > simEnd)
        return;

    Vector vp   = vehMob->GetPosition();
    Vector rp   = rsuMob->GetPosition();
    double dx   = rp.x - vp.x;
    double dy   = rp.y - vp.y;
    double dz   = rp.z - vp.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    double ang  = std::atan2(dy, dx) * 180.0 / M_PI;

    g_csv << Simulator::Now().GetSeconds()
          << "," << g_rsrpDbm
          << "," << g_sinrDb
          << "," << dist
          << "," << ang
          << "," << mpquic->GetTotalQueueDepth()
          << "," << rx->GetTotalRxPackets()
          << "," << rx->GetContinuity(0)
          << "\n";

    Simulator::Schedule(interval, &TakeSnapshot,
                        mpquic, rx, vehMob, rsuMob, interval, simEnd);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int
main(int argc, char* argv[])
{
    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    bool        logging        = false;
    Time        simTime        = Seconds(10.0);
    Time        appStart       = MilliSeconds(500);
    std::string outputDir      = ".";
    std::string simTag         = "nr-sl-mpquic";

    // Vehicle
    double vehicleX     = 50.0;    // m — initial position along road
    double vehicleSpeed = 15.0;    // m/s (~55 km/h)

    // RSU
    double rsuX = 300.0;
    double rsuY = 0.0;
    double rsuZ = 5.0;

    // NR Uu (Korea 5G NR n78, sub-6 GHz)
    double   uuFreqHz     = 3.5e9;
    double   uuBandwidth  = 20e6;
    uint16_t uuNumerology = 1;
    double   uuTxPowerDbm = 23.0;

    // NR SL PC5 (3GPP NR-V2X, ~n47/ch172 vicinity, 5.89 GHz)
    double   slFreqHz     = 5.89e9;
    uint16_t slBwRb       = 400;    // 40 MHz in units of 100 kHz
    uint16_t slNumerology = 2;      // μ=2, 60 kHz SCS
    double   slTxPowerDbm = 23.0;
    bool     slHarq       = false;  // disable SL HARQ for simplicity false

    // MPQUIC — single receiver on remoteHost collects from both paths.
    uint16_t    rxPort        = 4000;
    std::string schedulerType = "MinRtt";
    uint32_t    pktSize       = 1200;   // bytes
    Time        pktInterval   = MilliSeconds(20);  // 50 pps; each path ~25 pps < SL SPS 50 pps

    CommandLine cmd(__FILE__);
    cmd.AddValue("logging",       "Enable ns-3 logging",          logging);
    cmd.AddValue("simTime",       "Simulation duration (s)",       simTime);
    cmd.AddValue("appStart",      "App start time (s)",            appStart);
    cmd.AddValue("outputDir",     "Output directory",              outputDir);
    cmd.AddValue("simTag",        "Simulation tag (file prefix)",  simTag);
    cmd.AddValue("vehicleX",      "Vehicle initial X (m)",         vehicleX);
    cmd.AddValue("vehicleSpeed",  "Vehicle speed m/s",             vehicleSpeed);
    cmd.AddValue("rsuX",          "RSU X position (m)",            rsuX);
    cmd.AddValue("uuTxPowerDbm",  "NR UE Uu TX power (dBm)",       uuTxPowerDbm);
    cmd.AddValue("slTxPowerDbm",  "NR SL TX power (dBm)",          slTxPowerDbm);
    cmd.AddValue("slHarq",        "Enable NR SL HARQ",             slHarq);
    cmd.AddValue("schedulerType", "MPQUIC scheduler: RoundRobin|MinRtt|Redundant|Peekaboo",
                 schedulerType);
    cmd.AddValue("pktSize",       "MPQUIC packet size (B)",        pktSize);
    cmd.AddValue("pktInterval",   "MPQUIC send interval (ms)",     pktInterval);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(simTime <= appStart, "simTime must be > appStart");

    if (logging)
    {
        LogComponentEnable("CttcNrV2xNrSlMpquic",  LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicReceiverApp",     LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicSenderApp",       LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicSocketBase",      LOG_LEVEL_DEBUG);
        LogComponentEnable("MpQuicScheduler",       LOG_LEVEL_DEBUG);
        LogComponentEnable("NrSlHelper",            LOG_LEVEL_INFO);
        LogComponentEnable("NrSlUeRrc",             LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // -----------------------------------------------------------------------
    // Nodes
    // -----------------------------------------------------------------------
    NodeContainer gnbNodes;     gnbNodes.Create(1);
    NodeContainer vehicleNodes; vehicleNodes.Create(1);
    NodeContainer rsuNodes;     rsuNodes.Create(1);      // SL peer + backhaul relay
    NodeContainer remoteHostNC; remoteHostNC.Create(1);
    Ptr<Node> remoteHost = remoteHostNC.Get(0);

    // SL UE container: vehicle[0] + RSU[1]
    NodeContainer slUeContainer;
    slUeContainer.Add(vehicleNodes.Get(0));
    slUeContainer.Add(rsuNodes.Get(0));

    // -----------------------------------------------------------------------
    // Mobility
    // -----------------------------------------------------------------------
    MobilityHelper gnbMob;
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, 10.0));
    gnbMob.SetPositionAllocator(gnbPos);
    gnbMob.Install(gnbNodes);

    MobilityHelper vehMob;
    vehMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    Ptr<ListPositionAllocator> vehPos = CreateObject<ListPositionAllocator>();
    vehPos->Add(Vector(vehicleX, 0.0, 1.5));
    vehMob.SetPositionAllocator(vehPos);
    vehMob.Install(vehicleNodes);
    vehicleNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(vehicleSpeed, 0.0, 0.0));

    MobilityHelper rsuMob;
    rsuMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> rsuPos = CreateObject<ListPositionAllocator>();
    rsuPos->Add(Vector(rsuX, rsuY, rsuZ));
    rsuMob.SetPositionAllocator(rsuPos);
    rsuMob.Install(rsuNodes);

    MobilityHelper rhMob;
    rhMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> rhPos = CreateObject<ListPositionAllocator>();
    rhPos->Add(Vector(0.0, 0.0, 0.0));
    rhMob.SetPositionAllocator(rhPos);
    rhMob.Install(remoteHostNC);

    // -----------------------------------------------------------------------
    // Internet stack (all nodes)
    // -----------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(vehicleNodes);
    internet.Install(rsuNodes);
    internet.Install(remoteHostNC);
    // gnbNodes: NR module installs the stack automatically

    // =====================================================================
    // NR helper — shared for both Uu and SL bands
    // =====================================================================
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper>  bfHelper  = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // -----------------------------------------------------------------
    // Band 1: NR Uu (3.5 GHz, UMa channel model)
    // -----------------------------------------------------------------
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf uuBandConf(uuFreqHz, uuBandwidth, 1,
                                                      BandwidthPartInfo::UMa);
    OperationBandInfo uuBand = ccBwpCreator.CreateOperationBandContiguousCc(uuBandConf);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&uuBand);
    BandwidthPartInfoPtrVector uuBwps = CcBwpCreator::GetAllBwps({uuBand});

    Packet::EnableChecking();
    Packet::EnablePrinting();

    bfHelper->SetAttribute("BeamformingMethod",
                           TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Uu UE antenna (before installing Uu devices)
    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    // Uu gNB antenna
    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Install gNB and vehicle Uu device (uses default NrUeMac)
    NetDeviceContainer gnbNetDev    = nrHelper->InstallGnbDevice(gnbNodes, uuBwps);
    NetDeviceContainer vehicleNrDev = nrHelper->InstallUeDevice(vehicleNodes, uuBwps);

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(gnbNetDev,    stream);
    stream += nrHelper->AssignStreams(vehicleNrDev, stream);

    // gNB PHY config
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(uuNumerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(40.0));

    // UE Uu PHY: set TX power to 23 dBm (default 2 dBm causes uplink loss)
    nrHelper->GetUePhy(vehicleNrDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(uuTxPowerDbm));

    DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0))->UpdateConfig();
    DynamicCast<NrUeNetDevice>(vehicleNrDev.Get(0))->UpdateConfig();

    // -----------------------------------------------------------------
    // Band 2: NR SL (5.89 GHz, V2V_Highway channel model)
    // -----------------------------------------------------------------

    // Switch to NrSlUeMac for SL devices
    uint8_t bwpIdSl = 0;
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing",  BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1",             UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2",             UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId",   UintegerValue(0));
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdSl));

    CcBwpCreator::SimpleOperationBandConf slBandConf(slFreqHz, slBwRb * 100e3, 1,
                                                      BandwidthPartInfo::V2V_Highway);
    OperationBandInfo slBand = ccBwpCreator.CreateOperationBandContiguousCc(slBandConf);

    // SL UE antenna (isotropic, 1x2)
    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(slTxPowerDbm));

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&slBand);
    BandwidthPartInfoPtrVector slBwps = CcBwpCreator::GetAllBwps({slBand});

    // slNetDev[0] = vehicle SL device, slNetDev[1] = RSU SL device
    NetDeviceContainer slNetDev = nrHelper->InstallUeDevice(slUeContainer, slBwps);
    stream += nrHelper->AssignStreams(slNetDev, stream);

    for (auto it = slNetDev.Begin(); it != slNetDev.End(); ++it)
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // -----------------------------------------------------------------
    // NrSlHelper — configures SL physical/MAC layers
    // -----------------------------------------------------------------
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);
    nrSlHelper->SetSlErrorModel("ns3::NrEesmIrT1");
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdSl);
    nrSlHelper->PrepareUeForSidelink(slNetDev, bwpIdContainer);

    // -----------------------------------------------------------------------
    // EPC IP assignment
    // Vehicle Uu: 7.0.0.2, Vehicle SL: 7.0.0.3, RSU SL: 7.0.0.4
    // (EPC assigns sequentially from 7.0.0.2)
    // -----------------------------------------------------------------------
    Ipv4InterfaceContainer vehicleUuIp =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(vehicleNrDev));
    Ipv4InterfaceContainer vehicleSlIp =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(slNetDev.Get(0)));
    Ipv4InterfaceContainer rsuSlIp =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(slNetDev.Get(1)));

    // Vehicle default route via EPC gateway (Uu interface)
    Ipv4StaticRoutingHelper ipv4Routing;
    Ptr<Ipv4StaticRouting> vehicleRoute =
        ipv4Routing.GetStaticRouting(vehicleNodes.Get(0)->GetObject<Ipv4>());
    vehicleRoute->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // Attach vehicle Uu to nearest gNB
    nrHelper->AttachToClosestEnb(vehicleNrDev, gnbNetDev);

    // -----------------------------------------------------------------------
    // P2P 1: PGW <-> remoteHost (Uu backhaul, 100 Gbps)
    // -----------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu",      UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(0)));
    NetDeviceContainer pgwRhDevs = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwRhIfaces = ipv4h.Assign(pgwRhDevs);
    // pgw=1.0.0.1, remoteHost=1.0.0.2
    const Ipv4Address remoteUuIp = pgwRhIfaces.GetAddress(1);  // 1.0.0.2

    // -----------------------------------------------------------------------
    // P2P 2: RSU <-> remoteHost (SL backhaul, 1 Gbps)
    // -----------------------------------------------------------------------
    PointToPointHelper backhaulP2p;
    backhaulP2p.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    backhaulP2p.SetChannelAttribute("Delay",   StringValue("1ms"));
    NetDeviceContainer backhaulDevs = backhaulP2p.Install(rsuNodes.Get(0), remoteHost);

    ipv4h.SetBase("10.10.2.0", "255.255.255.0");
    Ipv4InterfaceContainer backhaulIfaces = ipv4h.Assign(backhaulDevs);
    // RSU backhaul=10.10.2.1, remoteHost SL=10.10.2.2
    const Ipv4Address remotePC5Ip = backhaulIfaces.GetAddress(1);  // 10.10.2.2

    // -----------------------------------------------------------------------
    // Routing
    // -----------------------------------------------------------------------

    // --- remoteHost ---
    Ptr<Ipv4StaticRouting> rhRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());

    // /8 route for Uu UE pool (7.0.0.0/8 → via PGW 1.0.0.1, iface 1 = P2P to pgw)
    rhRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"),
                               pgwRhIfaces.GetAddress(0), 1);

    // /32 host route for vehicle SL IP (7.0.0.3 → via RSU backhaul 10.10.2.1)
    // More specific than /8, overrides pgw route for SL ACKs.
    const uint32_t rhBackhaulIf =
        remoteHost->GetObject<Ipv4>()
                  ->GetInterfaceForDevice(backhaulDevs.Get(1));
    rhRoute->AddHostRouteTo(vehicleSlIp.GetAddress(0),   // 7.0.0.3
                            backhaulIfaces.GetAddress(0), // via RSU (10.10.2.1)
                            rhBackhaulIf);

    // --- RSU ---
    // RSU needs to forward SL packets (dst=10.10.2.2) via backhaul.
    // Connected route to backhaul (10.10.2.0/24) is auto-added.
    // Connected route to SL UE pool (7.0.0.0/8) is auto-added by EPC assignment.
    // No explicit routes needed — IP forwarding is on by default.

    // --- Vehicle ---
    // Explicit host route to remotePC5Ip (10.10.2.2) via RSU SL IP (7.0.0.4) on SL interface.
    // iface 2 = SL NrUeNetDevice (Uu is iface 1, SL is iface 2).
    const uint32_t vehSlIf =
        vehicleNodes.Get(0)->GetObject<Ipv4>()
                           ->GetInterfaceForDevice(slNetDev.Get(0));
    vehicleRoute->AddHostRouteTo(remotePC5Ip,             // 10.10.2.2
                                 rsuSlIp.GetAddress(0),   // via RSU SL IP (7.0.0.4)
                                 vehSlIf);

    // -----------------------------------------------------------------------
    // NR SL resource pool configuration
    // (same as cttc-nr-v2x-mpquic-test.cc; slHarq=false by default)
    // -----------------------------------------------------------------------
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100);
    ptrFactory->SetSlSelectionWindow(5);
    ptrFactory->SetSlFreqResourcePscch(10);
    ptrFactory->SetSlSubchannelSize(50);
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> reservePeriodList = {20, 100};
    ptrFactory->SetSlResourceReservePeriodList(reservePeriodList);
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();

    LteRrcSap::SlResourcePoolIdNr slPoolId;  slPoolId.id = 0;
    LteRrcSap::SlResourcePoolConfigNr slPoolCfg;
    slPoolCfg.haveSlResourcePoolConfigNr = true;
    slPoolCfg.slResourcePoolId           = slPoolId;
    slPoolCfg.slResourcePool             = pool;

    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPool;
    slBwpPool.slTxPoolSelectedNormal[slPoolId.id] = slPoolCfg;

    LteRrcSap::Bwp bwp;
    bwp.numerology      = slNumerology;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg        = 1;
    bwp.bandwidth       = slBwRb;

    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp             = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol   = LteRrcSap::GetSlStartSymbolEnum(0);

    LteRrcSap::SlBwpConfigCommonNr slBwpCfg;
    slBwpCfg.haveSlBwpGeneric            = true;
    slBwpCfg.slBwpGeneric                = slBwpGeneric;
    slBwpCfg.haveSlBwpPoolConfigCommonNr = true;
    slBwpCfg.slBwpPoolConfigCommonNr     = slBwpPool;

    LteRrcSap::SlFreqConfigCommonNr slFreqCfg;
    for (const auto& id : bwpIdContainer)
        slFreqCfg.slBwpList[id] = slBwpCfg;

    LteRrcSap::TddUlDlConfigCommon tddCfg;
    tddCfg.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneral;
    slPreconfigGeneral.slTddConfig = tddCfg;

    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = slHarq ? 5 : 1;
    LteRrcSap::SlPsschTxConfigList psschCfgList;
    psschCfgList.slPsschTxParameters[0] = psschParams;

    LteRrcSap::SlUeSelectedConfig slUeSelCfg;
    slUeSelCfg.slProbResourceKeep  = 0;
    slUeSelCfg.slPsschTxConfigList = psschCfgList;

    LteRrcSap::SidelinkPreconfigNr slPreCfg;
    slPreCfg.slPreconfigGeneral         = slPreconfigGeneral;
    slPreCfg.slUeSelectedPreConfig      = slUeSelCfg;
    slPreCfg.slPreconfigFreqInfoList[0] = slFreqCfg;

    nrSlHelper->InstallNrSlPreConfiguration(slNetDev, slPreCfg);
    stream += nrSlHelper->AssignStreams(slNetDev, stream);

    // -----------------------------------------------------------------------
    // SL bearer activation
    //
    // Vehicle TFT (TRANSMIT): remote = remotePC5Ip (10.10.2.2)
    //   → vehicle SL bearer carries packets destined to remoteHost via RSU
    //
    // RSU TFT (BIDIRECTIONAL): remote = vehicleSlIp (7.0.0.3)
    //   RECEIVE: accepts SL packets where src = 7.0.0.3 (from vehicle)
    //   TRANSMIT: sends ACK packets where dst = 7.0.0.3 (back to vehicle)
    // -----------------------------------------------------------------------
    Time slBearerActivation = appStart - MilliSeconds(10);

    SidelinkInfo slInfo;
    slInfo.m_castType    = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id     = 255;
    slInfo.m_rri         = MilliSeconds(20);  // SPS: 50 pps capacity per bearer
    slInfo.m_pdb         = Seconds(0);
    slInfo.m_harqEnabled = slHarq;

    // Vehicle: BIDIRECTIONAL bearer.
    //   TRANSMIT: matches packets to remotePC5Ip (10.10.2.2) → routes via NR SL.
    //   RECEIVE:  isReceive=true triggers AddNrSlRxDstL2Id(255) so MAC listens
    //             for groupcast dstL2Id=255 — required for ACK reception from RSU.
    Ptr<LteSlTft> vehTft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                             remotePC5Ip,  // 10.10.2.2
                                             slInfo);
    nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                   NetDeviceContainer(slNetDev.Get(0)),
                                   vehTft);

    // RSU: BIDIRECTIONAL bearer — receive from vehicle SL IP, send ACKs back
    Ptr<LteSlTft> rsuTft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                             vehicleSlIp.GetAddress(0),  // 7.0.0.3
                                             slInfo);
    nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                   NetDeviceContainer(slNetDev.Get(1)),
                                   rsuTft);

    // -----------------------------------------------------------------------
    // MPQUIC
    // -----------------------------------------------------------------------
    MpQuicHelper::SchedulerType sched = MpQuicHelper::SchedulerType::RoundRobin;
    if      (schedulerType == "MinRtt")    sched = MpQuicHelper::SchedulerType::MinRtt;
    else if (schedulerType == "Redundant") sched = MpQuicHelper::SchedulerType::Redundant;
    else if (schedulerType == "Peekaboo")  sched = MpQuicHelper::SchedulerType::Peekaboo;

    MpQuicHelper mpquicHelper;
    Ptr<MpQuicSocketBase> mpquic = mpquicHelper.InstallWithTwoPaths(
        vehicleNodes.Get(0),
        InetSocketAddress(remoteUuIp,  rxPort),   // path 0: NR Uu → remoteHost:4000
        InetSocketAddress(remotePC5Ip, rxPort),   // path 1: NR SL → RSU → remoteHost:4000
        sched);

    // Single receiver on remoteHost:rxPort collects from both paths (0.0.0.0 bind).
    auto rxApp = CreateObject<MpQuicReceiverApp>();
    rxApp->SetAttribute("Port", UintegerValue(rxPort));
    remoteHost->AddApplication(rxApp);
    rxApp->SetStartTime(appStart);
    rxApp->SetStopTime(simTime);

    // Sender on vehicle
    auto senderApp = CreateObject<MpQuicSenderApp>();
    senderApp->SetMpQuic(mpquic);
    senderApp->SetPacketSize(pktSize);
    senderApp->SetInterval(pktInterval);
    vehicleNodes.Get(0)->AddApplication(senderApp);
    senderApp->SetStartTime(appStart);
    senderApp->SetStopTime(simTime);

    // -----------------------------------------------------------------------
    // NR Uu PHY trace sources (RSRP, SINR)
    // -----------------------------------------------------------------------
    {
        auto nrUeDev = vehicleNrDev.Get(0)->GetObject<NrUeNetDevice>();
        auto uePhy   = nrUeDev->GetPhy(0);
        uePhy->TraceConnectWithoutContext("ReportRsrp", MakeCallback(&CbRsrp));
        uePhy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&CbSinr));
    }

    // -----------------------------------------------------------------------
    // Telemetry CSV
    // -----------------------------------------------------------------------
    const Time      snapshotInterval = MilliSeconds(100);
    const std::string csvPath        = outputDir + "/" + simTag + "_telemetry.csv";
    g_csv.open(csvPath, std::ofstream::out | std::ofstream::trunc);
    g_csv << "time_s,uu_rsrp_dBm,uu_sinr_dB,rsu_dist_m,rsu_angle_deg,"
          << "queue_depth,total_rx_pkts,gamma0\n";

    Simulator::Schedule(snapshotInterval, &TakeSnapshot,
                        mpquic,
                        rxApp,
                        vehicleNodes.Get(0)->GetObject<MobilityModel>(),
                        rsuNodes.Get(0)->GetObject<MobilityModel>(),
                        snapshotInterval,
                        simTime);

    // -----------------------------------------------------------------------
    // Run
    // -----------------------------------------------------------------------
    std::cout << "\n=== NR Uu + NR SL PC5 + MPQUIC Test ===\n"
              << "  Vehicle start:    (" << vehicleX << ", 0, 1.5) m  v=" << vehicleSpeed << " m/s\n"
              << "  RSU position:     (" << rsuX << ", " << rsuY << ", " << rsuZ << ") m\n"
              << "  gNB position:     (0, 0, 10) m\n"
              << "  Vehicle Uu IP:    " << vehicleUuIp.GetAddress(0) << "  (NR Uu)\n"
              << "  Vehicle SL IP:    " << vehicleSlIp.GetAddress(0) << "  (NR SL)\n"
              << "  RSU SL IP:        " << rsuSlIp.GetAddress(0) << "\n"
              << "  RemoteHost (Uu):  " << remoteUuIp << ":4000\n"
              << "  RemoteHost (SL):  " << remotePC5Ip << ":4000\n"
              << "  Scheduler:        " << schedulerType << "\n"
              << "  NR SL HARQ:       " << (slHarq ? "enabled" : "disabled") << "\n"
              << "  Pkt interval:     " << pktInterval.GetMilliSeconds() << " ms ("
              << static_cast<int>(1000.0 / pktInterval.GetMilliSeconds()) << " pps)\n"
              << "  Sim time:         " << simTime.GetSeconds() << " s\n"
              << "  Telemetry CSV:    " << csvPath << "\n\n";

    Simulator::Stop(simTime);
    Simulator::Run();

    // -----------------------------------------------------------------------
    // Results
    // -----------------------------------------------------------------------
    mpquic->PrintStats();

    const auto& paths = mpquic->GetPaths();
    uint64_t uuTx = 0, slTx = 0;
    for (const auto& p : paths)
    {
        if (p->GetPathType() == MpQuicPathType::UU)  uuTx = p->GetTxPackets();
        if (p->GetPathType() == MpQuicPathType::PC5) slTx = p->GetTxPackets();
    }

    std::cout << "\n=== MPQUIC Summary ===\n";
    std::cout << "  Packets sent (app): " << senderApp->GetTotalPacketsSent() << "\n";
    std::cout << "  Total Rx packets:   " << rxApp->GetTotalRxPackets()       << "\n";
    std::cout << "  Γ stream 0:         " << rxApp->GetContinuity(0)          << "\n";
    std::cout << "  Delivery rate:      "
              << (senderApp->GetTotalPacketsSent() > 0
                    ? 100.0 * rxApp->GetTotalRxPackets() / senderApp->GetTotalPacketsSent()
                    : 0.0)
              << " %\n";
    std::cout << "  Uu  TX:             " << uuTx << " pkts\n";
    std::cout << "  NR SL TX:           " << slTx << " pkts\n";
    std::cout << "  Total retx:         " << mpquic->GetTotalRetxPackets()    << "\n";
    std::cout << "  Queue depth:        " << mpquic->GetTotalQueueDepth()     << "\n";

    g_csv.close();

    Simulator::Destroy();
    return EXIT_SUCCESS;
}
