// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file cttc-nr-v2x-wifi-mpquic.cc
 * @brief NR Uu + WiFi-based PC5 + MPQUIC heterogeneous uplink test.
 *
 * Step 1: Verify that real NR Uu (gNB) + WiFi PC5 (802.11p approximation) +
 * MPQUIC co-exist and produce traffic on both paths.
 *
 * Topology:
 *
 *   [gNB (0,0,10)]
 *       |  NR Uu (n78, 3.5 GHz)
 *   [Vehicle (50,0,1.5)] ---(WiFi 802.11p, 5.86 GHz, 3GPP TR 37.885)---> [RSU (200,0,5)]
 *       |                                                                       |
 *       | path 0: Vehicle → NR Uu → EPC PGW → remoteHost:4000                 | backhaul P2P
 *       | path 1: Vehicle → WiFi PC5 → RSU → remoteHost:5000                  |
 *       +-----------------------------------------------------------------------+
 *                                                              [remoteHost]
 *
 * MPQUIC on vehicle sends on both paths simultaneously (RoundRobin by default).
 * MpQuicReceiverApp on remoteHost listens on both ports.
 *
 * Telemetry CSV: columns per 100ms snapshot
 *   time_s, uu_rsrp_dBm, uu_sinr_dB, pc5_rssi_dBm, pc5_snr_dB,
 *   rsu_dist_m, rsu_angle_deg, queue_depth, uu_rx_pkts, pc5_rx_pkts, gamma0
 *
 * Usage:
 *   ./ns3 run "cttc-nr-v2x-wifi-mpquic"
 *   ./ns3 run "cttc-nr-v2x-wifi-mpquic --simTime=5 --schedulerType=MinRtt --logging=true"
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mpquic-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrV2xWifiMpquic");

// ---------------------------------------------------------------------------
// Global signal quality telemetry (updated by trace callbacks)
// ---------------------------------------------------------------------------
double g_rsrpDbm{-140.0};    ///< NR UE PHY RSRP [dBm]
double g_sinrDb{-10.0};      ///< NR UE PHY DL SINR [dB]
double g_pc5RssiDbm{-140.0}; ///< PC5 received power at RSU [dBm]
double g_pc5SnrDb{-10.0};    ///< PC5 SNR at RSU [dB]

void CbRsrp(uint16_t, uint16_t, uint16_t, double rsrp, uint8_t)  { g_rsrpDbm = rsrp; }
void CbSinr(uint16_t, uint16_t, double sinr, uint16_t)
{
    g_sinrDb = (sinr > 0.0) ? 10.0 * std::log10(sinr) : -100.0;
}
void CbPc5Rssi(Ptr<const Packet>, uint16_t, WifiTxVector, MpduInfo,
               SignalNoiseDbm sn, uint16_t)
{
    g_pc5RssiDbm = sn.signal;
    g_pc5SnrDb   = sn.signal - sn.noise;
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
          << "," << g_pc5RssiDbm
          << "," << g_pc5SnrDb
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
    bool        logging         = false;
    Time        simTime         = Seconds(5.0);
    Time        appStart        = MilliSeconds(500);
    std::string outputDir       = ".";
    std::string simTag          = "wifi-mpquic";

    // Vehicle
    double vehicleX      = 50.0;   // m — initial position along road
    double vehicleSpeed  = 15.0;   // m/s along +x (highway speed ~55 km/h)

    // RSU
    double rsuX = 300.0;   // m — position of RSU along road
    double rsuY = 0.0;
    double rsuZ = 5.0;     // m — RSU antenna height

    // NR Uu (Korea 5G NR, n78, sub-6 GHz)
    double   uuFreqHz     = 3.5e9;   // Hz
    double   uuBandwidth  = 20e6;    // Hz (20 MHz)
    uint16_t uuNumerology = 1;       // μ=1: 30 kHz SCS, 0.5 ms slot
    double   uuTxPowerDbm = 23.0;    // dBm (UE uplink, 3GPP NR UE reference power TS 38.101)

    // PC5 WiFi (Korea ITS: ch172 = 5.86 GHz, 10 MHz BW, 23 dBm)
    uint16_t    pc5ChannelNum  = 172;
    double      pc5FreqHz      = 5.86e9;
    double      pc5TxPowerDbm  = 23.0;
    bool        pc5Shadowing   = false;
    bool        pc5ForceLos   = true;       // use AlwaysLosChannelConditionModel (no stochastic blocking)
    std::string pc5Scenario    = "Urban";   // Urban | Highway

    // MPQUIC — both paths deliver to the SAME port on remoteHost so the single
    // MpQuicReceiverApp can reassemble the full stream and compute Γ correctly.
    uint16_t    rxPort         = 4000;   // single server port for both paths
    std::string schedulerType  = "RoundRobin";
    uint32_t    pktSize        = 1200;   // bytes
    Time        pktInterval    = MilliSeconds(2);  // 500 pps total → 250 pps/path

    CommandLine cmd(__FILE__);
    cmd.AddValue("logging",       "Enable ns-3 logging",            logging);
    cmd.AddValue("simTime",       "Simulation duration (s)",         simTime);
    cmd.AddValue("appStart",      "App start time (s)",              appStart);
    cmd.AddValue("outputDir",     "Output directory",                outputDir);
    cmd.AddValue("simTag",        "Simulation tag (file prefix)",    simTag);
    cmd.AddValue("vehicleX",      "Vehicle initial X (m)",           vehicleX);
    cmd.AddValue("vehicleSpeed",  "Vehicle speed m/s",               vehicleSpeed);
    cmd.AddValue("rsuX",          "RSU X position (m)",              rsuX);
    cmd.AddValue("rsuY",          "RSU Y position (m)",              rsuY);
    cmd.AddValue("rsuZ",          "RSU Z position (m)",              rsuZ);
    cmd.AddValue("uuFreqHz",       "NR Uu center frequency (Hz)",     uuFreqHz);
    cmd.AddValue("uuBandwidth",   "NR Uu bandwidth (Hz)",            uuBandwidth);
    cmd.AddValue("uuNumerology",  "NR Uu numerology",                uuNumerology);
    cmd.AddValue("uuTxPowerDbm",  "NR UE uplink TX power (dBm)",     uuTxPowerDbm);
    cmd.AddValue("pc5TxPowerDbm", "PC5 Tx power (dBm)",              pc5TxPowerDbm);
    cmd.AddValue("pc5ForceLos",   "Force LoS channel condition (no stochastic blocking)", pc5ForceLos);
    cmd.AddValue("pc5Scenario",   "V2V scenario: Urban|Highway",     pc5Scenario);
    cmd.AddValue("schedulerType",
                 "MPQUIC scheduler: RoundRobin|MinRtt|Redundant|Peekaboo",
                 schedulerType);
    cmd.AddValue("pktSize",       "MPQUIC packet size (B)",          pktSize);
    cmd.AddValue("pktInterval",   "MPQUIC send interval (ms)",       pktInterval);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(simTime <= appStart, "simTime must be > appStart");
    NS_ABORT_MSG_IF(pc5Scenario != "Urban" && pc5Scenario != "Highway",
                    "pc5Scenario must be Urban or Highway");

    if (logging)
    {
        LogComponentEnable("CttcNrV2xWifiMpquic",  LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicReceiverApp",     LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicSenderApp",       LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicSocketBase",      LOG_LEVEL_DEBUG);
        LogComponentEnable("MpQuicScheduler",       LOG_LEVEL_DEBUG);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // -----------------------------------------------------------------------
    // Nodes
    // -----------------------------------------------------------------------
    NodeContainer gnbNodes;     gnbNodes.Create(1);
    NodeContainer vehicleNodes; vehicleNodes.Create(1);
    NodeContainer rsuNodes;     rsuNodes.Create(1);
    NodeContainer remoteHostNC; remoteHostNC.Create(1);
    Ptr<Node> remoteHost = remoteHostNC.Get(0);

    // -----------------------------------------------------------------------
    // Mobility
    // -----------------------------------------------------------------------
    // gNB: fixed at road centre, 10 m height
    MobilityHelper gnbMob;
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, 10.0));
    gnbMob.SetPositionAllocator(gnbPos);
    gnbMob.Install(gnbNodes);

    // Vehicle: constant velocity along +x (highway scenario)
    MobilityHelper vehMob;
    vehMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    Ptr<ListPositionAllocator> vehPos = CreateObject<ListPositionAllocator>();
    vehPos->Add(Vector(vehicleX, 0.0, 1.5));
    vehMob.SetPositionAllocator(vehPos);
    vehMob.Install(vehicleNodes);
    vehicleNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(vehicleSpeed, 0.0, 0.0));

    // RSU: fixed road-side
    MobilityHelper rsuMob;
    rsuMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> rsuPos = CreateObject<ListPositionAllocator>();
    rsuPos->Add(Vector(rsuX, rsuY, rsuZ));
    rsuMob.SetPositionAllocator(rsuPos);
    rsuMob.Install(rsuNodes);

    // remoteHost: fixed, off-road (placement irrelevant for wired links)
    MobilityHelper rhMob;
    rhMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> rhPos = CreateObject<ListPositionAllocator>();
    rhPos->Add(Vector(0.0, 0.0, 0.0));
    rhMob.SetPositionAllocator(rhPos);
    rhMob.Install(remoteHostNC);

    // -----------------------------------------------------------------------
    // Internet stack
    // -----------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(vehicleNodes);
    internet.Install(rsuNodes);
    internet.Install(remoteHostNC);  // remoteHost

    // -----------------------------------------------------------------------
    // NR Uu path: gNB <-> Vehicle UE
    // -----------------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper>  epcHelper  = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper>   bfHelper   = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper>                 nrHelper   = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Single BWP at uuFreqHz
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(uuFreqHz, uuBandwidth, 1,
                                                    BandwidthPartInfo::UMa);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&band);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    Packet::EnableChecking();
    Packet::EnablePrinting();

    bfHelper->SetAttribute("BeamformingMethod",
                           TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // UE antenna
    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    // gNB antenna
    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    NetDeviceContainer gnbNetDev     = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer vehicleNrDev  = nrHelper->InstallUeDevice(vehicleNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev,    randomStream);
    randomStream += nrHelper->AssignStreams(vehicleNrDev, randomStream);

    // gNB PHY config
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(uuNumerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(40.0));  // gNB DL power 40 dBm

    // UE PHY config — default UE TX power is 2 dBm which causes uplink loss.
    // Set to 23 dBm (3GPP NR UE reference power, TS 38.101).
    nrHelper->GetUePhy(vehicleNrDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(uuTxPowerDbm));

    DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0))->UpdateConfig();
    DynamicCast<NrUeNetDevice>(vehicleNrDev.Get(0))->UpdateConfig();

    // -----------------------------------------------------------------------
    // EPC: PGW <-> remoteHost (P2P, 100 Gbps)
    // -----------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu",      UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(0)));
    NetDeviceContainer pgwRhDevs = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4Routing;

    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwRhIfaces = ipv4h.Assign(pgwRhDevs);
    // pgw=1.0.0.1, remoteHost=1.0.0.2

    // remoteHost needs a route back to UE subnet (7.0.0.0/8 = EPC UE pool)
    Ptr<Ipv4StaticRouting> rhRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    rhRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"),
                               pgwRhIfaces.GetAddress(0),  // via pgw (1.0.0.1)
                               1);

    const Ipv4Address remoteUuIp = pgwRhIfaces.GetAddress(1);  // 1.0.0.2

    // Assign UE IP and attach to gNB
    Ipv4InterfaceContainer vehicleUuIp =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(vehicleNrDev));

    // UE default route via EPC gateway
    Ptr<Ipv4StaticRouting> vehicleRoute =
        ipv4Routing.GetStaticRouting(vehicleNodes.Get(0)->GetObject<Ipv4>());
    vehicleRoute->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // Attach vehicle UE to closest gNB
    nrHelper->AttachToClosestEnb(vehicleNrDev, gnbNetDev);

    // -----------------------------------------------------------------------
    // PC5 path: Vehicle <-(WiFi 802.11p)-> RSU <-(P2P backhaul)-> remoteHost
    //
    // Channel: 3GPP TR 37.885 V2V propagation model
    //          Korea ITS: ch172 (5.86 GHz), 10 MHz BW, 23 dBm
    // -----------------------------------------------------------------------

    // TR 37.885 V2V channel model
    Ptr<ThreeGppPropagationLossModel> v2vLossModel;
    Ptr<ChannelConditionModel>        condModel;

    // Channel condition: force LoS (no stochastic blocking) for first-pass testing,
    // or use probabilistic V2V model (Urban/Highway) for realistic evaluation.
    if (pc5ForceLos)
    {
        condModel = CreateObject<AlwaysLosChannelConditionModel>();
    }
    else if (pc5Scenario == "Highway")
    {
        auto cond = CreateObject<ProbabilisticV2vHighwayChannelConditionModel>();
        cond->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));
        condModel = cond;
    }
    else  // Urban (default)
    {
        auto cond = CreateObject<ProbabilisticV2vUrbanChannelConditionModel>();
        cond->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));
        condModel = cond;
    }

    if (pc5Scenario == "Highway")
    {
        auto loss = CreateObject<ThreeGppV2vHighwayPropagationLossModel>();
        v2vLossModel = loss;
    }
    else  // Urban
    {
        auto loss = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
        v2vLossModel = loss;
    }

    v2vLossModel->SetAttribute("Frequency",             DoubleValue(pc5FreqHz));
    v2vLossModel->SetAttribute("ShadowingEnabled",      BooleanValue(pc5Shadowing));
    v2vLossModel->SetAttribute("ChannelConditionModel", PointerValue(condModel));

    auto v2vChannel = CreateObject<YansWifiChannel>();
    v2vChannel->SetPropagationLossModel(v2vLossModel);
    v2vChannel->SetPropagationDelayModel(
        CreateObject<ConstantSpeedPropagationDelayModel>());

    // PHY: 802.11p at Korea ITS ch172
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(v2vChannel);
    wifiPhy.Set("TxPowerStart",   DoubleValue(pc5TxPowerDbm));
    wifiPhy.Set("TxPowerEnd",     DoubleValue(pc5TxPowerDbm));
    wifiPhy.Set("ChannelSettings",
                StringValue("{" + std::to_string(pc5ChannelNum) + ", 10, BAND_5GHZ, 0}"));
    wifiPhy.Set("RxSensitivity",  DoubleValue(-94.0));  // NR-V2X QPSK reference

    // MAC: ad-hoc (single vehicle, no MAC contention in this scenario)
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode",    StringValue("OfdmRate27MbpsBW10MHz"),
                                  "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));

    NodeContainer pc5Nodes;
    pc5Nodes.Add(vehicleNodes.Get(0));
    pc5Nodes.Add(rsuNodes.Get(0));
    NetDeviceContainer pc5Devs = wifi.Install(wifiPhy, wifiMac, pc5Nodes);

    // Connect PC5 signal quality trace to RSU WiFi PHY
    {
        auto rsuWifiDev = DynamicCast<WifiNetDevice>(pc5Devs.Get(1));
        rsuWifiDev->GetPhy()->TraceConnectWithoutContext(
            "MonitorSnifferRx", MakeCallback(&CbPc5Rssi));
    }

    // IP addressing for WiFi + RSU backhaul
    ipv4h.SetBase("10.10.1.0", "255.255.255.0");
    Ipv4InterfaceContainer pc5Ifaces = ipv4h.Assign(pc5Devs);
    // vehicle WiFi=10.10.1.1, RSU WiFi=10.10.1.2

    // RSU backhaul: RSU <-> remoteHost (P2P, 1 Gbps)
    PointToPointHelper backhaulP2p;
    backhaulP2p.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    backhaulP2p.SetChannelAttribute("Delay",   StringValue("1ms"));
    NetDeviceContainer backhaulDevs = backhaulP2p.Install(rsuNodes.Get(0), remoteHost);

    ipv4h.SetBase("10.10.2.0", "255.255.255.0");
    Ipv4InterfaceContainer backhaulIfaces = ipv4h.Assign(backhaulDevs);
    // RSU P2P=10.10.2.1, remoteHost=10.10.2.2
    const Ipv4Address remotePC5Ip = backhaulIfaces.GetAddress(1);  // 10.10.2.2

    // -----------------------------------------------------------------------
    // Static routing for PC5 path
    // -----------------------------------------------------------------------

    // Vehicle: route to remotePC5Ip via RSU WiFi (next hop 10.10.1.2)
    const uint32_t vehWifiIf =
        vehicleNodes.Get(0)->GetObject<Ipv4>()
                           ->GetInterfaceForDevice(pc5Devs.Get(0));
    vehicleRoute->AddHostRouteTo(remotePC5Ip,
                                 pc5Ifaces.GetAddress(1),  // 10.10.1.2 (RSU WiFi)
                                 vehWifiIf);

    // RSU: route to remotePC5Ip via backhaul P2P
    Ptr<Ipv4StaticRouting> rsuRoute =
        ipv4Routing.GetStaticRouting(rsuNodes.Get(0)->GetObject<Ipv4>());
    const uint32_t rsuBackhaulIf =
        rsuNodes.Get(0)->GetObject<Ipv4>()
                       ->GetInterfaceForDevice(backhaulDevs.Get(0));
    rsuRoute->AddHostRouteTo(remotePC5Ip,
                             backhaulIfaces.GetAddress(1),  // 10.10.2.2 (remoteHost)
                             rsuBackhaulIf);

    // RSU: route ACKs back to vehicle WiFi subnet (10.10.1.0/24)
    const uint32_t rsuWifiIf =
        rsuNodes.Get(0)->GetObject<Ipv4>()
                       ->GetInterfaceForDevice(pc5Devs.Get(1));
    rsuRoute->AddNetworkRouteTo(Ipv4Address("10.10.1.0"), Ipv4Mask("255.255.255.0"),
                                rsuWifiIf);

    // remoteHost: route ACKs to vehicle WiFi subnet via RSU backhaul
    const uint32_t rhBackhaulIf =
        remoteHost->GetObject<Ipv4>()
                  ->GetInterfaceForDevice(backhaulDevs.Get(1));
    rhRoute->AddNetworkRouteTo(Ipv4Address("10.10.1.0"), Ipv4Mask("255.255.255.0"),
                               backhaulIfaces.GetAddress(0),  // via RSU (10.10.2.1)
                               rhBackhaulIf);

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
        InetSocketAddress(remoteUuIp,  rxPort),   // path 0: NR Uu → remoteHost:rxPort
        InetSocketAddress(remotePC5Ip, rxPort),   // path 1: WiFi PC5 → RSU → remoteHost:rxPort
        sched);

    // Single receiver on remoteHost (0.0.0.0:rxPort) collects from both paths.
    // This allows proper stream reassembly and correct Γ computation.
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
    // NR UE PHY trace sources (RSRP, SINR)
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
    const Time snapshotInterval = MilliSeconds(100);
    const std::string csvPath   = outputDir + "/" + simTag + "_telemetry.csv";
    g_csv.open(csvPath, std::ofstream::out | std::ofstream::trunc);
    g_csv << "time_s,uu_rsrp_dBm,uu_sinr_dB,pc5_rssi_dBm,pc5_snr_dB,"
          << "rsu_dist_m,rsu_angle_deg,queue_depth,total_rx_pkts,gamma0\n";

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
    std::cout << "\n=== NR Uu + WiFi PC5 + MPQUIC Test ===\n"
              << "  Vehicle start:  (" << vehicleX << ", 0, 1.5) m  v=" << vehicleSpeed << " m/s\n"
              << "  RSU position:   (" << rsuX << ", " << rsuY << ", " << rsuZ << ") m\n"
              << "  gNB position:   (0, 0, 10) m\n"
              << "  Vehicle Uu IP:  " << vehicleUuIp.GetAddress(0) << "\n"
              << "  Vehicle WiFi:   " << pc5Ifaces.GetAddress(0) << "\n"
              << "  RemoteHost Uu:  " << remoteUuIp << "\n"
              << "  RemoteHost PC5: " << remotePC5Ip << "\n"
              << "  Scheduler:      " << schedulerType << "\n"
              << "  PC5 scenario:   " << pc5Scenario << "\n"
              << "  Sim time:       " << simTime.GetSeconds() << " s\n"
              << "  Telemetry CSV:  " << csvPath << "\n\n";

    Simulator::Stop(simTime);
    Simulator::Run();

    // -----------------------------------------------------------------------
    // Results
    // -----------------------------------------------------------------------
    mpquic->PrintStats();

    const auto& paths = mpquic->GetPaths();
    uint64_t uuTx=0, pc5Tx=0;
    for (const auto& p : paths)
    {
        if (p->GetPathType() == MpQuicPathType::UU)   uuTx  = p->GetTxPackets();
        if (p->GetPathType() == MpQuicPathType::PC5)  pc5Tx = p->GetTxPackets();
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
    std::cout << "  Uu  TX:             " << uuTx  << " pkts\n";
    std::cout << "  PC5 TX:             " << pc5Tx << " pkts\n";
    std::cout << "  Total retx:         " << mpquic->GetTotalRetxPackets()    << "\n";
    std::cout << "  Queue depth:        " << mpquic->GetTotalQueueDepth()     << "\n";

    g_csv.close();

    Simulator::Destroy();
    return EXIT_SUCCESS;
}
