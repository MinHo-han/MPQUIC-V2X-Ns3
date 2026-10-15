// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file cttc-nr-v2x-mpquic-test.cc
 * @brief Minimal NR-V2X PC5 sidelink + MPQUIC link test.
 *
 * Topology:
 *
 *   [Vehicle (10.1.1.1, 7.0.0.2)] --WiFi--> [UuProxy (10.1.1.2)] --P2P--> [remoteHost (10.1.2.2)]
 *   [Vehicle (7.0.0.2)]           --NR SL-> [RSU UE (7.0.0.3)]
 *
 * MPQUIC on the vehicle uses:
 *   Path 0 (Uu proxy): vehicle → WiFi → uuProxy → P2P → remoteHost:4000
 *   Path 1 (NR SL PC5): vehicle → NR sidelink → RSU UE:5000  (direct, no relay)
 *
 * ACK routing:
 *   Path 0: remoteHost (10.1.2.2) → uuProxy (10.1.2.1) → vehicle WiFi (10.1.1.1)
 *   Path 1: RSU UE (7.0.0.3) → NR SL → vehicle SL (7.0.0.2)
 *
 * Usage:
 *   ./ns3 run "cttc-nr-v2x-mpquic-test"
 *   ./ns3 run "cttc-nr-v2x-mpquic-test --simTime=5 --schedulerType=MinRtt --logging=true"
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
#include "ns3/wifi-module.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrV2xMpquicTest");

int
main(int argc, char* argv[])
{
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    bool        logging         = false;
    Time        simTime         = Seconds(10.0);
    Time        appStart        = MilliSeconds(500);
    double      interUeDist     = 20.0;   // m: vehicle <-> RSU distance

    // NR SL PC5 (Korea ITS band n47 / ch172 vicinity)
    double      slFreqHz        = 5.89e9;
    uint16_t    slBwRb          = 400;    // 40 MHz in units of 100 kHz
    uint16_t    slNumerology    = 2;
    double      slTxPowerDbm    = 23.0;
    bool        slHarq          = true;

    // MPQUIC
    uint16_t    uuPort          = 4000;
    uint16_t    pc5Port         = 5000;
    std::string schedulerType   = "RoundRobin";

    CommandLine cmd(__FILE__);
    cmd.AddValue("logging",       "Enable ns-3 logging",         logging);
    cmd.AddValue("simTime",       "Simulation duration (s)",      simTime);
    cmd.AddValue("appStart",      "App start time (s)",           appStart);
    cmd.AddValue("interUeDist",   "Vehicle-RSU distance (m)",     interUeDist);
    cmd.AddValue("slTxPowerDbm",  "PC5 Tx power (dBm)",           slTxPowerDbm);
    cmd.AddValue("slHarq",        "Enable NR SL HARQ",            slHarq);
    cmd.AddValue("schedulerType", "RoundRobin|MinRtt|Redundant|Peekaboo", schedulerType);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(simTime <= appStart, "simTime must be > appStart");

    if (logging)
    {
        LogComponentEnable("CttcNrV2xMpquicTest", LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicSocketBase",    LOG_LEVEL_DEBUG);
        LogComponentEnable("MpQuicScheduler",     LOG_LEVEL_DEBUG);
        LogComponentEnable("MpQuicReceiverApp",   LOG_LEVEL_INFO);
        LogComponentEnable("NrSlHelper",          LOG_LEVEL_INFO);
        LogComponentEnable("NrSlUeRrc",           LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // -------------------------------------------------------------------------
    // Nodes
    // -------------------------------------------------------------------------
    NodeContainer vehicleNode;   vehicleNode.Create(1);
    NodeContainer rsuSlNode;     rsuSlNode.Create(1);     // NR SL peer (RSU UE)
    NodeContainer uuProxyNode;   uuProxyNode.Create(1);   // WiFi proxy for Uu path
    NodeContainer remoteHostNC;  remoteHostNC.Create(1);
    Ptr<Node>     remoteHost = remoteHostNC.Get(0);

    // SL UE container: [0]=vehicle, [1]=RSU
    NodeContainer slUeContainer;
    slUeContainer.Add(vehicleNode.Get(0));
    slUeContainer.Add(rsuSlNode.Get(0));

    // -------------------------------------------------------------------------
    // Mobility
    // -------------------------------------------------------------------------
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0,          0.0, 1.5));   // vehicle
    pos->Add(Vector(interUeDist,  0.0, 5.0));   // RSU SL UE
    pos->Add(Vector(5.0,          5.0, 10.0));  // Uu proxy
    pos->Add(Vector(100.0,        0.0, 0.0));   // remoteHost (off-site)
    mob.SetPositionAllocator(pos);
    mob.Install(vehicleNode);
    mob.Install(rsuSlNode);
    mob.Install(uuProxyNode);
    mob.Install(remoteHostNC);

    // -------------------------------------------------------------------------
    // Internet stack (all nodes)
    // -------------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(vehicleNode);
    internet.Install(rsuSlNode);
    internet.Install(uuProxyNode);
    internet.Install(remoteHostNC);

    // =========================================================================
    // Path 0: Uu proxy
    //   Vehicle --WiFi (10.1.1.x)--> UuProxy --P2P (10.1.2.x)--> remoteHost
    // =========================================================================
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode",    StringValue("HtMcs7"),
                                  "ControlMode", StringValue("HtMcs0"));
    NodeContainer wifiNodes;
    wifiNodes.Add(vehicleNode.Get(0));
    wifiNodes.Add(uuProxyNode.Get(0));
    NetDeviceContainer wifiDevs = wifi.Install(wifiPhy, wifiMac, wifiNodes);
    // iface index after Assign: vehicle=1 (WiFi), uuProxy=1 (WiFi)

    PointToPointHelper uuP2P;
    uuP2P.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    uuP2P.SetChannelAttribute("Delay",   StringValue("1ms"));
    NetDeviceContainer uuBackhaul = uuP2P.Install(uuProxyNode.Get(0), remoteHost);
    // iface: uuProxy=2 (P2P), remoteHost=1 (P2P)

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer wifiIfaces = ipv4h.Assign(wifiDevs);
    // wifiIfaces[0]=10.1.1.1 (vehicle), wifiIfaces[1]=10.1.1.2 (uuProxy)

    ipv4h.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer uuBackhaulIfaces = ipv4h.Assign(uuBackhaul);
    // uuBackhaulIfaces[0]=10.1.2.1 (uuProxy P2P), uuBackhaulIfaces[1]=10.1.2.2 (remoteHost)
    const Ipv4Address remoteUuIp = uuBackhaulIfaces.GetAddress(1);  // 10.1.2.2

    // =========================================================================
    // Path 1: NR SL PC5 (out-of-coverage, no gNB)
    //   Vehicle --NR SL (7.0.0.x)--> RSU UE
    //   MpQuicReceiverApp runs directly on RSU UE (no relay to remoteHost)
    // =========================================================================

    // NR helper + EPC (EPC used only for IP assignment, no gNB)
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Single-BWP SL band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf slBandConf(slFreqHz,
                                                      slBwRb * 100e3,
                                                      1,
                                                      BandwidthPartInfo::V2V_Highway);
    OperationBandInfo slBand = ccBwpCreator.CreateOperationBandContiguousCc(slBandConf);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&slBand);
    BandwidthPartInfoPtrVector slBwps = CcBwpCreator::GetAllBwps({slBand});

    Packet::EnableChecking();
    Packet::EnablePrinting();

    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(slTxPowerDbm));

    uint8_t bwpIdSl = 0;
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing",  BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1",             UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2",             UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId",   UintegerValue(0));
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdSl));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdSl);

    // Install NR SL devices: slNetDev[0]=vehicle, slNetDev[1]=RSU
    NetDeviceContainer slNetDev = nrHelper->InstallUeDevice(slUeContainer, slBwps);

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(slNetDev, stream);

    for (auto it = slNetDev.Begin(); it != slNetDev.End(); ++it)
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // NR SL helper
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);
    nrSlHelper->SetSlErrorModel("ns3::NrEesmIrT1");
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));
    nrSlHelper->PrepareUeForSidelink(slNetDev, bwpIdContainer);

    // Assign UE IPs via EPC FIRST (needed before TFT creation)
    // vehicle: 7.0.0.2, RSU: 7.0.0.3 (EPC assigns sequentially from 7.0.0.2)
    Ipv4InterfaceContainer vehicleSlIp =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(slNetDev.Get(0)));
    Ipv4InterfaceContainer rsuSlIp =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(slNetDev.Get(1)));

    // Resource pool
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
    slPoolCfg.slResourcePoolId  = slPoolId;
    slPoolCfg.slResourcePool    = pool;

    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPool;
    slBwpPool.slTxPoolSelectedNormal[slPoolId.id] = slPoolCfg;

    LteRrcSap::Bwp bwp;
    bwp.numerology      = slNumerology;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg        = 1;
    bwp.bandwidth       = slBwRb;

    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp            = bwp;
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

    // SL bearer activation:
    // Use separate TFTs so each UE matches its PEER's SL IP.
    //   vehicle TFT: remoteAddr = RSU SL IP (7.0.0.3)
    //   RSU TFT:     remoteAddr = vehicle SL IP (7.0.0.2)
    Time slBearerActivation = appStart - MilliSeconds(10);

    SidelinkInfo slInfo;
    slInfo.m_castType    = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id     = 255;
    slInfo.m_rri         = MilliSeconds(20);  // 20 ms RRI → 50 pps SL capacity
    slInfo.m_pdb         = Seconds(0);
    slInfo.m_harqEnabled = slHarq;

    // Vehicle: sends to / receives from RSU SL IP
    Ptr<LteSlTft> vehTft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                             rsuSlIp.GetAddress(0),
                                             slInfo);
    nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                   NetDeviceContainer(slNetDev.Get(0)),
                                   vehTft);

    // RSU: sends to / receives from vehicle SL IP
    Ptr<LteSlTft> rsuTft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                             vehicleSlIp.GetAddress(0),
                                             slInfo);
    nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                   NetDeviceContainer(slNetDev.Get(1)),
                                   rsuTft);

    // =========================================================================
    // Routing
    // =========================================================================
    Ipv4StaticRoutingHelper ipv4Routing;

    // --- Path 0 forward: vehicle → remoteHost via uuProxy ---
    // Vehicle: route to 10.1.2.2 → via 10.1.1.2 (uuProxy WiFi), iface 1
    Ptr<Ipv4StaticRouting> vehicleRoute =
        ipv4Routing.GetStaticRouting(vehicleNode.Get(0)->GetObject<Ipv4>());
    vehicleRoute->AddHostRouteTo(remoteUuIp,
                                 wifiIfaces.GetAddress(1),  // 10.1.1.2 (uuProxy WiFi)
                                 1);                        // vehicle WiFi iface

    // UuProxy: route to 10.1.2.2 → via P2P iface 2
    Ptr<Ipv4StaticRouting> proxyRoute =
        ipv4Routing.GetStaticRouting(uuProxyNode.Get(0)->GetObject<Ipv4>());
    proxyRoute->AddHostRouteTo(remoteUuIp,
                               uuBackhaulIfaces.GetAddress(1),  // 10.1.2.2 (remoteHost P2P)
                               2);                              // uuProxy P2P iface

    // --- Path 0 reverse (ACK): remoteHost → vehicle WiFi (10.1.1.x) ---
    // remoteHost needs route to 10.1.1.0/24 via uuProxy (10.1.2.1), iface 1
    Ptr<Ipv4StaticRouting> remoteRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteRoute->AddNetworkRouteTo(Ipv4Address("10.1.1.0"),
                                   Ipv4Mask("255.255.255.0"),
                                   uuBackhaulIfaces.GetAddress(0),  // 10.1.2.1 (uuProxy P2P)
                                   1);                              // remoteHost P2P iface
    // uuProxy auto-forwards 10.1.1.x via its WiFi iface (connected subnet) ✓

    // --- Path 1: vehicle SL (7.0.0.2) <-> RSU SL (7.0.0.3) ---
    // Both are in 7.0.0.0/8 subnet on their NR SL interface.
    // Connected routes are auto-added by EPC IP assignment → no explicit routes needed.
    // SL bearer TFTs handle the NR SL forwarding.

    // =========================================================================
    // MPQUIC
    // =========================================================================
    MpQuicHelper::SchedulerType sched = MpQuicHelper::SchedulerType::RoundRobin;
    if      (schedulerType == "MinRtt")    sched = MpQuicHelper::SchedulerType::MinRtt;
    else if (schedulerType == "Redundant") sched = MpQuicHelper::SchedulerType::Redundant;
    else if (schedulerType == "Peekaboo")  sched = MpQuicHelper::SchedulerType::Peekaboo;

    MpQuicHelper mpquicHelper;
    Ptr<MpQuicSocketBase> mpquic = mpquicHelper.InstallWithTwoPaths(
        vehicleNode.Get(0),
        InetSocketAddress(remoteUuIp,              uuPort),   // path 0: via WiFi → remoteHost
        InetSocketAddress(rsuSlIp.GetAddress(0),   pc5Port),  // path 1: via NR SL → RSU UE
        sched);

    // Receiver: path 0 on remoteHost
    auto uuRxApp = CreateObject<MpQuicReceiverApp>();
    uuRxApp->SetAttribute("Port", UintegerValue(uuPort));
    remoteHost->AddApplication(uuRxApp);
    uuRxApp->SetStartTime(appStart);
    uuRxApp->SetStopTime(simTime);

    // Receiver: path 1 on RSU SL UE (direct NR SL endpoint)
    auto pc5RxApp = CreateObject<MpQuicReceiverApp>();
    pc5RxApp->SetAttribute("Port", UintegerValue(pc5Port));
    rsuSlNode.Get(0)->AddApplication(pc5RxApp);
    pc5RxApp->SetStartTime(appStart);
    pc5RxApp->SetStopTime(simTime);

    // Sender on vehicle
    auto senderApp = CreateObject<MpQuicSenderApp>();
    senderApp->SetMpQuic(mpquic);
    senderApp->SetPacketSize(1200);
    senderApp->SetInterval(MilliSeconds(40));  // 25 pps → each path gets 12.5 pps, fits NR SL SPS (20ms)
    vehicleNode.Get(0)->AddApplication(senderApp);
    senderApp->SetStartTime(appStart);
    senderApp->SetStopTime(simTime);

    // =========================================================================
    // Run
    // =========================================================================
    std::cout << "\n=== NR V2X PC5 + MPQUIC Test ===\n"
              << "  Path 0 (Uu proxy): vehicle → WiFi → uuProxy → remoteHost:" << uuPort  << "\n"
              << "                     vehicle WiFi=" << wifiIfaces.GetAddress(0)
              << "  remoteHost=" << remoteUuIp << "\n"
              << "  Path 1 (NR SL):    vehicle → NR-PC5 → RSU UE:" << pc5Port << "\n"
              << "                     vehicle SL=" << vehicleSlIp.GetAddress(0)
              << "  RSU SL=" << rsuSlIp.GetAddress(0) << "\n"
              << "  Scheduler:         " << schedulerType << "\n"
              << "  NR SL HARQ:        " << (slHarq ? "enabled" : "disabled") << "\n"
              << "  Sim time:          " << simTime.GetSeconds() << " s\n\n";

    Simulator::Stop(simTime);
    Simulator::Run();

    // =========================================================================
    // Results
    // =========================================================================
    mpquic->PrintStats();
    std::cout << "\n=== MPQUIC Summary ===\n";
    std::cout << "  Packets sent:       " << senderApp->GetTotalPacketsSent() << "\n";
    std::cout << "  Uu  Rx (stream0):   " << uuRxApp->GetTotalRxPackets()  << " pkts\n";
    std::cout << "  PC5 Rx (stream0):   " << pc5RxApp->GetTotalRxPackets() << " pkts\n";
    std::cout << "  Uu  Γ(stream0):     " << uuRxApp->GetContinuity(0)    << "\n";
    std::cout << "  PC5 Γ(stream0):     " << pc5RxApp->GetContinuity(0)   << "\n";
    std::cout << "  Total retx:         " << mpquic->GetTotalRetxPackets() << "\n";
    std::cout << "  Queue depth:        " << mpquic->GetTotalQueueDepth()  << "\n";

    Simulator::Destroy();
    return EXIT_SUCCESS;
}
