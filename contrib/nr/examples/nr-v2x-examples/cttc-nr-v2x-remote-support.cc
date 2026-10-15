// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file cttc-nr-v2x-remote-support.cc
 * @brief Urban V2X scenario with remote-support vehicles — NR Uu + NR SL PC5.
 *
 * Two independent uplink paths:
 *   ① NR Uu  — remote-support vehicles send video chunks + metadata
 *               vehicle_uu_ip → gNB → EPC → PGW → server (1.0.0.2)
 *   ② NR SL PC5 — all vehicles send cooperative awareness/hazard messages
 *               vehicle_sl_ip → SL groupcast (dstL2Id=255) → RSU → backhaul → server
 *
 * Topology (urban_large default):
 *   6 gNBs (3×2 grid), 8 RSUs (2 rows), 1 edge server.
 *
 * Vehicle split:
 *   IDs 0 … nRemote-1  → EdgeVideoMetadataApp (video/meta via Uu)
 *                       + CooperativeVehicleApp (awareness/hazard via PC5)
 *   IDs nRemote … N-1  → CooperativeVehicleApp only (awareness/hazard via PC5)
 *
 * Output: per-vehicle CSV logs in OutputDir/vehicles/
 *
 * Usage:
 *   ./ns3 run "cttc-nr-v2x-remote-support --nVehicles=80 --remoteRatio=0.05 \
 *              --traceFile=.../gangnam_large_mobility.tcl --outputDir=./results-5pct"
 */

#include "cooperative-vehicle-app.h"
#include "edge-video-metadata-app.h"
#include "server-sink-app.h"
#include "v2x-headers.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrV2xRemoteSupport");

// ============================================================================
// Load buildings from CSV produced by extract_buildings.py
// CSV header: x_min,x_max,y_min,y_max,z_min,z_max
// Returns number of buildings loaded.
// ============================================================================
static uint32_t
LoadBuildingsFromCsv(const std::string& csvPath, double minAreaM2 = 0.0)
{
    std::ifstream f(csvPath);
    NS_ABORT_MSG_IF(!f.is_open(), "Cannot open buildings CSV: " << csvPath);

    uint32_t count = 0;
    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line))
    {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        double vals[6]{};
        for (int i = 0; i < 6 && std::getline(ss, tok, ','); ++i)
            vals[i] = std::stod(tok);

        // Skip buildings below area threshold (filter tiny buildings for performance)
        if (minAreaM2 > 0.0)
        {
            double area = (vals[1] - vals[0]) * (vals[3] - vals[2]);
            if (area < minAreaM2)
                continue;
        }

        Ptr<Building> b = CreateObject<Building>();
        b->SetBoundaries(Box(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]));
        b->SetBuildingType(Building::Residential);
        b->SetExtWallsType(Building::ConcreteWithWindows);
        ++count;
    }
    return count;
}

// ============================================================================
// Position-based Uu handover helper (bypasses broken NrUePhy measurement chain)
// ============================================================================
struct UuHandoverHelper
{
    uint32_t                    vehIdx;
    Ptr<NrUeNetDevice>          ueNrDev;
    std::vector<Ptr<LteEnbRrc>> gnbRrcs;
    std::vector<uint16_t>       gnbCellIds;
    std::vector<Vector>         gnbPos;
    Time                        lastHoTime{Seconds(-10.0)};
    Ptr<ServerSinkApp>          sinkApp;   ///< if non-null, NotifyHandover() on each HO

    // 30 s cooldown: prevents back-to-back HOs that trigger the ns-3 LTE/EPC
    // Modify-Bearer race.  Patched upstream (DoPathSwitchRequestAcknowledge
    // HasUeManager guard), but the long cooldown is a belt-and-suspenders
    // protection and keeps at most ~10 HOs per 300 s run.
    static constexpr double kCooldownSec     = 30.0;
    static constexpr double kCheckIntervalMs = 1000.0;

    static void TriggerIfNeeded(UuHandoverHelper* h)
    {
        const Time now = Simulator::Now();
        if ((now - h->lastHoTime).GetSeconds() < kCooldownSec)
        {
            Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                                &UuHandoverHelper::TriggerIfNeeded, h);
            return;
        }
        Ptr<MobilityModel> mob = h->ueNrDev->GetNode()->GetObject<MobilityModel>();
        if (!mob)
        {
            Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                                &UuHandoverHelper::TriggerIfNeeded, h);
            return;
        }
        Vector   vp      = mob->GetPosition();
        uint32_t nearest = 0;
        double   bestD2  = 1e36;
        for (uint32_t j = 0; j < h->gnbPos.size(); ++j)
        {
            const double dx = h->gnbPos[j].x - vp.x;
            const double dy = h->gnbPos[j].y - vp.y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; nearest = j; }
        }
        Ptr<LteUeRrc> ueRrc = h->ueNrDev->GetRrc();
        if (!ueRrc)
        {
            Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                                &UuHandoverHelper::TriggerIfNeeded, h);
            return;
        }
        uint16_t curCellId = ueRrc->GetCellId();
        uint16_t rnti      = ueRrc->GetRnti();
        uint16_t tgtCell   = h->gnbCellIds[nearest];
        int32_t srcIdx = -1;
        for (uint32_t j = 0; j < h->gnbCellIds.size(); ++j)
            if (h->gnbCellIds[j] == curCellId) { srcIdx = static_cast<int32_t>(j); break; }
        if (srcIdx >= 0 && static_cast<uint32_t>(srcIdx) != nearest && rnti != 0
            && h->gnbRrcs[srcIdx])
        {
            NS_LOG_INFO("UuHO Veh[" << h->vehIdx << "] gNB[" << srcIdx << "]→gNB[" << nearest
                                    << "] dist=" << std::sqrt(bestD2) << "m");
            h->gnbRrcs[srcIdx]->SendHandoverRequest(rnti, tgtCell);
            h->lastHoTime = now;
            if (h->sinkApp)
                h->sinkApp->NotifyHandover(h->vehIdx);
        }
        Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                            &UuHandoverHelper::TriggerIfNeeded, h);
    }
};

// ============================================================================
// main
// ============================================================================
int
main(int argc, char* argv[])
{
    // ── Parameters ────────────────────────────────────────────────────────────
    uint32_t    nVehicles       = 80;
    double      remoteRatio     = 0.05;
    Time        simTime         = Seconds(120.0);
    Time        appStart        = MilliSeconds(500);
    std::string outputDir       = ".";
    std::string traceFile       = "";
    std::string buildingsFile   = "";  // path to gangnam_buildings.csv; empty = no buildings
    std::string topoMode        = "urban_large";
    bool        logging         = false;
    double      vehicleSpeed    = 22.22;  // m/s; 0 = stationary (for range tests)
    double      minBuildingAreaM2 = 0.0; // 0=load all; >0 skips buildings smaller than this (m²)
    uint32_t    chCondUpdateMs  = 500;   // channel-condition model update period (ms)

    // NR Uu
    double   uuFreqHz       = 3.5e9;
    double   uuBandwidth    = 20e6;
    uint16_t uuNumerology   = 1;
    double   uuTxPowerDbm   = 23.0;
    double   gnbTxPowerDbm  = 40.0;

    // NR SL PC5  (defaults match real V2X equipment)
    // Freq=5875 MHz, BW=20 MHz, SCS=30 kHz, Symbol 0/14
    // sizeSubchannel=10, NumSubchannel=5, MCS=QAM64/20, MaxTxTrans=4, HARQ=off, Tx=20dBm
    double   slFreqHz          = 5.875e9;
    uint16_t slBwRb            = 200;      // bandwidth in 100-kHz units (200=20MHz)
    uint16_t slNumerology      = 1;        // μ: 0=15kHz,1=30kHz,2=60kHz
    double   slTxPowerDbm      = 20.0;
    bool     slHarq            = false;
    uint8_t  slMaxTxTrans      = 4;        // max PSSCH transmissions (1=no retx, 2/4 with HARQ)
    uint16_t slSubchannelSize  = 10;       // RBs per subchannel (200RBs/10=~5 subchannels)
    uint16_t slReservePeriodMs = 0;        // 0 = dynamic (no SPS pre-reservation)
    uint8_t  slMcs             = 20;       // fixed MCS index (20=64QAM ~3.85 bps/Hz)

    // Application parameters
    uint32_t fps                = 15;
    uint32_t avgFrameKB         = 100;
    uint32_t keyframeInterval   = 30;
    double   keyframeMultiplier = 3.0;
    uint32_t metaPayloadBytes   = 512;
    uint32_t chunkSize          = 1200;
    double   frameDeadlineSec   = 0.1;
    double   hazardProb         = 0.01;
    double   pathWeightUu       = 1.0;  // 1.0 = Uu only, 0.0 = PC5 only

    CommandLine cmd(__FILE__);
    cmd.AddValue("nVehicles",         "Total number of vehicles",           nVehicles);
    cmd.AddValue("remoteRatio",       "Fraction that send video (0–1)",     remoteRatio);
    cmd.AddValue("simTime",           "Simulation duration (s)",            simTime);
    cmd.AddValue("appStart",          "Application start time (s)",         appStart);
    cmd.AddValue("outputDir",         "Output directory for CSV logs",      outputDir);
    cmd.AddValue("traceFile",         "SUMO ns-2 mobility trace (.tcl)",    traceFile);
    cmd.AddValue("buildingsFile",     "Buildings CSV from extract_buildings.py (empty=off)", buildingsFile);
    cmd.AddValue("minBuildingAreaM2", "Skip buildings smaller than this area (m²); 0=load all", minBuildingAreaM2);
    cmd.AddValue("chCondUpdateMs",    "Channel-condition model update period (ms)", chCondUpdateMs);
    cmd.AddValue("topoMode",          "urban_large | urban | highway | gangnam | gangnam_large | range_test", topoMode);
    cmd.AddValue("logging",           "Enable ns-3 INFO logging",           logging);
    cmd.AddValue("vehicleSpeed",      "Vehicle speed m/s (0=stationary)",   vehicleSpeed);
    cmd.AddValue("fps",               "Video frames per second",            fps);
    cmd.AddValue("avgFrameKB",        "Average P-frame size (KB)",          avgFrameKB);
    cmd.AddValue("keyframeInterval",  "Frames between I-frames",            keyframeInterval);
    cmd.AddValue("keyframeMultiplier","I-frame size multiplier",            keyframeMultiplier);
    cmd.AddValue("metaPayloadBytes",  "Dummy metadata payload size (B)",    metaPayloadBytes);
    cmd.AddValue("chunkSize",         "Max UDP payload per video chunk (B)", chunkSize);
    cmd.AddValue("frameDeadline",     "Video delivery deadline (s)",        frameDeadlineSec);
    cmd.AddValue("hazardProb",        "Hazard events per vehicle per second", hazardProb);
    cmd.AddValue("slHarq",            "Enable NR SL HARQ",                  slHarq);
    cmd.AddValue("slMaxTxTrans",      "Max PSSCH TX count (1=no retx)",     slMaxTxTrans);
    cmd.AddValue("slSubchannelSize",  "SL subchannel size in RBs",          slSubchannelSize);
    cmd.AddValue("slReservePeriodMs", "SPS resource reserve period (ms)",   slReservePeriodMs);
    cmd.AddValue("slMcs",             "SL fixed MCS index",                 slMcs);
    cmd.AddValue("slNumerology",      "SL BWP numerology (0=15k,1=30k,2=60k)", slNumerology);
    cmd.AddValue("slBwRb",            "SL bandwidth in 100-kHz units",      slBwRb);
    cmd.AddValue("pathWeightUu",      "Fraction of video frames sent via Uu [0,1]", pathWeightUu);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(topoMode != "highway" && topoMode != "urban"
                    && topoMode != "urban_large" && topoMode != "gangnam"
                    && topoMode != "gangnam_large" && topoMode != "range_test",
                    "topoMode must be 'highway', 'urban', 'urban_large', 'gangnam', 'gangnam_large', or 'range_test'");
    NS_ABORT_MSG_IF(remoteRatio < 0.0 || remoteRatio > 1.0, "remoteRatio must be in [0,1]");
    NS_ABORT_MSG_IF(nVehicles == 0, "nVehicles must be > 0");

    // gangnam_large: auto-detect buildings CSV from traceFile directory if not set
    if (topoMode == "gangnam_large" && buildingsFile.empty() && !traceFile.empty())
    {
        std::filesystem::path bfPath =
            std::filesystem::path(traceFile).parent_path() / "gangnam_buildings.csv";
        if (std::filesystem::exists(bfPath))
            buildingsFile = bfPath.string();
        else
            NS_LOG_WARN("gangnam_large: gangnam_buildings.csv not found next to traceFile; "
                        "running without buildings (probabilistic NLOS only)");
    }

    const uint32_t nRemote = static_cast<uint32_t>(std::round(nVehicles * remoteRatio));
    const uint32_t nNormal = nVehicles - nRemote;

    if (logging)
    {
        LogComponentEnable("CttcNrV2xRemoteSupport",  LOG_LEVEL_INFO);
        LogComponentEnable("CooperativeVehicleApp",   LOG_LEVEL_INFO);
        LogComponentEnable("EdgeVideoMetadataApp",    LOG_LEVEL_INFO);
        LogComponentEnable("ServerSinkApp",           LOG_LEVEL_INFO);
    }

    // ── gNB + RSU positions per topoMode ─────────────────────────────────────
    uint32_t nGnbs, nRsus;
    double   gnbPosX[9], gnbPosY[9];
    double   rsuPosX[9], rsuPosY[9];
    constexpr double rsuPosZ = 6.0;

    if (topoMode == "urban_large")
    {
        nGnbs = 6;  nRsus = 8;
        gnbPosX[0]=1200.0; gnbPosY[0]=2500.0;  // SW
        gnbPosX[1]=3600.0; gnbPosY[1]=2500.0;  // S
        gnbPosX[2]=6000.0; gnbPosY[2]=2500.0;  // SE
        gnbPosX[3]=1200.0; gnbPosY[3]=6200.0;  // NW
        gnbPosX[4]=3600.0; gnbPosY[4]=6200.0;  // N
        gnbPosX[5]=6000.0; gnbPosY[5]=6200.0;  // NE
        rsuPosX[0]= 700.0; rsuPosY[0]=1400.0;
        rsuPosX[1]=2400.0; rsuPosY[1]=1400.0;
        rsuPosX[2]=4700.0; rsuPosY[2]=1400.0;
        rsuPosX[3]=6500.0; rsuPosY[3]=1400.0;
        rsuPosX[4]= 700.0; rsuPosY[4]=4800.0;
        rsuPosX[5]=2400.0; rsuPosY[5]=4800.0;
        rsuPosX[6]=4700.0; rsuPosY[6]=4800.0;
        rsuPosX[7]=6500.0; rsuPosY[7]=4800.0;
    }
    else if (topoMode == "urban")
    {
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0]=1500.0; gnbPosY[0]=1610.0;
        gnbPosX[1]=3600.0; gnbPosY[1]=1610.0;
        gnbPosX[2]=1500.0; gnbPosY[2]=3760.0;
        gnbPosX[3]=3600.0; gnbPosY[3]=3760.0;
        rsuPosX[0]=1000.0; rsuPosY[0]=2700.0;
        rsuPosX[1]=2500.0; rsuPosY[1]=2700.0;
        rsuPosX[2]=4200.0; rsuPosY[2]=2700.0;
        rsuPosX[3]=2500.0; rsuPosY[3]=1200.0;
    }
    else if (topoMode == "gangnam")
    {
        // ns-3 area: 5036.68 x 5722.67 m  (SUMO Seoul Gangnam, 15 vehicles)
        // Coordinate origin = SUMO (x_min=4.71, y_min=429.95) → ns-3 (0,0)
        nGnbs = 6;  nRsus = 8;
        gnbPosX[0]=1007.3; gnbPosY[0]=1430.7;
        gnbPosX[1]=2518.3; gnbPosY[1]=1430.7;
        gnbPosX[2]=4029.3; gnbPosY[2]=1430.7;
        gnbPosX[3]=1007.3; gnbPosY[3]=4292.0;
        gnbPosX[4]=2518.3; gnbPosY[4]=4292.0;
        gnbPosX[5]=4029.3; gnbPosY[5]=4292.0;
        rsuPosX[0]= 503.7; rsuPosY[0]= 858.4;
        rsuPosX[1]=2014.7; rsuPosY[1]= 858.4;
        rsuPosX[2]=3525.7; rsuPosY[2]= 858.4;
        rsuPosX[3]=5036.7; rsuPosY[3]= 858.4;
        rsuPosX[4]= 503.7; rsuPosY[4]=3147.5;
        rsuPosX[5]=2014.7; rsuPosY[5]=3147.5;
        rsuPosX[6]=3525.7; rsuPosY[6]=3147.5;
        rsuPosX[7]=5036.7; rsuPosY[7]=3147.5;
    }
    else if (topoMode == "gangnam_large")
    {
        // Seoul Gangnam-gu large scenario: X=56~7025, Y=354~8239 (~7×8 km²)
        // 100 vehicles from gangnam_large_mobility.tcl SUMO trace.
        //
        // RSU/gNB placement rationale (JSAC TiL paper):
        //   • Measured NR SL PC5 effective range: ~850 m (5875 MHz, 20 MHz, 20 dBm, MCS 20)
        //   • RSU inter-site distance (ISD) = 2500 m ≈ 3× effective range
        //   • Follows ETSI TS 102 687 §B.3 sparse urban V2X deployment guideline (ISD ≤ 3×range)
        //   • 3×3 grid → coverage ratio ≈ 28% of total area (π×850²×9 / 56e6 km²)
        //   • Dead zones (>850 m from any RSU) cover ~72% → ensures TiL training data
        //     includes diverse risk states: OK (near RSU), Moderate (mid-range), Severe (dead zone)
        //   • gNBs co-located with RSUs (same XY, height=25 m) per 3GPP TR 22.186 §5.2
        //
        // Grid: 3 columns × 3 rows, ISD=2500 m, centred in SUMO area
        //   X: 1000, 3500, 6000  (within 56~7025)
        //   Y: 1800, 4300, 6800  (within 354~8239)
        nGnbs = 9;  nRsus = 9;
        // Row 0 (Y=1800)
        gnbPosX[0]=1000.0; gnbPosY[0]=1800.0;
        gnbPosX[1]=3500.0; gnbPosY[1]=1800.0;
        gnbPosX[2]=6000.0; gnbPosY[2]=1800.0;
        // Row 1 (Y=4300)
        gnbPosX[3]=1000.0; gnbPosY[3]=4300.0;
        gnbPosX[4]=3500.0; gnbPosY[4]=4300.0;
        gnbPosX[5]=6000.0; gnbPosY[5]=4300.0;
        // Row 2 (Y=6800)
        gnbPosX[6]=1000.0; gnbPosY[6]=6800.0;
        gnbPosX[7]=3500.0; gnbPosY[7]=6800.0;
        gnbPosX[8]=6000.0; gnbPosY[8]=6800.0;
        // RSUs co-located with gNBs (6 m height, set below)
        rsuPosX[0]=1000.0; rsuPosY[0]=1800.0;
        rsuPosX[1]=3500.0; rsuPosY[1]=1800.0;
        rsuPosX[2]=6000.0; rsuPosY[2]=1800.0;
        rsuPosX[3]=1000.0; rsuPosY[3]=4300.0;
        rsuPosX[4]=3500.0; rsuPosY[4]=4300.0;
        rsuPosX[5]=6000.0; rsuPosY[5]=4300.0;
        rsuPosX[6]=1000.0; rsuPosY[6]=6800.0;
        rsuPosX[7]=3500.0; rsuPosY[7]=6800.0;
        rsuPosX[8]=6000.0; rsuPosY[8]=6800.0;
    }
    else if (topoMode == "range_test")
    {
        // Single RSU at (2000, 3000) — PC5 range sweep test.
        // Vehicles are spread from x=100 to x=3900 (stationary, vehicleSpeed=0).
        // dist_to_nearest_rsu in epoch CSV directly gives range vs. PDR data.
        nGnbs = 1;  nRsus = 1;
        gnbPosX[0]=2000.0; gnbPosY[0]=2975.0;
        rsuPosX[0]=2000.0; rsuPosY[0]=3000.0;
    }
    else // highway
    {
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0]=1000.0; gnbPosY[0]=2975.0;
        gnbPosX[1]=2000.0; gnbPosY[1]=2975.0;
        gnbPosX[2]=3000.0; gnbPosY[2]=2975.0;
        gnbPosX[3]=4000.0; gnbPosY[3]=2975.0;
        rsuPosX[0]=1200.0; rsuPosY[0]=3000.0;
        rsuPosX[1]=2000.0; rsuPosY[1]=3000.0;
        rsuPosX[2]=2800.0; rsuPosY[2]=3000.0;
        rsuPosX[3]=3600.0; rsuPosY[3]=3000.0;
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // ── Nodes ─────────────────────────────────────────────────────────────────
    NodeContainer vehicleNodes; vehicleNodes.Create(nVehicles);
    NodeContainer gnbNodes;     gnbNodes.Create(nGnbs);
    NodeContainer rsuNodes;     rsuNodes.Create(nRsus);
    NodeContainer serverNC;     serverNC.Create(1);
    Ptr<Node>     serverNode = serverNC.Get(0);

    // SL UE container: vehicles first, then RSUs
    NodeContainer slUeNodes;
    for (uint32_t i = 0; i < nVehicles; ++i) slUeNodes.Add(vehicleNodes.Get(i));
    for (uint32_t r = 0; r < nRsus;     ++r) slUeNodes.Add(rsuNodes.Get(r));

    // ── Mobility ──────────────────────────────────────────────────────────────
    {   // gNBs: fixed at 25 m height
        MobilityHelper mob;
        mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        for (uint32_t j = 0; j < nGnbs; ++j)
            pos->Add(Vector(gnbPosX[j], gnbPosY[j], 25.0));
        mob.SetPositionAllocator(pos);
        mob.Install(gnbNodes);
    }
    {   // RSUs: fixed at 6 m height
        MobilityHelper mob;
        mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        for (uint32_t r = 0; r < nRsus; ++r)
            pos->Add(Vector(rsuPosX[r], rsuPosY[r], rsuPosZ));
        mob.SetPositionAllocator(pos);
        mob.Install(rsuNodes);
    }
    {   // Server: off-map static
        MobilityHelper mob;
        mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        pos->Add(Vector(0.0, -200.0, 0.0));
        mob.SetPositionAllocator(pos);
        mob.Install(serverNC);
    }
    // Vehicles
    if (!traceFile.empty())
    {
        MobilityHelper mob;
        mob.SetMobilityModel("ns3::WaypointMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        for (uint32_t i = 0; i < nVehicles; ++i)
            pos->Add(Vector(1000.0 + i * 80.0, 3000.0, 1.5));
        mob.SetPositionAllocator(pos);
        mob.Install(vehicleNodes);
        Ns2MobilityHelper ns2mob(traceFile);
        ns2mob.Install(vehicleNodes.Begin(), vehicleNodes.End());
        std::cout << "  Mobility    : SUMO trace " << traceFile << "\n";
    }
    else
    {
        MobilityHelper mob;
        mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        // range_test: spread vehicles 100m~3900m from the single RSU at x=2000
        // otherwise: spread vehicles evenly across 3000m road
        const double startX  = (topoMode == "range_test") ? 100.0  : 1000.0;
        const double spanX   = (topoMode == "range_test") ? 3800.0 : 3000.0;
        const double spacing = (nVehicles > 1) ? spanX / (nVehicles - 1) : 0.0;
        for (uint32_t i = 0; i < nVehicles; ++i)
            pos->Add(Vector(startX + i * spacing, 3000.0, 1.5));
        mob.SetPositionAllocator(pos);
        mob.Install(vehicleNodes);
        for (uint32_t i = 0; i < nVehicles; ++i)
            vehicleNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()
                ->SetVelocity(Vector(vehicleSpeed, 0.0, 0.0));
        std::cout << "  Mobility    : ConstantVelocity " << vehicleSpeed << " m/s\n";
    }

    // ── Buildings (UMa_Buildings / V2V_Urban channel condition) ──────────────
    // MobilityBuildingInfo must always be attached because the V2V_Urban SL
    // channel condition model queries it even when no buildings are loaded.
    BuildingsHelper::Install(vehicleNodes);
    BuildingsHelper::Install(gnbNodes);
    BuildingsHelper::Install(rsuNodes);
    BuildingsHelper::Install(serverNC);

    if (!buildingsFile.empty())
    {
        uint32_t nBuildings = LoadBuildingsFromCsv(buildingsFile, minBuildingAreaM2);
        std::cout << "  Buildings   : " << nBuildings << " loaded from " << buildingsFile;
        if (minBuildingAreaM2 > 0.0)
            std::cout << " (filtered: area >= " << minBuildingAreaM2 << " m²)";
        std::cout << "\n";
    }

    // ── Internet stack ────────────────────────────────────────────────────────
    InternetStackHelper internet;
    internet.Install(vehicleNodes);
    internet.Install(rsuNodes);
    internet.Install(serverNC);

    // ── NR helper + EPC ───────────────────────────────────────────────────────
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper>  bfHelper  = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(2)));

    // ── Band 1: NR Uu  (3.5 GHz, UMa_LoS) ───────────────────────────────────
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf uuConf(uuFreqHz, uuBandwidth, 1,
                                                  BandwidthPartInfo::UMa_Buildings);
    OperationBandInfo uuBand = ccBwpCreator.CreateOperationBandContiguousCc(uuConf);
    nrHelper->InitializeOperationBand(&uuBand);
    BandwidthPartInfoPtrVector uuBwps = CcBwpCreator::GetAllBwps({uuBand});

    Packet::EnableChecking();
    Packet::EnablePrinting();

    bfHelper->SetAttribute("BeamformingMethod",
                           TypeIdValue(DirectPathBeamforming::GetTypeId()));

    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    NetDeviceContainer gnbDevs       = nrHelper->InstallGnbDevice(gnbNodes, uuBwps);
    NetDeviceContainer vehicleNrDevs = nrHelper->InstallUeDevice(vehicleNodes, uuBwps);

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(gnbDevs,       stream);
    stream += nrHelper->AssignStreams(vehicleNrDevs, stream);

    for (uint32_t j = 0; j < nGnbs; ++j)
    {
        nrHelper->GetGnbPhy(gnbDevs.Get(j), 0)->SetAttribute("Numerology",
                                                              UintegerValue(uuNumerology));
        nrHelper->GetGnbPhy(gnbDevs.Get(j), 0)->SetAttribute("TxPower",
                                                              DoubleValue(gnbTxPowerDbm));
        DynamicCast<NrGnbNetDevice>(gnbDevs.Get(j))->UpdateConfig();
        // SRS periodicity: 320 supports up to ~160 UEs/gNB
        DynamicCast<NrGnbNetDevice>(gnbDevs.Get(j))
            ->GetRrc()->SetAttribute("SrsPeriodicity", UintegerValue(320));
    }
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        nrHelper->GetUePhy(vehicleNrDevs.Get(i), 0)->SetAttribute("TxPower",
                                                                   DoubleValue(uuTxPowerDbm));
        DynamicCast<NrUeNetDevice>(vehicleNrDevs.Get(i))->UpdateConfig();
    }

    // X2 for handover
    for (uint32_t j = 0; j < nGnbs; ++j)
        for (uint32_t k = j + 1; k < nGnbs; ++k)
            epcHelper->AddX2Interface(gnbNodes.Get(j), gnbNodes.Get(k));

    // ── Band 2: NR SL PC5  (5.89 GHz, V2V_Highway) ───────────────────────────
    const uint8_t bwpIdSl = 0;
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing",  BooleanValue(true));
    nrHelper->SetUeMacAttribute("T1",             UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2",             UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId",   UintegerValue(0));
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK", UintegerValue(bwpIdSl));

    CcBwpCreator::SimpleOperationBandConf slConf(slFreqHz, slBwRb * 100e3, 1,
                                                  BandwidthPartInfo::V2V_Urban);
    OperationBandInfo slBand = ccBwpCreator.CreateOperationBandContiguousCc(slConf);

    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(MilliSeconds(chCondUpdateMs)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    nrHelper->InitializeOperationBand(&slBand);
    BandwidthPartInfoPtrVector slBwps = CcBwpCreator::GetAllBwps({slBand});

    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(slTxPowerDbm));

    // slNetDevs[0..N-1] = vehicle SL devices
    // slNetDevs[N..N+nRsus-1] = RSU SL devices
    NetDeviceContainer slNetDevs = nrHelper->InstallUeDevice(slUeNodes, slBwps);
    stream += nrHelper->AssignStreams(slNetDevs, stream);
    for (uint32_t i = 0; i < nVehicles + nRsus; ++i)
        DynamicCast<NrUeNetDevice>(slNetDevs.Get(i))->UpdateConfig();

    // ── NrSlHelper ────────────────────────────────────────────────────────────
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);
    nrSlHelper->SetSlErrorModel("ns3::NrEesmIrT1");
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(slMcs));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdSl);
    nrSlHelper->PrepareUeForSidelink(slNetDevs, bwpIdContainer);

    // ── IP addresses ──────────────────────────────────────────────────────────
    Ipv4AddressHelper       ipv4h;
    Ipv4StaticRoutingHelper ipv4Routing;

    // Uu IPs for vehicles (7.0.0.x)
    std::vector<Ipv4InterfaceContainer> vehicleUuIps(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
        vehicleUuIps[i] = epcHelper->AssignUeIpv4Address(
            NetDeviceContainer(vehicleNrDevs.Get(i)));

    // SL IPs for vehicles
    std::vector<Ipv4InterfaceContainer> vehicleSlIps(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
        vehicleSlIps[i] = epcHelper->AssignUeIpv4Address(
            NetDeviceContainer(slNetDevs.Get(i)));

    // SL IPs for RSUs
    std::vector<Ipv4InterfaceContainer> rsuSlIps(nRsus);
    for (uint32_t r = 0; r < nRsus; ++r)
        rsuSlIps[r] = epcHelper->AssignUeIpv4Address(
            NetDeviceContainer(slNetDevs.Get(nVehicles + r)));

    // Vehicle default route → EPC gateway (for Uu)
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        Ptr<Ipv4StaticRouting> rt =
            ipv4Routing.GetStaticRouting(vehicleNodes.Get(i)->GetObject<Ipv4>());
        rt->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    nrHelper->AttachToClosestEnb(vehicleNrDevs, gnbDevs);

    // ── P2P 1: PGW ↔ server  (Uu backhaul) ───────────────────────────────────
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
    p2ph.SetDeviceAttribute("Mtu",      UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(10)));
    NetDeviceContainer pgwSrvDevs = p2ph.Install(pgw, serverNode);
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwSrvIfaces = ipv4h.Assign(pgwSrvDevs);
    const Ipv4Address serverUuIp = pgwSrvIfaces.GetAddress(1);  // 1.0.0.2

    // ── P2P 2: RSU[r] ↔ server  (PC5 backhaul, one link per RSU) ────────────
    // Subnet 10.10.(2+r).0/30:  RSU[r]=10.10.(2+r).1,  server=10.10.(2+r).2
    PointToPointHelper bhP2p;
    bhP2p.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    bhP2p.SetChannelAttribute("Delay",   StringValue("5ms"));

    std::vector<NetDeviceContainer>     backhaulDevs(nRsus);
    std::vector<Ipv4InterfaceContainer> backhaulIfaces(nRsus);
    std::vector<Ipv4Address>            rsuBackhaulIps(nRsus);

    for (uint32_t r = 0; r < nRsus; ++r)
    {
        backhaulDevs[r] = bhP2p.Install(rsuNodes.Get(r), serverNode);
        std::ostringstream subnet;
        subnet << "10.10." << (2 + r) << ".0";
        ipv4h.SetBase(subnet.str().c_str(), "255.255.255.252");
        backhaulIfaces[r] = ipv4h.Assign(backhaulDevs[r]);
        rsuBackhaulIps[r] = backhaulIfaces[r].GetAddress(0); // RSU side
    }
    // remotePC5Ip: server's address on RSU0's backhaul link (10.10.2.2)
    const Ipv4Address remotePC5Ip = backhaulIfaces[0].GetAddress(1);

    // ── Routing ───────────────────────────────────────────────────────────────
    Ptr<Ipv4StaticRouting> srvRoute =
        ipv4Routing.GetStaticRouting(serverNode->GetObject<Ipv4>());

    // Server: 7.0.0.0/8 → via PGW (for Uu packets)
    srvRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"),
                                pgwSrvIfaces.GetAddress(0), 1);

    // Each RSU:
    //   - Route to remotePC5Ip (10.10.2.2) via its own backhaul next-hop
    //   - Route 7.0.0.0/8 via SL interface (for any downlink to vehicles, if needed)
    for (uint32_t r = 0; r < nRsus; ++r)
    {
        Ptr<Ipv4StaticRouting> rsuRt =
            ipv4Routing.GetStaticRouting(rsuNodes.Get(r)->GetObject<Ipv4>());
        const uint32_t rsuBhIf =
            rsuNodes.Get(r)->GetObject<Ipv4>()
                           ->GetInterfaceForDevice(backhaulDevs[r].Get(0));
        rsuRt->AddHostRouteTo(remotePC5Ip, backhaulIfaces[r].GetAddress(1), rsuBhIf);

        const uint32_t rsuSlIf =
            rsuNodes.Get(r)->GetObject<Ipv4>()
                           ->GetInterfaceForDevice(slNetDevs.Get(nVehicles + r));
        rsuRt->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), rsuSlIf);
    }

    // Each vehicle: host route to remotePC5Ip via nearest RSU's SL IP
    // (SL TFT matches 10.10.0.0/16 → groupcast dstL2Id=255; all in-range RSUs receive)
    std::vector<uint32_t> vehNearestRsu(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        Vector vp = vehicleNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        uint32_t best = 0; double bestD = 1e18;
        for (uint32_t r = 0; r < nRsus; ++r)
        {
            double dx = rsuPosX[r] - vp.x;
            double dy = rsuPosY[r] - vp.y;
            double d  = std::sqrt(dx*dx + dy*dy);
            if (d < bestD) { bestD = d; best = r; }
        }
        vehNearestRsu[i] = best;

        const uint32_t vehSlIf =
            vehicleNodes.Get(i)->GetObject<Ipv4>()
                               ->GetInterfaceForDevice(slNetDevs.Get(i));
        Ptr<Ipv4StaticRouting> vehRt =
            ipv4Routing.GetStaticRouting(vehicleNodes.Get(i)->GetObject<Ipv4>());
        vehRt->AddHostRouteTo(remotePC5Ip, rsuSlIps[vehNearestRsu[i]].GetAddress(0), vehSlIf);
    }

    // ── NR SL resource pool ───────────────────────────────────────────────────
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100);
    ptrFactory->SetSlSelectionWindow(5);
    ptrFactory->SetSlFreqResourcePscch(10);
    ptrFactory->SetSlSubchannelSize(slSubchannelSize);
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> slReservePeriodList = {0, slReservePeriodMs};
    ptrFactory->SetSlResourceReservePeriodList(slReservePeriodList);
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();

    LteRrcSap::SlResourcePoolIdNr slPoolId; slPoolId.id = 0;
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
    psschParams.slMaxTxTransNumPssch = slHarq ? slMaxTxTrans : 1;
    LteRrcSap::SlPsschTxConfigList psschCfgList;
    psschCfgList.slPsschTxParameters[0] = psschParams;

    LteRrcSap::SlUeSelectedConfig slUeSelCfg;
    slUeSelCfg.slProbResourceKeep  = 0;
    slUeSelCfg.slPsschTxConfigList = psschCfgList;

    LteRrcSap::SidelinkPreconfigNr slPreCfg;
    slPreCfg.slPreconfigGeneral         = slPreconfigGeneral;
    slPreCfg.slUeSelectedPreConfig      = slUeSelCfg;
    slPreCfg.slPreconfigFreqInfoList[0] = slFreqCfg;

    nrSlHelper->InstallNrSlPreConfiguration(slNetDevs, slPreCfg);
    stream += nrSlHelper->AssignStreams(slNetDevs, stream);

    // ── NR SL bearer activation ───────────────────────────────────────────────
    const Time slBearerActivation = appStart - MilliSeconds(10);

    SidelinkInfo slInfo;
    slInfo.m_castType    = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id     = 255;
    slInfo.m_rri         = MilliSeconds(slReservePeriodMs);
    slInfo.m_pdb         = Seconds(0);
    slInfo.m_harqEnabled = slHarq;

    // Vehicles: TFT matches packets to 10.10.0.0/255.255.0.0 (any RSU backhaul IP)
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                              Ipv4Address("10.10.0.0"),
                                              Ipv4Mask("255.255.0.0"),
                                              slInfo);
        nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                       NetDeviceContainer(slNetDevs.Get(i)), tft);
    }
    // RSUs: TFT matches 7.0.0.0/8 (vehicle SL subnet, for groupcast reception + any downlink)
    for (uint32_t r = 0; r < nRsus; ++r)
    {
        Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                              Ipv4Address("7.0.0.0"),
                                              Ipv4Mask("255.0.0.0"),
                                              slInfo);
        nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                       NetDeviceContainer(slNetDevs.Get(nVehicles + r)), tft);
    }

    // ── Position-based Uu handover helpers ────────────────────────────────────
    std::vector<UuHandoverHelper> uuHoHelpers(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        auto& h   = uuHoHelpers[i];
        h.vehIdx  = i;
        h.ueNrDev = DynamicCast<NrUeNetDevice>(vehicleNrDevs.Get(i));
        for (uint32_t j = 0; j < nGnbs; ++j)
        {
            auto gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(j));
            h.gnbRrcs.push_back(gnbDev->GetRrc());
            h.gnbCellIds.push_back(gnbDev->GetCellIds().at(0));
            h.gnbPos.push_back(Vector(gnbPosX[j], gnbPosY[j], 25.0));
        }
        // Position-based HO re-enabled.  Race condition patched in
        // lte-enb-rrc.cc::DoPathSwitchRequestAcknowledge (HasUeManager guard).
        // 30 s cooldown prevents back-to-back HOs.
        Simulator::Schedule(Seconds(2.0), &UuHandoverHelper::TriggerIfNeeded, &uuHoHelpers[i]);
    }

    // ── Applications ──────────────────────────────────────────────────────────
    std::filesystem::create_directories(outputDir);

    // ServerSinkApp (receives both Uu video/meta AND PC5 coop via RSU backhaul)
    Ptr<ServerSinkApp> sink = CreateObject<ServerSinkApp>();
    sink->SetAttribute("VideoPort",     UintegerValue(5000));
    sink->SetAttribute("MetaPort",      UintegerValue(5001));
    sink->SetAttribute("CoopPort",      UintegerValue(5002));
    sink->SetAttribute("OutputDir",     StringValue(outputDir));
    sink->SetAttribute("FrameDeadline", DoubleValue(frameDeadlineSec));
    serverNode->AddApplication(sink);
    sink->SetStartTime(appStart);
    sink->SetStopTime(simTime);

    // Pass infrastructure positions so the server can compute dist_to_serving_gnb
    // and dist_to_nearest_rsu in every epoch row.
    {
        std::vector<std::pair<double, double>> gnbXY, rsuXY;
        for (uint32_t j = 0; j < nGnbs; ++j)
            gnbXY.emplace_back(gnbPosX[j], gnbPosY[j]);
        for (uint32_t r = 0; r < nRsus; ++r)
            rsuXY.emplace_back(rsuPosX[r], rsuPosY[r]);
        sink->SetInfrastructurePositions(gnbXY, rsuXY);
    }

    // Wire handover notifications: each UuHandoverHelper calls sink->NotifyHandover()
    // when a position-based Uu HO fires, so recent_handover_flag and
    // time_since_last_handover_ms are accurate in the epoch CSV.
    for (auto& h : uuHoHelpers)
        h.sinkApp = sink;

    // Remote-support vehicles: video+meta via Uu  +  awareness/hazard via PC5
    for (uint32_t i = 0; i < nRemote; ++i)
    {
        // ① EdgeVideoMetadataApp — dual-path (Uu + PC5), runtime-steerable
        Ptr<EdgeVideoMetadataApp> vapp = CreateObject<EdgeVideoMetadataApp>();
        vapp->SetAttribute("ServerAddressUu",    Ipv4AddressValue(serverUuIp));
        vapp->SetAttribute("VideoPortUu",        UintegerValue(5000));
        vapp->SetAttribute("MetaPortUu",         UintegerValue(5001));
        vapp->SetAttribute("ServerAddressPc5",   Ipv4AddressValue(remotePC5Ip));
        vapp->SetAttribute("VideoPortPc5",       UintegerValue(5000));
        vapp->SetAttribute("MetaPortPc5",        UintegerValue(5001));
        vapp->SetAttribute("PathWeightUu",       DoubleValue(pathWeightUu));
        vapp->SetAttribute("VehicleId",          UintegerValue(i));
        vapp->SetAttribute("FPS",                UintegerValue(fps));
        vapp->SetAttribute("AvgFrameSize",       UintegerValue(avgFrameKB * 1024));
        vapp->SetAttribute("KeyframeInterval",   UintegerValue(keyframeInterval));
        vapp->SetAttribute("KeyframeMultiplier", DoubleValue(keyframeMultiplier));
        vapp->SetAttribute("MetaPayloadSize",    UintegerValue(metaPayloadBytes));
        vapp->SetAttribute("ChunkSize",          UintegerValue(chunkSize));
        vehicleNodes.Get(i)->AddApplication(vapp);
        vapp->SetStartTime(appStart);
        vapp->SetStopTime(simTime);

        // ② CooperativeVehicleApp → PC5 (SL groupcast) → RSU → server:5002
        Ptr<CooperativeVehicleApp> capp = CreateObject<CooperativeVehicleApp>();
        capp->SetAttribute("ServerAddress",     Ipv4AddressValue(remotePC5Ip));
        capp->SetAttribute("ServerPort",        UintegerValue(5002));
        capp->SetAttribute("VehicleId",         UintegerValue(i));
        capp->SetAttribute("AwarenessInterval", TimeValue(MilliSeconds(100)));
        capp->SetAttribute("HazardProbPerSec",  DoubleValue(hazardProb));
        vehicleNodes.Get(i)->AddApplication(capp);
        capp->SetStartTime(appStart);
        capp->SetStopTime(simTime);
    }

    // Normal vehicles: awareness/hazard via PC5 only
    for (uint32_t i = nRemote; i < nVehicles; ++i)
    {
        Ptr<CooperativeVehicleApp> capp = CreateObject<CooperativeVehicleApp>();
        capp->SetAttribute("ServerAddress",     Ipv4AddressValue(remotePC5Ip));
        capp->SetAttribute("ServerPort",        UintegerValue(5002));
        capp->SetAttribute("VehicleId",         UintegerValue(i));
        capp->SetAttribute("AwarenessInterval", TimeValue(MilliSeconds(100)));
        capp->SetAttribute("HazardProbPerSec",  DoubleValue(hazardProb));
        vehicleNodes.Get(i)->AddApplication(capp);
        capp->SetStartTime(appStart);
        capp->SetStopTime(simTime);
    }

    std::cout << "\n=== NR V2X Remote-Support  (Uu + PC5) ===\n"
              << "  topoMode    : " << topoMode << "\n"
              << "  Vehicles    : " << nVehicles
              << "  (remote=" << nRemote << " normal=" << nNormal << ")\n"
              << "  gNBs / RSUs : " << nGnbs << " / " << nRsus << "\n"
              << "  simTime     : " << simTime.GetSeconds() << " s\n"
              << "  Server Uu   : " << serverUuIp   << " (video/meta :5000/:5001)\n"
              << "  Server PC5  : " << remotePC5Ip  << " (coop :5002 via RSU backhaul)\n"
              << "  OutputDir   : " << outputDir << "/vehicles/\n\n";

    // ── Run ───────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "  Done.\n";
    return 0;
}
