// Copyright (c) 2026
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file cttc-nr-v2x-highway-mpquic.cc
 * @brief NR Uu + NR SL PC5 + MPQUIC multi-path V2X simulation framework.
 *
 * Topology (all nodes on highway y=3000):
 *
 *   ══════════════════════════════════════════════════ Road (y=3000)
 *   gNB0(1000)  gNB1(2000)  gNB2(3000)  gNB3(4000)    [1000m spacing, z=25m]
 *   RSU0(1200)  RSU1(2000)  RSU2(2800)  RSU3(3600)    [800m  spacing, z=6m]
 *
 *   Vehicle[i] starts at x = 1000 + i*spacing, speed = 22.22 m/s (+x)
 *   Max vehicle–gNB distance: 500 m  |  Max vehicle–RSU distance: 400 m
 *
 * Paths per vehicle:
 *   Uu : vehicle_uu_ip → gNB → EPC → PGW(1.0.0.1) → remoteHost(1.0.0.2):4000
 *   PC5: vehicle_sl_ip → NR SL (5.89 GHz) → RSU → P2P → remoteHost(10.10.2.2):4000
 *
 * Key degradation events:
 *   Uu:  RSRP degrades as vehicle moves away from serving gNB (no handover)
 *        Handover at gNB boundary (via A3-RSRP algorithm + X2)
 *   PC5: d_RSU > 300 m → PC5 range exit (enters/exits ~x=700-1300 m)
 *
 * Per-vehicle CSV log (100 ms interval):
 *   time_s, veh_id, veh_x, veh_y,
 *   uu_rsrp_dBm, uu_sinr_dB, uu_rtt_ms, uu_cwnd, uu_tx_pkts, uu_retx,
 *   ho_flag, serving_gnb,
 *   pc5_rtt_ms, pc5_cwnd, pc5_tx_pkts, pc5_retx, pc5_pending,
 *   nearest_rsu, dist_rsu_m, angle_rsu_deg,
 *   rx_pkts, delivery_pct, gamma, queue_depth
 *
 * Usage:
 *   ./ns3 run cttc-nr-v2x-highway-mpquic
 *   ./ns3 run "cttc-nr-v2x-highway-mpquic --nVehicles=20 --simTime=120 --schedulerType=Redundant"
 */

#include "nr-sl-cam-bsm-app.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/channel-condition-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/buildings-module.h"
#include "ns3/mpquic-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"

#include <algorithm>
#include <cmath>
#include <execinfo.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <signal.h>
#include <sstream>
#include <unistd.h>
#include <unordered_set>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CttcNrV2xHighwayMpquic");

// ---------------------------------------------------------------------------
// Per-vehicle Uu PHY telemetry (updated by trace callbacks)
// ---------------------------------------------------------------------------
struct VehicleUuState
{
    double  rsrpDbm{-140.0};
    double  sinrDb{-10.0};
    bool    hoFlag{false};    // set true for one snapshot interval after handover
    int32_t servingGnb{0};   // index of currently serving gNB
};

static std::vector<VehicleUuState>    g_ueState;
static std::vector<Ptr<MobilityModel>> g_allVehicleMobs; ///< all vehicle mobilities for nearby_vehicle_count

// HandoverStart: (imsi, cellId, rnti, targetCellId)
void CbHandoverStart(uint32_t vehIdx, uint64_t /*imsi*/, uint16_t /*cellId*/,
                     uint16_t /*rnti*/, uint16_t targetCellId)
{
    g_ueState[vehIdx].hoFlag = true;
    NS_LOG_INFO("Vehicle " << vehIdx << " handover started → cellId=" << targetCellId);
}

// HandoverEndOk: (imsi, cellId, rnti)
// cellId is the CC-level cell ID (2, 4, 6, 8 for 4 gNBs in NR).
// gNB index = (cellId / 2) - 1  because NrHelper allocates 2 IDs per gNB:
//   gNB[0] → cellIds {1, 2}  → CC ID = 2 → idx = (2/2)-1 = 0
//   gNB[1] → cellIds {3, 4}  → CC ID = 4 → idx = (4/2)-1 = 1
//   gNB[2] → cellIds {5, 6}  → CC ID = 6 → idx = (6/2)-1 = 2
//   gNB[3] → cellIds {7, 8}  → CC ID = 8 → idx = (8/2)-1 = 3
void CbHandoverEndOk(uint32_t vehIdx, uint64_t /*imsi*/, uint16_t cellId, uint16_t /*rnti*/)
{
    g_ueState[vehIdx].hoFlag    = true;  // keep flag until next snapshot
    g_ueState[vehIdx].servingGnb = static_cast<int32_t>(cellId / 2) - 1;
    NS_LOG_INFO("Vehicle " << vehIdx << " handover completed → cellId=" << cellId
                << " → gNB[" << g_ueState[vehIdx].servingGnb << "]");
}

// ReportRsrp trace: TracedCallback<uint16_t, uint16_t, uint16_t, double, uint8_t>
// fires with (GetCellId(), m_imsi[truncated], m_rnti, avg_rsrp, GetBwpId())
void CbRsrp(uint32_t vehIdx, uint16_t /*cellId*/, uint16_t /*imsi16*/,
             uint16_t /*rnti*/, double rsrp, uint8_t /*bwpId*/)
{
    if (vehIdx < g_ueState.size())
        g_ueState[vehIdx].rsrpDbm = rsrp;
}

// DlDataSinr trace: TracedCallback<uint16_t, uint16_t, double, uint16_t>
// fires with (GetCellId(), m_rnti, sinr_linear, GetBwpId())
void CbSinr(uint32_t vehIdx, uint16_t /*cellId*/, uint16_t /*rnti*/,
             double sinr, uint16_t /*bwpId*/)
{
    if (vehIdx < g_ueState.size())
        g_ueState[vehIdx].sinrDb = (sinr > 0.0) ? 10.0 * std::log10(sinr) : -100.0;
}

// ---------------------------------------------------------------------------
// Per-vehicle snapshot → CSV
// ---------------------------------------------------------------------------
// Epoch-level CSV logger
// ---------------------------------------------------------------------------
struct EpochLogger
{
    uint32_t              vehIdx;   // actual ns-3 node ID (for CSV output)
    uint32_t              connId{0}; // sequential MPQUIC connId (for GetConnRxPackets)
    std::ofstream         csv;
    Ptr<MpQuicSocketBase> mpquic;
    Ptr<MpQuicReceiverApp> rxApp;
    Ptr<MpQuicSenderApp>  senderApp;

    // snapshot at epoch start
    uint64_t prevUuTx{0};
    uint64_t prevPc5Tx{0};
    uint64_t prevRx{0};
    uint32_t epochIdx{0};
    // Per-path accurate delivery snapshots (cumulative, read as delta each epoch)
    uint64_t prevUuRxFrames{0};
    uint64_t prevPc5RxFrames{0};
    uint64_t prevUuOnTime{0};
    uint64_t prevPc5OnTime{0};
    uint64_t prevE2eOnTime{0};
    uint64_t prevTxUniqFrames{0};  ///< snapshot of mpquic->GetStreamSeqNo(0)

    // CAM/BSM background traffic reference (null = no CAM app on this vehicle)
    Ptr<NrSlCamBsmApp> camBsmApp{nullptr};
    uint64_t prevCamTx{0};
    uint64_t prevBsmTx{0};

    // Per-path retransmission snapshots (for missing_frame_burst_len proxy)
    uint64_t prevRetxUu{0};
    uint64_t prevRetxPc5{0};

    // Mobility / geometry features
    Ptr<MobilityModel> vehMob{nullptr};
    uint32_t           nRsuPos{0};
    double             rsuPX[9]{};
    double             rsuPY[9]{};

    // gNB positions (for dist_serving_gnb_m + handover_zone_flag)
    uint32_t           nGnbPos{0};
    double             gnbPX[9]{};
    double             gnbPY[9]{};
};

void TakeEpochSnap(EpochLogger* el, Time epochLen, Time simEnd)
{
    if (Simulator::Now() > simEnd)
        return;

    ++el->epochIdx;

    // Current cumulative counters
    uint64_t uuTxCur = 0, pc5TxCur = 0;
    double   uuRtt   = -1.0, pc5Rtt = -1.0;
    double   uuRttVar = -1.0, pc5RttVar = -1.0;
    uint64_t uuRetxCur = 0, pc5RetxCur = 0;
    for (const auto& p : el->mpquic->GetPaths())
    {
        if (p->GetPathType() == MpQuicPathType::UU)
        {
            uuTxCur = p->GetTxPackets();
            if (p->IsRttInitialized())
            {
                uuRtt    = p->GetRtt().GetMilliSeconds();
                uuRttVar = p->GetRttVar().GetMilliSeconds();
            }
            uuRetxCur = p->GetRetxPackets();
        }
        else
        {
            pc5TxCur = p->GetTxPackets();
            if (p->IsRttInitialized())
            {
                pc5Rtt    = p->GetRtt().GetMilliSeconds();
                pc5RttVar = p->GetRttVar().GetMilliSeconds();
            }
            pc5RetxCur = p->GetRetxPackets();
        }
    }
    uint64_t rxCur       = el->rxApp->GetConnRxPackets(el->connId);
    uint64_t txUniqCur   = static_cast<uint64_t>(el->mpquic->GetStreamSeqNo(0));

    // Per-path accurate delivery counters (cumulative → epoch delta)
    uint64_t uuRxFramesCur  = el->rxApp->GetConnRxUuFrames(el->connId);
    uint64_t pc5RxFramesCur = el->rxApp->GetConnRxPc5Frames(el->connId);
    uint64_t uuOnTimeCur    = el->rxApp->GetConnRxUuOnTime(el->connId);
    uint64_t pc5OnTimeCur   = el->rxApp->GetConnRxPc5OnTime(el->connId);
    uint64_t e2eOnTimeCur   = el->rxApp->GetConnRxE2eOnTime(el->connId);

    // Epoch deltas — TX and RX
    uint64_t uuTxD  = (uuTxCur  >= el->prevUuTx)  ? uuTxCur  - el->prevUuTx  : 0;
    uint64_t pc5TxD = (pc5TxCur >= el->prevPc5Tx) ? pc5TxCur - el->prevPc5Tx : 0;

    // Per-path delivery deltas (accurate, dedup by streamSeqNo per path)
    uint64_t uuRxD      = (uuRxFramesCur  >= el->prevUuRxFrames)  ? uuRxFramesCur  - el->prevUuRxFrames  : 0;
    uint64_t pc5RxD     = (pc5RxFramesCur >= el->prevPc5RxFrames) ? pc5RxFramesCur - el->prevPc5RxFrames : 0;
    uint64_t uuOnTimeD  = (uuOnTimeCur    >= el->prevUuOnTime)    ? uuOnTimeCur    - el->prevUuOnTime    : 0;
    uint64_t pc5OnTimeD = (pc5OnTimeCur   >= el->prevPc5OnTime)   ? pc5OnTimeCur   - el->prevPc5OnTime   : 0;
    uint64_t e2eOnTimeD = (e2eOnTimeCur   >= el->prevE2eOnTime)   ? e2eOnTimeCur   - el->prevE2eOnTime   : 0;

    // Delay proxy: RTT/2 (one-way delay estimate)
    double uuDelay  = (uuRtt  >= 0) ? uuRtt  / 2.0 : -1.0;
    double pc5Delay = (pc5Rtt >= 0) ? pc5Rtt / 2.0 : -1.0;

    // p95 OWD estimate: (smoothed_rtt + 2*rttvar) / 2  (RFC 9002 tail latency formula)
    double uuP95  = (uuRtt >= 0 && uuRttVar >= 0) ? (uuRtt + 2.0*uuRttVar) / 2.0 : -1.0;
    double pc5P95 = (pc5Rtt >= 0 && pc5RttVar >= 0) ? (pc5Rtt + 2.0*pc5RttVar) / 2.0 : -1.0;

    // Jitter: OWD variance proxy = rttVar / 2
    double uuJitter  = (uuRttVar  >= 0) ? uuRttVar  / 2.0 : -1.0;
    double pc5Jitter = (pc5RttVar >= 0) ? pc5RttVar / 2.0 : -1.0;

    // missing_frame_burst_len: retransmissions this epoch (combined paths, proxy for consecutive losses)
    uint64_t uuRetxD  = (uuRetxCur  >= el->prevRetxUu)  ? uuRetxCur  - el->prevRetxUu  : 0;
    uint64_t pc5RetxD = (pc5RetxCur >= el->prevRetxPc5) ? pc5RetxCur - el->prevRetxPc5 : 0;
    uint64_t burstLen = uuRetxD + pc5RetxD;

    // Unique frames sent per path (exclude retransmissions; retx shares same streamSeqNo)
    uint64_t uuUniqSent  = (uuTxD  >= uuRetxD)  ? uuTxD  - uuRetxD  : 0;
    uint64_t pc5UniqSent = (pc5TxD >= pc5RetxD) ? pc5TxD - pc5RetxD : 0;

    // E2E denominator: unique application frames generated this epoch.
    // m_streamSeq[0] is incremented ONCE per Send() call before path dispatch,
    // so txUniqD is scheduler-independent and avoids the Redundant double-count problem.
    uint64_t txUniqD     = (txUniqCur >= el->prevTxUniqFrames) ? txUniqCur - el->prevTxUniqFrames : 0;
    uint64_t totalUniqSent = (txUniqD > 0) ? txUniqD : 1;  // avoid div-by-zero at warmup

    // e2e deadline met fraction: on-time at application-layer drain / unique frames sent
    double e2eFrac = static_cast<double>(e2eOnTimeD) / static_cast<double>(totalUniqSent);

    // Freeze: e2e delivery < 70% in epoch
    int freeze = (e2eFrac >= 0.0 && e2eFrac < 0.70) ? 1 : 0;

    // Mobility / geometry features
    double vehX = -1.0, vehY = -1.0, speedMps = -1.0, headingDeg = -1.0;
    double distNearestRsu = -1.0;
    if (el->vehMob)
    {
        Vector pos = el->vehMob->GetPosition();
        Vector vel = el->vehMob->GetVelocity();
        vehX     = pos.x;
        vehY     = pos.y;
        speedMps = std::sqrt(vel.x*vel.x + vel.y*vel.y);
        if (speedMps > 0.01)
            headingDeg = std::atan2(vel.y, vel.x) * 180.0 / M_PI;
        double minDist = 1e9;
        for (uint32_t r = 0; r < el->nRsuPos; ++r)
        {
            double dx = el->rsuPX[r] - pos.x;
            double dy = el->rsuPY[r] - pos.y;
            double d  = std::sqrt(dx*dx + dy*dy);
            if (d < minDist) minDist = d;
        }
        if (el->nRsuPos > 0) distNearestRsu = minDist;
    }

    // gNB distance features
    double distServingGnb  = -1.0;
    int    handoverZoneFlag = 0;
    if (el->vehMob && el->nGnbPos > 0)
    {
        Vector pos = el->vehMob->GetPosition();
        int32_t sGnb = (el->vehIdx < g_ueState.size())
                       ? g_ueState[el->vehIdx].servingGnb : 0;
        if (sGnb >= 0 && static_cast<uint32_t>(sGnb) < el->nGnbPos)
        {
            double dx = el->gnbPX[sGnb] - pos.x;
            double dy = el->gnbPY[sGnb] - pos.y;
            distServingGnb = std::sqrt(dx*dx + dy*dy);
        }
        // handover zone: second-nearest gNB within 400 m of vehicle
        double minD1 = 1e9, minD2 = 1e9;
        for (uint32_t g = 0; g < el->nGnbPos; ++g)
        {
            double dx = el->gnbPX[g] - pos.x;
            double dy = el->gnbPY[g] - pos.y;
            double d  = std::sqrt(dx*dx + dy*dy);
            if (d < minD1) { minD2 = minD1; minD1 = d; }
            else if (d < minD2) { minD2 = d; }
        }
        handoverZoneFlag = (minD2 < 400.0) ? 1 : 0;
    }

    // Nearby vehicle count (within 300 m)
    const double kNearbyRadiusM = 300.0;
    uint32_t nearbyVehCnt = 0;
    if (el->vehMob)
    {
        Vector pos = el->vehMob->GetPosition();
        for (const auto& mob : g_allVehicleMobs)
        {
            if (mob == el->vehMob) continue;   // skip self
            Vector op = mob->GetPosition();
            double dx = op.x - pos.x, dy = op.y - pos.y;
            if (std::sqrt(dx*dx + dy*dy) <= kNearbyRadiusM)
                ++nearbyVehCnt;
        }
    }

    // GPS timestamp model: capture = epoch_start, server_rx = epoch_end (1-epoch delay)
    double epochEndMs   = Simulator::Now().GetMilliSeconds();
    double epochStartMs = epochEndMs - epochLen.GetMilliSeconds();
    double gpsCaptureTs = epochStartMs;
    double gpsRxTs      = epochEndMs;

    el->csv
        << el->epochIdx           << ","
        << el->vehIdx             << ","
        << Simulator::Now().GetSeconds() << ","
        << uuUniqSent             << ","    // uu_completed_frames (unique frames sent via Uu, excl. retx)
        << 0                      << ","    // uu_timeout_frames
        << uuOnTimeD              << ","    // uu_deadline_met_frames (accurate: per-path counter)
        << pc5UniqSent            << ","    // pc5_video_completed_frames (unique sent via PC5, excl. retx)
        << 0                      << ","    // pc5_video_timeout_frames
        << pc5OnTimeD             << ","    // pc5_video_deadline_met_frames (accurate: per-path counter)
        << std::fixed << std::setprecision(3)
        << uuDelay                << ","    // uu_avg_delay_ms
        << pc5Delay               << ","    // pc5_video_avg_delay_ms
        << e2eFrac                << ","    // e2e_deadline_met_frac (e2eOnTimeD / totalUniqSent)
        << freeze                 << ","    // freeze_flag_e2e
        << uuRtt                  << ","    // uu_rtt_ms (raw)
        << pc5Rtt                 << ",";   // pc5_rtt_ms (raw)

    // CAM/BSM epoch deltas (0 if no CAM app installed)
    uint64_t camTxCur = el->camBsmApp ? el->camBsmApp->GetCamTxCount() : 0;
    uint64_t bsmTxCur = el->camBsmApp ? el->camBsmApp->GetBsmTxCount() : 0;
    uint64_t camTxD   = (camTxCur >= el->prevCamTx) ? camTxCur - el->prevCamTx : 0;
    uint64_t bsmTxD   = (bsmTxCur >= el->prevBsmTx) ? bsmTxCur - el->prevBsmTx : 0;
    el->csv << camTxD << "," << bsmTxD << ","
            << uuP95 << "," << pc5P95 << ","
            << uuJitter << "," << pc5Jitter << ","
            << burstLen << ","
            << vehX << "," << vehY << ","
            << speedMps << "," << headingDeg << ","
            << distNearestRsu << ","
            << distServingGnb << "," << handoverZoneFlag << ","
            << nearbyVehCnt  << ","
            << gpsCaptureTs  << "," << gpsRxTs << ","
            << uuRxD         << "," << pc5RxD  << ","
            << txUniqD       << "\n";
    el->csv.flush();

    el->prevRetxUu  = uuRetxCur;
    el->prevRetxPc5 = pc5RetxCur;
    el->prevUuTx  = uuTxCur;
    el->prevPc5Tx = pc5TxCur;
    el->prevRx    = rxCur;
    el->prevCamTx = camTxCur;
    el->prevBsmTx = bsmTxCur;
    el->prevUuRxFrames  = uuRxFramesCur;
    el->prevPc5RxFrames = pc5RxFramesCur;
    el->prevUuOnTime    = uuOnTimeCur;
    el->prevPc5OnTime   = pc5OnTimeCur;
    el->prevE2eOnTime   = e2eOnTimeCur;
    el->prevTxUniqFrames = txUniqCur;

    Simulator::Schedule(epochLen, &TakeEpochSnap, el, epochLen, simEnd);
}

// ---------------------------------------------------------------------------
struct VehicleLogger
{
    uint32_t                          vehIdx;
    uint32_t                          connId{0}; // sequential MPQUIC connId (matches SetConnId(i))
    std::ofstream                     csv;
    Ptr<MpQuicSocketBase>             mpquic;
    Ptr<MpQuicReceiverApp>            rxApp;
    Ptr<MpQuicSenderApp>              senderApp;  ///< for app-generated TX count (excludes retx)
    Ptr<MobilityModel>                vehMob;
    std::vector<Ptr<MobilityModel>>   rsuMobs;  // all RSU mobility models
    uint64_t                          prevTx{0};
};

void TakeSnapshot(VehicleLogger* vl, Time interval, Time simEnd)
{
    if (Simulator::Now() > simEnd)
        return;

    const VehicleUuState& us = g_ueState[vl->vehIdx];

    // Position + nearest RSU geometry
    Vector vp        = vl->vehMob->GetPosition();
    double dist      = 1e9;
    double ang       = 0.0;
    int32_t nearRsu  = -1;
    for (std::size_t r = 0; r < vl->rsuMobs.size(); ++r)
    {
        Vector rp = vl->rsuMobs[r]->GetPosition();
        double dx = rp.x - vp.x;
        double dy = rp.y - vp.y;
        double dz = rp.z - vp.z;
        double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d < dist)
        {
            dist    = d;
            ang     = std::atan2(dy, dx) * 180.0 / M_PI;
            nearRsu = static_cast<int32_t>(r);
        }
    }

    // MPQUIC path stats
    const auto& paths = vl->mpquic->GetPaths();
    double uuRtt  = 0, pc5Rtt  = 0;
    uint64_t uuTx = 0, pc5Tx  = 0;
    uint64_t uuRetx = 0, pc5Retx = 0;
    uint32_t uuCwnd  = 0, pc5Cwnd  = 0;
    uint32_t pc5Pend = 0;
    for (const auto& p : paths)
    {
        if (p->GetPathType() == MpQuicPathType::UU)
        {
            uuRtt  = p->GetRtt().GetMilliSeconds();
            uuTx   = p->GetTxPackets();
            uuRetx = p->GetRetxPackets();
            uuCwnd = p->GetCwndPackets();
        }
        else
        {
            pc5Rtt  = p->GetRtt().GetMilliSeconds();
            pc5Tx   = p->GetTxPackets();
            pc5Retx = p->GetRetxPackets();
            pc5Cwnd = p->GetCwndPackets();
            pc5Pend = p->GetPendingCount();
        }
    }

    // delivery_pct denominator = app-generated unique packets (excludes retransmissions)
    uint64_t appTx     = vl->senderApp->GetTotalPacketsSent();
    uint64_t totalRx   = vl->rxApp->GetConnRxPackets(vl->connId);
    double   delivPct  = (appTx > 0)
                         ? 100.0 * static_cast<double>(totalRx) / static_cast<double>(appTx)
                         : 0.0;
    double   gamma     = vl->rxApp->GetContinuity(vl->connId);
    uint32_t queueDep  = vl->mpquic->GetTotalQueueDepth();

    vl->csv
        << Simulator::Now().GetSeconds()          << ","
        << vl->vehIdx                             << ","
        << vp.x                                   << ","
        << vp.y                                   << ","
        // Uu
        << us.rsrpDbm                             << ","
        << us.sinrDb                              << ","
        << uuRtt                                  << ","
        << uuCwnd                                 << ","
        << uuTx                                   << ","
        << uuRetx                                 << ","
        << (us.hoFlag ? 1 : 0)                    << ","
        << us.servingGnb                          << ","
        // PC5
        << pc5Rtt                                 << ","
        << pc5Cwnd                                << ","
        << pc5Tx                                  << ","
        << pc5Retx                                << ","
        << pc5Pend                                << ","
        << nearRsu                                << ","
        << dist                                   << ","
        << ang                                    << ","
        // service
        << totalRx                                << ","
        << std::fixed << std::setprecision(2)
        << delivPct                               << ","
        << gamma                                  << ","
        << queueDep                               << "\n";

    // Clear one-shot handover flag after snapshot
    g_ueState[vl->vehIdx].hoFlag = false;

    Simulator::Schedule(interval, &TakeSnapshot, vl, interval, simEnd);
}

// ---------------------------------------------------------------------------
// Dynamic PC5 routing: track nearest RSU per vehicle and update routes.
//
// ROOT CAUSE of RoundRobin PC5 loss:
//   Data path (vehicle → SL → nearest RSU → backhaul → remoteHost) is already
//   resilient: the SL is groupcast (dstL2Id=255), so ALL RSUs in SL range receive
//   each data packet and IP-forward it to 10.10.2.2 via their own backhault.
//   ACK path (remoteHost → RSU → SL → vehicle) is STATIC: remoteHost has a
//   host route for vehicleSlIp via the RSU nearest at t=0.  As the vehicle moves
//   away, that RSU's SL link degrades and ACKs are never received → PTO fires →
//   retransmit → kMaxRetries=3 → permanent drop.
//
// FIX: check the nearest RSU every second and swap the host route on remoteHost
//   (and on the vehicle) to use the currently-nearest RSU.
// ---------------------------------------------------------------------------
struct Pc5RouteUpdater
{
    uint32_t                    vehIdx;
    Ptr<Node>                   vehNode;
    Ptr<NetDevice>              vehSlDev;
    std::vector<Vector>         rsuPos;
    std::vector<Ipv4Address>    rsuSlIps;        ///< RSU SL interface IPs (vehicle nexthop)
    std::vector<Ipv4Address>    rsuPC5Ips;       ///< RSU backhaul IPs (MpQuicPath remote)
    Ptr<MpQuicPath>             pc5Path;         ///< PC5 MpQuicPath for SetRemote() calls
    uint16_t                    rxPort{0};
    Ipv4Address                 currentRemotePC5Ip; ///< PC5 remote IP in use (for route removal)
    uint32_t                    currentRsu{0};

    static void Update(Pc5RouteUpdater* ru)
    {
        // Find nearest RSU by 2D distance
        Vector   vp      = ru->vehNode->GetObject<MobilityModel>()->GetPosition();
        uint32_t nearest = 0;
        double   bestD   = 1e18;
        for (uint32_t r = 0; r < ru->rsuPos.size(); ++r)
        {
            double dx = ru->rsuPos[r].x - vp.x;
            double dy = ru->rsuPos[r].y - vp.y;
            double d  = std::sqrt(dx * dx + dy * dy);
            if (d < bestD) { bestD = d; nearest = r; }
        }

        std::cerr << "[Pc5RT t=" << Simulator::Now().GetMilliSeconds() << "ms]"
                  << " veh[" << ru->vehIdx << "] pos=(" << vp.x << "," << vp.y << ")"
                  << " nearest=RSU[" << nearest << "] dist=" << bestD
                  << " currentRsu=" << ru->currentRsu << "\n";

        if (nearest != ru->currentRsu)
        {
            Ipv4StaticRoutingHelper h;
            auto vehRt = h.GetStaticRouting(ru->vehNode->GetObject<Ipv4>());

            // Remove old host route to currentRemotePC5Ip (the previous RSU's backhaul IP)
            for (uint32_t k = 0; k < vehRt->GetNRoutes(); ++k)
            {
                if (vehRt->GetRoute(k).GetDest() == ru->currentRemotePC5Ip &&
                    vehRt->GetRoute(k).IsHost())
                {
                    vehRt->RemoveRoute(k);
                    break;
                }
            }

            // Add new host route to nearest RSU's backhaul IP via that RSU's SL IP
            const uint32_t vehSlIf =
                ru->vehNode->GetObject<Ipv4>()->GetInterfaceForDevice(ru->vehSlDev);
            vehRt->AddHostRouteTo(ru->rsuPC5Ips[nearest], ru->rsuSlIps[nearest], vehSlIf);

            // Update MpQuicPath remote endpoint to nearest RSU's backhaul IP
            if (ru->pc5Path)
                ru->pc5Path->SetRemote(InetSocketAddress(ru->rsuPC5Ips[nearest], ru->rxPort));

            std::cerr << "  -> RSU[" << ru->currentRsu << "]->RSU[" << nearest
                      << "] nexthop=" << ru->rsuSlIps[nearest]
                      << " pc5Remote=" << ru->rsuPC5Ips[nearest] << "\n";
            NS_LOG_INFO("Veh[" << ru->vehIdx << "] PC5 RSU["
                        << ru->currentRsu << "]->RSU[" << nearest
                        << "]  dist=" << std::fixed << std::setprecision(0)
                        << bestD << "m  t=" << Simulator::Now().GetSeconds() << "s");

            ru->currentRemotePC5Ip = ru->rsuPC5Ips[nearest];
            ru->currentRsu = nearest;
        }

        Simulator::Schedule(MilliSeconds(20), &Pc5RouteUpdater::Update, ru);
    }
};

// ---------------------------------------------------------------------------
// Position-based Uu handover helper.
//
// ROOT CAUSE of Uu handover failure in ns-3-nr:
//   NrUePhy::ReportUeMeasurements() has the line
//   `m_ueCphySapUser->ReportUeMeasurements(ret)` COMMENTED OUT.
//   Consequently LteUeRrc::DoReportUeMeasurements is NEVER called, so the
//   A3 event is NEVER evaluated, and A3RsrpHandoverAlgorithm NEVER fires.
//
// FIX: bypass the broken measurement chain entirely.  Every kCheckIntervalMs
//   compare the vehicle's Euclidean distance to each gNB; if the nearest gNB
//   differs from the current serving cell, call SendHandoverRequest() directly
//   on the source gNB's LteEnbRrc.  A kCooldownSec guard prevents repeated
//   requests while a handover is in progress.
// ---------------------------------------------------------------------------
struct UuHandoverHelper
{
    uint32_t                    vehIdx;
    Ptr<NrUeNetDevice>          ueNrDev;
    std::vector<Ptr<LteEnbRrc>> gnbRrcs;     ///< gnbRrcs[j] = LteEnbRrc of gNB j
    std::vector<uint16_t>       gnbCellIds;  ///< gnbCellIds[j] = cell ID of gNB j
    std::vector<Vector>         gnbPos;      ///< (x,y,z) of gNB j
    Time                        lastHoTime{Seconds(-10.0)};

    static constexpr double kCooldownSec     = 1.5;   ///< min s between HO requests
    static constexpr double kCheckIntervalMs = 200.0; ///< polling period (ms)

    static void TriggerIfNeeded(UuHandoverHelper* h)
    {
        Time now = Simulator::Now();

        // Rate-limit: skip while a prior request is still being processed
        if ((now - h->lastHoTime).GetSeconds() < kCooldownSec)
        {
            Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                                &UuHandoverHelper::TriggerIfNeeded, h);
            return;
        }

        // Find nearest gNB by squared 2D Euclidean distance
        Vector   vp      = h->ueNrDev->GetNode()->GetObject<MobilityModel>()->GetPosition();
        uint32_t nearest = 0;
        double   bestD2  = 1e36;
        for (uint32_t j = 0; j < h->gnbPos.size(); ++j)
        {
            double dx = h->gnbPos[j].x - vp.x;
            double dy = h->gnbPos[j].y - vp.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; nearest = j; }
        }

        // Query current serving cell and RNTI from UE RRC
        Ptr<LteUeRrc> ueRrc      = h->ueNrDev->GetRrc();
        uint16_t      curCellId  = ueRrc->GetCellId();
        uint16_t      rnti       = ueRrc->GetRnti();
        uint16_t      targetCell = h->gnbCellIds[nearest];

        // Skip if UE is mid-handover: RNTI may have been removed from source gNB's
        // m_ueMap already, but GetRnti() still returns the old value.
        // Calling SendHandoverRequest with a stale RNTI causes SIGSEGV because
        // GetUeManager's NS_ASSERT_MSG is compiled out in the default build profile.
        if (ueRrc->GetState() != LteUeRrc::CONNECTED_NORMALLY)
        {
            Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                                &UuHandoverHelper::TriggerIfNeeded, h);
            return;
        }

        // Identify source gNB index from serving cell ID
        int32_t srcIdx = -1;
        for (uint32_t j = 0; j < h->gnbCellIds.size(); ++j)
            if (h->gnbCellIds[j] == curCellId) { srcIdx = static_cast<int32_t>(j); break; }

        // Request handover when: source known, target differs, RNTI assigned
        if (srcIdx >= 0 && static_cast<uint32_t>(srcIdx) != nearest && rnti != 0)
        {
            NS_LOG_INFO("UuHO Veh[" << h->vehIdx << "]"
                        << " gNB[" << srcIdx << "](cell=" << curCellId << ")"
                        << "→gNB[" << nearest << "](cell=" << targetCell << ")"
                        << "  rnti=" << rnti
                        << "  dist=" << std::fixed << std::setprecision(0)
                        << std::sqrt(bestD2) << " m"
                        << "  t=" << now.GetSeconds() << " s");
            h->gnbRrcs[srcIdx]->SendHandoverRequest(rnti, targetCell);
            h->lastHoTime = now;
        }

        Simulator::Schedule(MilliSeconds(kCheckIntervalMs),
                            &UuHandoverHelper::TriggerIfNeeded, h);
    }
};

// ---------------------------------------------------------------------------
// main
static void
CrashHandler(int sig)
{
    void* bt[50];
    int n = backtrace(bt, 50);
    const char* msg = sig == SIGABRT ? "\n=== SIGABRT backtrace ===\n"
                                     : "\n=== SIGSEGV backtrace ===\n";
    write(STDERR_FILENO, msg, 30);
    backtrace_symbols_fd(bt, n, STDERR_FILENO);
    write(STDERR_FILENO, "=========================\n", 26);
    signal(sig, SIG_DFL);
    raise(sig);
}

// ---------------------------------------------------------------------------
int
main(int argc, char* argv[])
{
    signal(SIGABRT, CrashHandler);
    signal(SIGSEGV, CrashHandler);
    // -------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------
    uint32_t    nVehicles      = 60;
    uint32_t    nSenders       = 0;   // vehicles that send video (0 = all nVehicles)
    uint32_t    nGnbs          = 4;   // overridden by topoMode below
    uint32_t    nRsus          = 4;   // overridden by topoMode below
    double      roadLength     = 2000.0;   // m (used only for vehicle initial spacing)
    double      vehicleSpeed   = 22.22;    // m/s  (~80 km/h)
    Time        simTime        = Seconds(120.0);
    Time        appStart       = MilliSeconds(500);
    std::string outputDir      = ".";
    std::string simTag         = "highway-mpquic";
    std::string traceFile      = "";   // if set, use SUMO ns-2 mobility trace
    std::string topoMode       = "highway"; // "highway", "urban", "urban_large",
                                            // "gangnam_burst", "gangnam_corridor", "gangnam_nlos"
    bool        logging        = false;

    std::string videoPreset    = "";      // "lo"=0.5Mbps, "sd"=1.0Mbps, "hd"=3.0Mbps; "" = use pktInterval/pktSize
    double      disablePc5At  = -1.0;  // [DIAGNOSTIC] seconds rel. to appStart to call SetPathActive(PC5,false); <0 = disabled
    uint32_t    seed           = 1;      // RNG seed (RngSeedManager::SetSeed + SetRun)
    double      testDist       = 0.0;    // [DIAGNOSTIC] if >0, override RSU[0] to (vehStart + testDist) on highway axis

    uint32_t    epochLenMs   = 5000;  // epoch length for data logging (ms)

    // Background Uu uplink load (non-sender vehicles)
    // Class 1 (bgUuClass1Pct %): persistent low-rate  — 64 kbps, IoT/sensor-like
    // Class 2 (bgUuClass2Pct %): bursty medium-rate   — 512 kbps peak, app-like
    // Class 3 (remainder)      : quasi-persistent high — 2 Mbps, video-upload-like
    //
    // congLevel preset mappings (per-gNB bg UL load at 7 bg-vehicles / gNB):
    //   none   → bgUuEnabled=false                                 (0 Mbps)
    //   light  → C1=70%, C2=25%, C3= 5%  → avg ~209 kbps/vehicle (~1.5 Mbps)
    //   medium → C1=40%, C2=35%, C3=25%  → avg ~615 kbps/vehicle (~4.3 Mbps)
    //   heavy  → C1=10%, C2=20%, C3=70%  → avg ~1.5 Mbps/vehicle (~10 Mbps)
    std::string congLevel    = "none";   // none | light | medium | heavy
    bool     bgUuEnabled   = false;
    uint32_t bgUuClass1Pct = 50;
    uint32_t bgUuClass2Pct = 30;

    // NR Uu
    double   uuFreqHz     = 3.5e9;
    double   uuBandwidth  = 20e6;
    uint16_t uuNumerology = 1;
    double   uuTxPowerDbm = 23.0;
    double   gnbTxPowerDbm= 40.0;

    // NR SL PC5
    double   slFreqHz     = 5.89e9;
    uint16_t slBwRb       = 200;   // 200 × 100 kHz = 20 MHz, μ=1 → 53 usable PRBs
    uint16_t slNumerology = 1;
    double   slTxPowerDbm = 23.0;
    bool     slHarq       = false;  // groupcast HARQ unsupported in this NR SL version
    uint32_t slMcs        = 14;    // NR SL MCS (0-27); lower = more robust, less throughput
    uint32_t slSubchSize  = 25;    // SL subchannel size in RBs; smaller → more subchannels
    bool     slAlwaysLos  = false; // Force AlwaysLOS on SL band (isolates channel model effect)
    bool     slSensing    = true;  // Enable Mode-2 sensing (false = random resource selection)

    // MPQUIC
    uint16_t    rxPort        = 4000;
    std::string schedulerType = "MinRtt";
    uint32_t    pktSize       = 1200;
    Time        pktInterval   = MilliSeconds(20);   // 50 pps

    // gNB/RSU positions depend on topoMode (set after CommandLine::Parse).
    // Supports up to 9 gNBs and 9 RSUs; actual count set per mode.
    double gnbPosX[9]{}, gnbPosY[9]{};
    double rsuPosX[9]{}, rsuPosY[9]{};
    double gnbPosZ[9], rsuPosZ[9];
    std::fill(gnbPosZ, gnbPosZ + 9, 25.0);
    std::fill(rsuPosZ, rsuPosZ + 9,  6.0);

    CommandLine cmd(__FILE__);
    cmd.AddValue("nVehicles",    "Number of vehicles",                nVehicles);
    cmd.AddValue("nSenders",     "Vehicles that send video (0=all)",  nSenders);
    cmd.AddValue("nRsus",        "Number of RSUs (1-9)",              nRsus);
    cmd.AddValue("roadLength",   "Initial vehicle spacing span (m)",  roadLength);
    cmd.AddValue("vehicleSpeed", "Vehicle speed (m/s)",               vehicleSpeed);
    cmd.AddValue("simTime",      "Simulation duration (s)",           simTime);
    cmd.AddValue("appStart",     "App start time (s)",                appStart);
    cmd.AddValue("outputDir",    "Output directory",                  outputDir);
    cmd.AddValue("simTag",       "Simulation tag",                    simTag);
    cmd.AddValue("logging",      "Enable ns-3 logging",               logging);
    cmd.AddValue("traceFile",   "SUMO ns-2 mobility trace (.tcl); empty = ConstantVelocity", traceFile);
    cmd.AddValue("topoMode",    "Node placement: highway|urban|urban_large|gangnam_burst|gangnam_corridor|gangnam_nlos", topoMode);
    cmd.AddValue("uuTxPower",    "NR UE Uu TX power (dBm)",           uuTxPowerDbm);
    cmd.AddValue("slTxPower",    "NR SL TX power (dBm)",              slTxPowerDbm);
    cmd.AddValue("slHarq",       "Enable NR SL HARQ",                 slHarq);
    cmd.AddValue("slMcs",        "NR SL MCS index (0-27)",            slMcs);
    cmd.AddValue("slAlwaysLos",  "Force AlwaysLOS on SL band (root-cause isolation)", slAlwaysLos);
    cmd.AddValue("slSensing",    "Enable Mode-2 sensing (false=random resource selection)", slSensing);
    cmd.AddValue("slSubchSize",  "SL subchannel size in RBs",         slSubchSize);
    cmd.AddValue("schedulerType","MPQUIC scheduler (MinRtt|RoundRobin|Redundant)",  schedulerType);
    cmd.AddValue("pktSize",      "MPQUIC payload size (B)",           pktSize);
    cmd.AddValue("pktInterval",  "MPQUIC send interval (ms)",         pktInterval);
    cmd.AddValue("videoPreset",  "Video preset: lo|sd|hd (overrides pktSize/pktInterval)", videoPreset);
    cmd.AddValue("disablePc5At","[DIAGNOSTIC] Seconds relative to appStart to hard-disable PC5 paths (<0=off)", disablePc5At);
    cmd.AddValue("seed",         "RNG seed (sets RngSeedManager::SetSeed + SetRun)", seed);
    cmd.AddValue("testDist",     "[DIAGNOSTIC] RSU[0] distance (m) from vehicle start for highway distance-sweep (0=disabled)", testDist);
    cmd.AddValue("epochLenMs",   "Epoch length for data logging (ms)", epochLenMs);

    // Diagnostic per-packet logs
    bool        diagLog       = false;       // enable per-packet TX/RX diagnostic logs
    cmd.AddValue("diagLog",       "Enable per-packet TX/RX diagnostic CSV logs", diagLog);
    cmd.AddValue("congLevel",     "Uu uplink congestion preset: none|light|medium|heavy (overrides bgUuEnabled/ClassPct)",
                 congLevel);
    cmd.AddValue("bgUuEnabled",   "Enable background Uu uplink load on non-sender vehicles",
                 bgUuEnabled);
    cmd.AddValue("bgUuClass1Pct", "% of bg vehicles as persistent low-rate Class 1 (64 kbps)",
                 bgUuClass1Pct);
    cmd.AddValue("bgUuClass2Pct", "% of bg vehicles as bursty medium Class 2 (512 kbps peak)",
                 bgUuClass2Pct);
    cmd.Parse(argc, argv);

    // Apply congestion level preset (overrides manual bgUuEnabled/ClassPct if set)
    NS_ABORT_MSG_IF(congLevel != "none" && congLevel != "light" &&
                    congLevel != "medium" && congLevel != "heavy",
                    "congLevel must be none|light|medium|heavy");
    if (congLevel == "light") {
        bgUuEnabled = true; bgUuClass1Pct = 70; bgUuClass2Pct = 25;  // C3=5%
    } else if (congLevel == "medium") {
        bgUuEnabled = true; bgUuClass1Pct = 40; bgUuClass2Pct = 35;  // C3=25%
    } else if (congLevel == "heavy") {
        bgUuEnabled = true; bgUuClass1Pct = 10; bgUuClass2Pct = 20;  // C3=70%
    }

    // Apply RNG seed
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(seed);

    // Resolve nSenders: 0 means all vehicles send
    if (nSenders == 0 || nSenders > nVehicles)
        nSenders = nVehicles;

    // Randomise sender selection per seed so different seeds pick different
    // vehicle subsets, improving training-set diversity across replications.
    std::vector<uint32_t> allVehIdx(nVehicles);
    std::iota(allVehIdx.begin(), allVehIdx.end(), 0);
    {
        std::mt19937 senderRng(static_cast<uint32_t>(seed) * 2654435761u);
        std::shuffle(allVehIdx.begin(), allVehIdx.end(), senderRng);
    }
    std::vector<uint32_t> senderIndices(allVehIdx.begin(), allVehIdx.begin() + nSenders);
    std::sort(senderIndices.begin(), senderIndices.end());
    std::unordered_set<uint32_t> senderSet(senderIndices.begin(), senderIndices.end());
    std::vector<uint32_t> bgIndices;
    bgIndices.reserve(nVehicles - nSenders);
    for (uint32_t i = 0; i < nVehicles; ++i)
        if (!senderSet.count(i)) bgIndices.push_back(i);

    // Apply video preset (overrides pktSize / pktInterval)
    // lo=0.5Mbps, sd=1.0Mbps, hd=3.0Mbps  deadline=100ms
    if (!videoPreset.empty())
    {
        NS_ABORT_MSG_IF(videoPreset != "lo" && videoPreset != "sd" && videoPreset != "hd",
                        "videoPreset must be 'lo', 'sd', or 'hd'");
        if      (videoPreset == "lo") { pktSize = 1200; pktInterval = MilliSeconds(19); }
        else if (videoPreset == "sd") { pktSize = 1200; pktInterval = MilliSeconds(10); }
        else                          { pktSize = 1400; pktInterval = MilliSeconds(4);  }
    }

    NS_ABORT_MSG_IF(topoMode != "highway" && topoMode != "highway_tau" &&
                    topoMode != "urban" && topoMode != "urban_large" &&
                    topoMode != "gangnam_burst" && topoMode != "gangnam_corridor" &&
                    topoMode != "gangnam_nlos" &&
                    topoMode != "nagoya_tile_c" &&
                    topoMode != "mechanism_G" && topoMode != "mechanism_R" && topoMode != "mechanism_B",
                    "topoMode must be highway|urban|urban_large|gangnam_burst|gangnam_corridor|"
                    "gangnam_nlos|nagoya_tile_c|"
                    "mechanism_G|mechanism_R|mechanism_B");

    // Set gNB/RSU count and positions based on topoMode
    if (topoMode == "gangnam_burst")
    {
        // gangnam_2km_mobility.tcl: X=4000~7000, Y=6000~8000 (43 vehicles)
        // 2×2 gNBs at corners, 4 RSUs at interior junctions
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] = 4300.0;  gnbPosY[0] = 6700.0;  // SW
        gnbPosX[1] = 6200.0;  gnbPosY[1] = 6700.0;  // SE
        gnbPosX[2] = 4300.0;  gnbPosY[2] = 7800.0;  // NW
        gnbPosX[3] = 6200.0;  gnbPosY[3] = 7800.0;  // NE
        rsuPosX[0] = 4700.0;  rsuPosY[0] = 7100.0;
        rsuPosX[1] = 5300.0;  rsuPosY[1] = 6500.0;
        rsuPosX[2] = 5800.0;  rsuPosY[2] = 7400.0;
        rsuPosX[3] = 5100.0;  rsuPosY[3] = 7700.0;
    }
    else if (topoMode == "gangnam_corridor")
    {
        // gangnam_corridor_mobility.tcl: X=4000~6000, Y=4000~6000 (42 vehicles)
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] = 4300.0;  gnbPosY[0] = 4700.0;  // SW
        gnbPosX[1] = 5700.0;  gnbPosY[1] = 4700.0;  // SE
        gnbPosX[2] = 4300.0;  gnbPosY[2] = 5700.0;  // NW
        gnbPosX[3] = 5700.0;  gnbPosY[3] = 5700.0;  // NE
        rsuPosX[0] = 4600.0;  rsuPosY[0] = 5100.0;
        rsuPosX[1] = 5000.0;  rsuPosY[1] = 4600.0;
        rsuPosX[2] = 5400.0;  rsuPosY[2] = 5300.0;
        rsuPosX[3] = 5000.0;  rsuPosY[3] = 5700.0;
    }
    else if (topoMode == "gangnam_nlos")
    {
        // gangnam_nlos_mobility.tcl: X=2000~4000, Y=2000~4000 (37 vehicles, dense NLOS)
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] = 2300.0;  gnbPosY[0] = 2700.0;  // SW
        gnbPosX[1] = 3700.0;  gnbPosY[1] = 2700.0;  // SE
        gnbPosX[2] = 2300.0;  gnbPosY[2] = 3700.0;  // NW
        gnbPosX[3] = 3700.0;  gnbPosY[3] = 3700.0;  // NE
        rsuPosX[0] = 2600.0;  rsuPosY[0] = 3100.0;
        rsuPosX[1] = 3000.0;  rsuPosY[1] = 2600.0;
        rsuPosX[2] = 3400.0;  rsuPosY[2] = 3300.0;
        rsuPosX[3] = 3000.0;  rsuPosY[3] = 3700.0;
    }
    else if (topoMode == "nagoya_tile_c")
    {
        // Tile C (High, foot=59.6%): inner-city residential, SUMO X=[14000,16000] Y=[14000,16000]
        // gNB2 NW displaced +5.7m; gNB3 NE displaced +5.3m (both polygon indoor)
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] =  300.0;  gnbPosY[0] =  700.0;
        gnbPosX[1] = 1700.0;  gnbPosY[1] =  700.0;
        gnbPosX[2] =  295.8;  gnbPosY[2] = 1703.8;   // was (300,1700) — polygon corrected
        gnbPosX[3] = 1700.5;  gnbPosY[3] = 1694.7;   // was (1700,1700) — polygon corrected
        rsuPosX[0] =  511.5;  rsuPosY[0] =  801.1;
        rsuPosX[1] =  117.3;  rsuPosY[1] = 1896.3;
        rsuPosX[2] = 1272.8;  rsuPosY[2] =  104.6;
        rsuPosX[3] =  907.9;  rsuPosY[3] = 1859.8;
    }
    else if (topoMode == "urban_large")
    {
        // 3×2 grid (6 gNBs) for gangnam_large SUMO area (X:56-7025, Y:354-8394)
        // ISD ≈ 2.5 km.  Heights: gNB=25 m (macro), RSU=6 m (pole).
        // 8 RSUs in 2 E-W rows between gNB rows.
        nGnbs = 6;  nRsus = 8;
        gnbPosX[0] = 1200.0;  gnbPosY[0] = 2500.0;  // SW
        gnbPosX[1] = 3600.0;  gnbPosY[1] = 2500.0;  // S
        gnbPosX[2] = 6000.0;  gnbPosY[2] = 2500.0;  // SE
        gnbPosX[3] = 1200.0;  gnbPosY[3] = 6200.0;  // NW
        gnbPosX[4] = 3600.0;  gnbPosY[4] = 6200.0;  // N
        gnbPosX[5] = 6000.0;  gnbPosY[5] = 6200.0;  // NE
        // RSU row-S (y=1400): one per inter-gNB gap + flanks
        rsuPosX[0] =  700.0;  rsuPosY[0] = 1400.0;
        rsuPosX[1] = 2400.0;  rsuPosY[1] = 1400.0;
        rsuPosX[2] = 4700.0;  rsuPosY[2] = 1400.0;
        rsuPosX[3] = 6500.0;  rsuPosY[3] = 1400.0;
        // RSU row-M (y=4800): mid-band between gNB rows
        rsuPosX[4] =  700.0;  rsuPosY[4] = 4800.0;
        rsuPosX[5] = 2400.0;  rsuPosY[5] = 4800.0;
        rsuPosX[6] = 4700.0;  rsuPosY[6] = 4800.0;
        rsuPosX[7] = 6500.0;  rsuPosY[7] = 4800.0;
    }
    else if (topoMode == "urban")
    {
        // 2×2 grid (4 gNBs) for gangnam_small SUMO area (X: 488–4672, Y: 570–4815)
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] = 1500.0;  gnbPosY[0] = 1610.0;  // SW
        gnbPosX[1] = 3600.0;  gnbPosY[1] = 1610.0;  // SE
        gnbPosX[2] = 1500.0;  gnbPosY[2] = 3760.0;  // NW
        gnbPosX[3] = 3600.0;  gnbPosY[3] = 3760.0;  // NE
        rsuPosX[0] = 1000.0;  rsuPosY[0] = 2700.0;
        rsuPosX[1] = 2500.0;  rsuPosY[1] = 2700.0;
        rsuPosX[2] = 4200.0;  rsuPosY[2] = 2700.0;
        rsuPosX[3] = 2500.0;  rsuPosY[3] = 1200.0;
    }
    else if (topoMode == "mechanism_G")
    {
        // Controlled mechanism study — gNB transition only.
        // Corridor: X=2000~4000, Y=3000.  Speed 60 km/h.
        // 4 gNBs at corners → Voronoi boundary at X=3000 on the road.
        // 1 RSU at center (3000,3000) → NO RSU transition anywhere on road.
        // tau=100ms: pred_pos crosses gNB boundary before vehicle does.
        nGnbs = 4;  nRsus = 1;
        gnbPosX[0] = 2300.0;  gnbPosY[0] = 2700.0;  // SW
        gnbPosX[1] = 3700.0;  gnbPosY[1] = 2700.0;  // SE
        gnbPosX[2] = 2300.0;  gnbPosY[2] = 3700.0;  // NW
        gnbPosX[3] = 3700.0;  gnbPosY[3] = 3700.0;  // NE
        rsuPosX[0] = 3000.0;  rsuPosY[0] = 3000.0;  // single RSU at center
    }
    else if (topoMode == "mechanism_R")
    {
        // Controlled mechanism study — RSU transition only.
        // Corridor: X=2000~4000, Y=3000.  Speed 60 km/h.
        // 1 gNB at center (3000,3000) → NO gNB transition on road.
        // 2 RSUs at (2000,3000) and (4000,3000) → Voronoi boundary at X=3000.
        // tau=100ms: pred_pos crosses RSU boundary before vehicle does.
        nGnbs = 1;  nRsus = 2;
        gnbPosX[0] = 3000.0;  gnbPosY[0] = 3000.0;  // single gNB at center
        rsuPosX[0] = 2000.0;  rsuPosY[0] = 3000.0;  // West RSU
        rsuPosX[1] = 4000.0;  rsuPosY[1] = 3000.0;  // East RSU
    }
    else if (topoMode == "mechanism_B")
    {
        // Controlled mechanism study — building-induced blockage only.
        // Corridor: X=2000~4000, Y=3000.  Same gangnam_nlos buildings.
        // 1 gNB at center, 1 RSU at center → no G or R transition.
        // Buildings cause NLOS at various X positions → B events.
        nGnbs = 1;  nRsus = 1;
        gnbPosX[0] = 3000.0;  gnbPosY[0] = 3000.0;
        rsuPosX[0] = 3000.0;  rsuPosY[0] = 3000.0;
    }
    else if (topoMode == "highway_tau")
    {
        // gNBs placed AT RSU Voronoi boundaries so Uu is strongest exactly
        // where PC5 is transitioning RSUs — forces two-path at handover zones.
        // RSU Voronoi boundaries: x=1600 (RSU0→1), x=2400 (RSU1→2), x=3200 (RSU2→3)
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] = 1000.0;  gnbPosY[0] = 2975.0;  // start coverage
        gnbPosX[1] = 1600.0;  gnbPosY[1] = 2975.0;  // RSU[0->1] boundary
        gnbPosX[2] = 2400.0;  gnbPosY[2] = 2975.0;  // RSU[1->2] boundary
        gnbPosX[3] = 3200.0;  gnbPosY[3] = 2975.0;  // RSU[2->3] boundary
        rsuPosX[0] = 1200.0;  rsuPosY[0] = 3000.0;
        rsuPosX[1] = 2000.0;  rsuPosY[1] = 3000.0;
        rsuPosX[2] = 2800.0;  rsuPosY[2] = 3000.0;
        rsuPosX[3] = 3600.0;  rsuPosY[3] = 3000.0;
    }
    else // highway
    {
        // Linear 4 gNBs at y=2975 (25 m lateral offset from road at y=3000)
        nGnbs = 4;  nRsus = 4;
        gnbPosX[0] = 1000.0;  gnbPosY[0] = 2975.0;
        gnbPosX[1] = 2000.0;  gnbPosY[1] = 2975.0;
        gnbPosX[2] = 3000.0;  gnbPosY[2] = 2975.0;
        gnbPosX[3] = 4000.0;  gnbPosY[3] = 2975.0;
        rsuPosX[0] = 1200.0;  rsuPosY[0] = 3000.0;
        rsuPosX[1] = 2000.0;  rsuPosY[1] = 3000.0;
        rsuPosX[2] = 2800.0;  rsuPosY[2] = 3000.0;
        rsuPosX[3] = 3600.0;  rsuPosY[3] = 3000.0;
    }
    // testDist>0: override RSU[0] for controlled distance-sweep in highway mode
    if (testDist > 0.0 && topoMode == "highway")
    {
        const double hwyStartX = 1000.0, hwyRoadY = 3000.0;
        rsuPosX[0] = hwyStartX + testDist;  rsuPosY[0] = hwyRoadY;
    }
    NS_ABORT_MSG_IF(nRsus < 1 || nRsus > 9, "nRsus out of range");

    // CSV infrastructure override: if <traceDir>/<topoMode>_infra.csv exists,
    // it overrides the hardcoded gNB/RSU positions above.
    // Format: one entry per line — "gnb,x,y" or "rsu,x,y" (lines starting with '#' are skipped).
    if (!traceFile.empty())
    {
        std::string traceDir0  = traceFile.substr(0, traceFile.rfind('/'));
        std::string infraCsv   = traceDir0 + "/" + topoMode + "_infra.csv";
        std::ifstream infraIfs(infraCsv);
        if (infraIfs.is_open())
        {
            uint32_t ng = 0, nr = 0;
            std::string infraLine;
            while (std::getline(infraIfs, infraLine))
            {
                if (infraLine.empty() || infraLine[0] == '#') continue;
                std::istringstream ss(infraLine);
                std::string type; double x, y; char c1, c2;
                if (!(ss >> type >> c1 >> x >> c2 >> y)) continue;
                double z = -1.0; char c3;
                if (ss >> c3 >> z) { /* z provided */ }
                if (type == "gnb" && ng < 9)
                {
                    gnbPosX[ng] = x; gnbPosY[ng] = y;
                    if (z > 0) gnbPosZ[ng] = z;
                    ++ng;
                }
                else if (type == "rsu" && nr < 9)
                {
                    rsuPosX[nr] = x; rsuPosY[nr] = y;
                    if (z > 0) rsuPosZ[nr] = z;
                    ++nr;
                }
            }
            if (ng > 0 || nr > 0)
            {
                if (ng > 0) nGnbs = ng;
                if (nr > 0) nRsus = nr;
                NS_LOG_UNCOND("[InfraCsv] Loaded " << nGnbs << " gNBs + " << nRsus << " RSUs from " << infraCsv);
            }
        }
    }

    // Load buildings for V2V_Urban channel condition model.
    // BuildingsChannelConditionModel requires Building objects in the global BuildingList.
    if (topoMode == "gangnam_corridor" || topoMode == "gangnam_nlos" ||
        topoMode == "gangnam_burst"    || topoMode == "urban" || topoMode == "urban_large" ||
        topoMode == "nagoya_tile_c" ||
        topoMode == "mechanism_B")
    {
        std::string traceDir = traceFile.substr(0, traceFile.rfind('/'));
        std::string buildingCsv;
        if      (topoMode == "gangnam_corridor") buildingCsv = traceDir + "/gangnam_corridor_buildings.csv";
        else if (topoMode == "gangnam_nlos")     buildingCsv = traceDir + "/gangnam_nlos_buildings.csv";
        else if (topoMode == "gangnam_burst")    buildingCsv = traceDir + "/gangnam_corridor_buildings.csv";
        else if (topoMode == "nagoya_tile_c")    buildingCsv = traceDir + "/nagoya_tile_c_buildings.csv";
        else if (topoMode == "mechanism_B")      buildingCsv = traceDir + "/gangnam_nlos_buildings.csv";
        else                                     buildingCsv = traceDir + "/gangnam_corridor_buildings.csv";

        std::ifstream bfile(buildingCsv);
        NS_ABORT_MSG_IF(!bfile.is_open(), "Cannot open buildings CSV: " + buildingCsv);

        std::string line;
        std::getline(bfile, line); // skip header
        uint32_t bCount = 0;
        while (std::getline(bfile, line))
        {
            std::istringstream ss(line);
            double xmin, xmax, ymin, ymax, zmin, zmax;
            char comma;
            if (!(ss >> xmin >> comma >> xmax >> comma >> ymin >> comma
                     >> ymax >> comma >> zmin >> comma >> zmax))
                continue;
            Ptr<Building> b = CreateObject<Building>();
            b->SetBoundaries(Box(xmin, xmax, ymin, ymax, zmin, zmax));
            ++bCount;
        }
        NS_LOG_INFO("[Buildings] loaded " << bCount << " buildings from " << buildingCsv);
    }

    NS_ABORT_MSG_IF(simTime <= appStart, "simTime must be > appStart");

    // initialise per-vehicle Uu state storage
    g_ueState.resize(nVehicles);

    if (logging)
    {
        LogComponentEnable("CttcNrV2xHighwayMpquic", LOG_LEVEL_INFO);
        LogComponentEnable("MpQuicSocketBase",        LOG_LEVEL_DEBUG);
        LogComponentEnable("MpQuicReceiverApp",       LOG_LEVEL_INFO);
        LogComponentEnable("NrSlHelper",              LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // -------------------------------------------------------------------
    // Nodes
    // -------------------------------------------------------------------
    NodeContainer vehicleNodes; vehicleNodes.Create(nVehicles);   // NodeId 0..N-1
    NodeContainer gnbNodes;     gnbNodes.Create(nGnbs);
    NodeContainer rsuNodes;     rsuNodes.Create(nRsus);
    NodeContainer remoteHostNC; remoteHostNC.Create(1);
    Ptr<Node>     remoteHost = remoteHostNC.Get(0);

    // SL UE container: vehicles first, then all RSU nodes
    NodeContainer slUeContainer;
    for (uint32_t i = 0; i < nVehicles; ++i)
        slUeContainer.Add(vehicleNodes.Get(i));
    for (uint32_t r = 0; r < nRsus; ++r)
        slUeContainer.Add(rsuNodes.Get(r));

    // -------------------------------------------------------------------
    // Mobility
    // -------------------------------------------------------------------
    // gNBs: fixed Gangnam positions (25m height)
    {
        MobilityHelper gnbMob;
        gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        for (uint32_t j = 0; j < nGnbs; ++j)
            pos->Add(Vector(gnbPosX[j], gnbPosY[j], gnbPosZ[j]));
        gnbMob.SetPositionAllocator(pos);
        gnbMob.Install(gnbNodes);
    }

    // RSUs: fixed positions (default height 6m, overridable via infra CSV)
    {
        MobilityHelper rsuMob;
        rsuMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        for (uint32_t r = 0; r < nRsus; ++r)
            pos->Add(Vector(rsuPosX[r], rsuPosY[r], rsuPosZ[r]));
        rsuMob.SetPositionAllocator(pos);
        rsuMob.Install(rsuNodes);
    }

    // Vehicles: SUMO trace or simple highway
    if (!traceFile.empty())
    {
        // ConstantVelocityMobilityModel: required by Ns2MobilityHelper which creates/updates
        // ConstantVelocityMobilityModel objects. Using WaypointMobilityModel here would add
        // a second mobility model that stays at the allocator position, causing AttachToClosestEnb
        // and UuHandoverHelper to use wrong positions.
        MobilityHelper vehMob;
        vehMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        // Initial positions are overwritten by Ns2MobilityHelper from the trace file.
        // Place at (0,0,1.5) as placeholder; Ns2MobilityHelper sets the real initial pos.
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        for (uint32_t i = 0; i < nVehicles; ++i)
            pos->Add(Vector(0.0, 0.0, 1.5));
        vehMob.SetPositionAllocator(pos);
        vehMob.Install(vehicleNodes);

        Ns2MobilityHelper ns2mob(traceFile);
        ns2mob.Install(vehicleNodes.Begin(), vehicleNodes.End());  // only vehicles — prevents trace nodes 5..14 from overwriting gNB/RSU positions
        std::cout << "  Mobility:     SUMO trace: " << traceFile << "\n";

        // Log sender vehicle initial positions (to identify SUMO vehicle IDs)
        std::cout << "  Senders (" << nSenders << " of " << nVehicles << "):\n";
        for (uint32_t i = 0; i < nSenders; ++i)
        {
            auto mob = vehicleNodes.Get(senderIndices[i])->GetObject<MobilityModel>();
            Vector pos = mob ? mob->GetPosition() : Vector(0,0,0);
            std::cout << "    node" << senderIndices[i]
                      << "  init_pos=(" << std::fixed << std::setprecision(1)
                      << pos.x << "," << pos.y << ")\n";
        }
    }
    else
    {
        MobilityHelper vehMob;
        vehMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        double spacing = (nVehicles > 1) ? (roadLength * 0.9) / (nVehicles - 1) : 0.0;
        // Highway runs east–west at y=3000 m, aligned with gNB2(900,3000)/gNB3(3100,3000)
        // and RSUs at y≈2361–3190.  Starting x=1800 keeps all vehicles within RSU1-3 SL range.
        const double roadY    = 3000.0;
        const double startX   = 1000.0;  // aligned with gNB0 for best initial Uu coverage
        for (uint32_t i = 0; i < nVehicles; ++i)
            pos->Add(Vector(startX + i * spacing, roadY, 1.5));
        vehMob.SetPositionAllocator(pos);
        vehMob.Install(vehicleNodes);
        for (uint32_t i = 0; i < nVehicles; ++i)
            vehicleNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()
                ->SetVelocity(Vector(vehicleSpeed, 0.0, 0.0));
        std::cout << "  Mobility:     ConstantVelocity " << vehicleSpeed << " m/s\n";
    }

    // remoteHost: static
    {
        MobilityHelper rhMob;
        rhMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
        pos->Add(Vector(0.0, -100.0, 0.0));
        rhMob.SetPositionAllocator(pos);
        rhMob.Install(remoteHostNC);
    }

    // -------------------------------------------------------------------
    // Internet stack
    // -------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(vehicleNodes);
    internet.Install(rsuNodes);
    internet.Install(remoteHostNC);

    // Vehicles are endpoints, not routers — disable IP forwarding so they never
    // generate ICMP TTL-exceeded via SL (which crashes when SL TX DRB is absent).
    // RSU nodes keep forwarding ON (needed for SL→backhaul IP forwarding).
    for (uint32_t i = 0; i < nVehicles; ++i)
        vehicleNodes.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(false));

    // Install MobilityBuildingInfo on all nodes (required by BuildingsChannelConditionModel
    // used by V2V_Urban SL channel model).  Safe to call even with an empty building list
    // (highway mode: no buildings → always returns outdoor/LOS condition).
    BuildingsHelper::Install(vehicleNodes);
    BuildingsHelper::Install(rsuNodes);
    BuildingsHelper::Install(gnbNodes);

    // Populate global vehicle mobility table (used by TakeEpochSnap for nearby_vehicle_count)
    g_allVehicleMobs.clear();
    for (uint32_t i = 0; i < nVehicles; ++i)
        g_allVehicleMobs.push_back(vehicleNodes.Get(i)->GetObject<MobilityModel>());

    // ===================================================================
    // NR helper — shared for Uu and SL bands
    // ===================================================================
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper>  bfHelper  = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper>                nrHelper  = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(2)));  // gNB→EPC fiber fronthaul (3GPP TR 38.913)

    // Handover algorithm will be wired manually after gNB installation
    // (NrHelper does not expose SetHandoverAlgorithmType)

    // -------------------------------------------------------------------
    // Band 1: NR Uu  (3.5 GHz)
    // -------------------------------------------------------------------
    // Urban building modes (gangnam_*/nagoya_tile_c/urban*) use UMa_Buildings:
    //   ThreeGppUmaPropagationLossModel + BuildingsChannelConditionModel.
    //   LOS/NLOS is determined geometrically from loaded building objects, which
    //   correctly captures urban blockage without the stochastic bias of UMa.
    // Highway / open-terrain modes keep UMa_LoS (AlwaysLos) because
    //   DirectPathBeamforming degrades severely in probabilistic NLOS and gNBs
    //   at 25 m are predominantly LOS on open terrain anyway.
    bool useUmaBuildings = (topoMode == "gangnam_corridor" || topoMode == "gangnam_nlos" ||
                            topoMode == "gangnam_burst" ||
                            topoMode == "urban" || topoMode == "urban_large" ||
                            topoMode == "nagoya_tile_c" ||
                            topoMode == "mechanism_B");
    BandwidthPartInfo::Scenario uuScenario = useUmaBuildings ? BandwidthPartInfo::UMa_Buildings
                                                             : BandwidthPartInfo::UMa_LoS;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf uuBandConf(uuFreqHz, uuBandwidth, 1, uuScenario);
    OperationBandInfo uuBand = ccBwpCreator.CreateOperationBandContiguousCc(uuBandConf);

    // UpdatePeriod=0: channel matrix recomputed on every query (fresh fast-fading).
    // BuildingsChannelConditionModel has no UpdatePeriod (recomputes geometrically per call).
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    nrHelper->InitializeOperationBand(&uuBand);
    BandwidthPartInfoPtrVector uuBwps = CcBwpCreator::GetAllBwps({uuBand});

    // Packet::EnableChecking() removed: it caused NS_FATAL_ERROR in the NR SL
    // PDCP path when RLC reordering-timer expiry delivers continuation fragments
    // that don't start with LteSlPdcpHeader (standard reassembly edge case).
    // ns-3 default is checking disabled; re-enabling it here caused crashes.
    Packet::EnablePrinting();

    bfHelper->SetAttribute("BeamformingMethod",
                           TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // gNB antenna
    nrHelper->SetGnbAntennaAttribute("NumRows",    UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    // UE antenna (Uu)
    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Install gNBs and vehicle Uu devices
    NetDeviceContainer gnbNetDevs     = nrHelper->InstallGnbDevice(gnbNodes, uuBwps);
    NetDeviceContainer vehicleNrDevs  = nrHelper->InstallUeDevice(vehicleNodes, uuBwps);

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(gnbNetDevs,    stream);
    stream += nrHelper->AssignStreams(vehicleNrDevs, stream);

    // Configure each gNB
    for (uint32_t j = 0; j < nGnbs; ++j)
    {
        nrHelper->GetGnbPhy(gnbNetDevs.Get(j), 0)
            ->SetAttribute("Numerology", UintegerValue(uuNumerology));
        nrHelper->GetGnbPhy(gnbNetDevs.Get(j), 0)
            ->SetAttribute("TxPower",   DoubleValue(gnbTxPowerDbm));
        DynamicCast<NrGnbNetDevice>(gnbNetDevs.Get(j))->UpdateConfig();
    }
    // Configure each vehicle Uu PHY
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        nrHelper->GetUePhy(vehicleNrDevs.Get(i), 0)
            ->SetAttribute("TxPower", DoubleValue(uuTxPowerDbm));
        DynamicCast<NrUeNetDevice>(vehicleNrDevs.Get(i))->UpdateConfig();
    }

    // Increase SRS periodicity so the gNB SRS pool can accommodate all vehicles.
    // Default is 40 (only 40 simultaneous UEs per gNB), which is insufficient
    // for large vehicle counts.  320 supports up to 320 simultaneous UEs per gNB.
    for (uint32_t j = 0; j < nGnbs; ++j)
    {
        auto gnbDev = DynamicCast<NrGnbNetDevice>(gnbNetDevs.Get(j));
        // SrsPeriodicity pool: 320 supports up to 160 UEs/gNB. Enough for 100 vehicles ÷ 6 gNBs.
        gnbDev->GetRrc()->SetAttribute("SrsPeriodicity", UintegerValue(320));
    }

    // X2 interface for handover
    for (uint32_t j = 0; j < nGnbs; ++j)
        for (uint32_t k = j + 1; k < nGnbs; ++k)
            epcHelper->AddX2Interface(gnbNodes.Get(j), gnbNodes.Get(k));

    // -------------------------------------------------------------------
    // Position-based Uu handover (replaces the non-functional A3-RSRP chain).
    // NrUePhy::ReportUeMeasurements() has the RRC reporting line commented
    // out in ns-3-nr, so A3RsrpHandoverAlgorithm never receives measurements.
    // We bypass it entirely and call LteEnbRrc::SendHandoverRequest() directly
    // based on vehicle-to-gNB Euclidean distance.
    // X2 interface (set up above) is still required for the handover to proceed.
    // -------------------------------------------------------------------
    std::vector<UuHandoverHelper> uuHoHelpers(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        auto& h   = uuHoHelpers[i];
        h.vehIdx  = i;
        h.ueNrDev = DynamicCast<NrUeNetDevice>(vehicleNrDevs.Get(i));
        for (uint32_t j = 0; j < nGnbs; ++j)
        {
            auto gnbDev = DynamicCast<NrGnbNetDevice>(gnbNetDevs.Get(j));
            h.gnbRrcs.push_back(gnbDev->GetRrc());
            // Use CC-level cell ID (GetCellIds()[0]), NOT the device-level GetCellId().
            // NrHelper allocates two consecutive IDs per gNB:
            //   device-level: m_cellIdCounter++ (1, 3, 5, 7 for 4 gNBs)
            //   CC-level     : m_cellIdCounter++ (2, 4, 6, 8)
            // The UE RRC camps on the CC-level ID, and the X2 interface is also
            // keyed on CC-level IDs (NrPointToPointEpcHelper::DoAddX2Interface uses
            // GetCellIds()[0]).  Using the device-level ID causes srcIdx=-1 forever.
            h.gnbCellIds.push_back(gnbDev->GetCellIds().at(0));
            h.gnbPos.push_back(Vector(gnbPosX[j], gnbPosY[j], gnbPosZ[j]));
        }
        // Stagger first check across vehicles: t = 5.0 + i*500ms
        // → avoids simultaneous SendHandoverRequest on the same gNB,
        //   which can crash LteEnbRrc when many UEs try to HO simultaneously.
        // Start at t=5s to let MPQUIC/NR state machine stabilize first.
        Simulator::Schedule(Seconds(5.0) + MilliSeconds(500 * i),
                            &UuHandoverHelper::TriggerIfNeeded, &h);
    }

    // -------------------------------------------------------------------
    // Band 2: NR SL PC5  (5.89 GHz, V2V_Highway)
    // -------------------------------------------------------------------
    uint8_t bwpIdSl = 0;
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing",  BooleanValue(slSensing));
    nrHelper->SetUeMacAttribute("T1",             UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2",             UintegerValue(49)); // max safe for RRI=50ms: (49-2+1)*1ms=48ms<50ms
    nrHelper->SetUeMacAttribute("ActivePoolId",   UintegerValue(0));
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdSl));

    CcBwpCreator::SimpleOperationBandConf slBandConf(slFreqHz, slBwRb * 100e3, 1,
                                                      BandwidthPartInfo::V2V_Urban);
    OperationBandInfo slBand = ccBwpCreator.CreateOperationBandContiguousCc(slBandConf);

    nrHelper->SetUeAntennaAttribute("NumRows",    UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(slTxPowerDbm));

    // SetChannelConditionModelAttribute("UpdatePeriod") would fail here because the NrHelper's
    // condition-model factory is typed to AlwaysLosChannelConditionModel or
    // BuildingsChannelConditionModel (from the Uu band above), neither of which has an
    // UpdatePeriod attribute.  Use Config::SetDefault instead, which applies to all
    // ThreeGppChannelConditionModel instances (V2vUrban for the SL band).
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(MilliSeconds(200)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&slBand);

    if (slAlwaysLos)
    {
        // Override the V2V_Urban stochastic condition model with AlwaysLOS on the SL band only.
        // The previous Config::Set("/ChannelList/*/...") wildcard also hit the Uu band channels,
        // causing simulation artifacts when combined with EnableSensing=true.
        for (auto& cc : slBand.m_cc)
            for (auto& bwp : cc->m_bwp)
                if (auto loss = DynamicCast<ThreeGppPropagationLossModel>(bwp->m_propagation))
                    loss->SetChannelConditionModel(CreateObject<AlwaysLosChannelConditionModel>());
        std::cout << "  [SL] AlwaysLOS override: SL band links forced to LOS\n";
    }

    BandwidthPartInfoPtrVector slBwps = CcBwpCreator::GetAllBwps({slBand});

    // slNetDevs[0..N-1] = vehicle SL, slNetDevs[N..N+nRsus-1] = RSU SL
    NetDeviceContainer slNetDevs = nrHelper->InstallUeDevice(slUeContainer, slBwps);
    stream += nrHelper->AssignStreams(slNetDevs, stream);

    for (uint32_t i = 0; i < nVehicles + nRsus; ++i)
        DynamicCast<NrUeNetDevice>(slNetDevs.Get(i))->UpdateConfig();

    // -------------------------------------------------------------------
    // NrSlHelper
    // -------------------------------------------------------------------
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);
    nrSlHelper->SetSlErrorModel("ns3::NrEesmIrT1");
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(slMcs));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdSl);
    nrSlHelper->PrepareUeForSidelink(slNetDevs, bwpIdContainer);

    // -------------------------------------------------------------------
    // EPC IP assignment
    //   vehicleNrDevs[i] → 7.0.0.(2+i)        Uu IPs
    //   slNetDevs[i]      → 7.0.0.(2+N+i)      vehicle SL IPs
    //   slNetDevs[N]      → 7.0.0.(2+N+N)      RSU SL IP
    // -------------------------------------------------------------------
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4Routing;

    // Uu IPs
    std::vector<Ipv4InterfaceContainer> vehicleUuIps(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
        vehicleUuIps[i] = epcHelper->AssignUeIpv4Address(
                              NetDeviceContainer(vehicleNrDevs.Get(i)));

    // SL IPs (vehicles)
    std::vector<Ipv4InterfaceContainer> vehicleSlIps(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
        vehicleSlIps[i] = epcHelper->AssignUeIpv4Address(
                              NetDeviceContainer(slNetDevs.Get(i)));

    // SL IPs (RSUs)
    std::vector<Ipv4InterfaceContainer> rsuSlIps(nRsus);
    for (uint32_t r = 0; r < nRsus; ++r)
        rsuSlIps[r] = epcHelper->AssignUeIpv4Address(
                          NetDeviceContainer(slNetDevs.Get(nVehicles + r)));

    // Set default route for each vehicle via EPC gateway (Uu)
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        Ptr<Ipv4StaticRouting> vehRoute =
            ipv4Routing.GetStaticRouting(vehicleNodes.Get(i)->GetObject<Ipv4>());
        vehRoute->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach all vehicle Uu devices to closest gNB
    nrHelper->AttachToClosestEnb(vehicleNrDevs, gnbNetDevs);

    // Log per-gNB UE distribution (sender vs background)
    {
        std::vector<uint32_t> gnbSenderCnt(nGnbs, 0);
        std::vector<uint32_t> gnbBgCnt(nGnbs, 0);
        for (uint32_t i = 0; i < nVehicles; ++i)
        {
            auto vehMob = vehicleNodes.Get(i)->GetObject<MobilityModel>();
            if (!vehMob) continue;
            Vector vpos = vehMob->GetPosition();
            uint32_t nearest = 0;
            double bestDist = 1e18;
            for (uint32_t g = 0; g < nGnbs; ++g)
            {
                auto gnbMob = gnbNodes.Get(g)->GetObject<MobilityModel>();
                if (!gnbMob) continue;
                Vector gpos = gnbMob->GetPosition();
                double d = std::sqrt(std::pow(vpos.x-gpos.x,2)+std::pow(vpos.y-gpos.y,2));
                if (d < bestDist) { bestDist = d; nearest = g; }
            }
            if (senderSet.count(i)) gnbSenderCnt[nearest]++;
            else                    gnbBgCnt[nearest]++;
        }
        std::cout << "  Uu attachment (sender / bg per gNB):\n";
        for (uint32_t g = 0; g < nGnbs; ++g)
            std::cout << "    gNB[" << g << "]  sender=" << gnbSenderCnt[g]
                      << "  bg=" << gnbBgCnt[g]
                      << "  total=" << gnbSenderCnt[g] + gnbBgCnt[g] << "\n";
        std::cout << "  Uu BWP: shared (sender + bg compete for same PRBs, UL)\n";
    }

    // -------------------------------------------------------------------
    // P2P 1: PGW ↔ remoteHost  (Uu backhaul)
    // -------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu",      UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(3)));  // EPC/PGW→MEC server (ETSI MEC GS 003)
    NetDeviceContainer pgwRhDevs = p2ph.Install(pgw, remoteHost);

    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwRhIfaces = ipv4h.Assign(pgwRhDevs);
    const Ipv4Address remoteUuIp = pgwRhIfaces.GetAddress(1);  // 1.0.0.2

    // -------------------------------------------------------------------
    // P2P 2: RSU[r] ↔ remoteHost  (SL backhaul, one link per RSU)
    //   Subnets: 10.10.(2+r).0/30
    //     RSU[r] = 10.10.(2+r).1,  remoteHost = 10.10.(2+r).2
    //   remotePC5Ip = 10.10.2.2  (RSU0's side; all RSUs route there)
    // -------------------------------------------------------------------
    PointToPointHelper backhaulP2p;
    backhaulP2p.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    backhaulP2p.SetChannelAttribute("Delay",   StringValue("2ms"));  // RSU→MEC server local fiber (5GAA white paper)

    std::vector<NetDeviceContainer>      backhaulDevs(nRsus);
    std::vector<Ipv4InterfaceContainer>  backhaulIfaces(nRsus);
    std::vector<Ipv4Address>             rsuBackhaulIps(nRsus);

    for (uint32_t r = 0; r < nRsus; ++r)
    {
        backhaulDevs[r] = backhaulP2p.Install(rsuNodes.Get(r), remoteHost);
        std::ostringstream subnet;
        subnet << "10.10." << (2 + r) << ".0";
        ipv4h.SetBase(subnet.str().c_str(), "255.255.255.252");
        backhaulIfaces[r] = ipv4h.Assign(backhaulDevs[r]);
        rsuBackhaulIps[r] = backhaulIfaces[r].GetAddress(0);  // RSU side
    }
    // remotePC5Ip: remoteHost's address on RSU0's link (10.10.2.2)
    const Ipv4Address remotePC5Ip = backhaulIfaces[0].GetAddress(1);

    // -------------------------------------------------------------------
    // Routing
    // -------------------------------------------------------------------
    Ptr<Ipv4StaticRouting> rhRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());

    // remoteHost: 7.0.0.0/8 → via PGW (Uu ACKs to vehicle Uu IPs 7.0.0.x)
    // This MUST be added first so it serves as the default for Uu-addressed packets.
    rhRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"),
                               pgwRhIfaces.GetAddress(0), 1);
    // remoteHost: /32 host routes for each vehicle SL IP → via nearest RSU backhaul.
    // Nearest RSU is computed from each vehicle's initial position so ACKs travel the
    // shortest backhaul path and the vehicle is within SL range of that RSU.
    // Build a helper: vehNearestRsu[i] = index of RSU closest to vehicle i at t=0.
    std::vector<uint32_t> vehNearestRsu(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        Vector vp = vehicleNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        uint32_t best = 0;
        double   bestD = 1e18;
        for (uint32_t r = 0; r < nRsus; ++r)
        {
            double dx = rsuPosX[r] - vp.x;
            double dy = rsuPosY[r] - vp.y;
            double d  = std::sqrt(dx*dx + dy*dy);
            if (d < bestD) { bestD = d; best = r; }
        }
        vehNearestRsu[i] = best;
        // ACKs for vehicle Uu IPs route via network route 7.0.0.0/8 → PGW → Uu (no SL collision).
        // No per-vehicle SL-IP host routes on remoteHost needed.
    }

    // Each RSU: route to remotePC5Ip (10.10.2.2) via its direct backhaul link,
    // AND route back to vehicle SL IPs (7.0.0.0/8) via the RSU's own SL interface
    // so that ACKs forwarded from remoteHost can reach the vehicle.
    for (uint32_t r = 0; r < nRsus; ++r)
    {
        Ptr<Ipv4StaticRouting> rsuRoute =
            ipv4Routing.GetStaticRouting(rsuNodes.Get(r)->GetObject<Ipv4>());

        // Uplink (vehicle → remoteHost): forward to remotePC5Ip via P2P backhaul
        const uint32_t rsuBhIf =
            rsuNodes.Get(r)->GetObject<Ipv4>()->GetInterfaceForDevice(
                backhaulDevs[r].Get(0));
        rsuRoute->AddHostRouteTo(remotePC5Ip,
                                 backhaulIfaces[r].GetAddress(1),  // next-hop = remoteHost
                                 rsuBhIf);

        // Downlink (remoteHost ACK → vehicle): forward 7.0.0.0/8 via SL interface
        // The RSU's BIDIRECTIONAL SL TFT (7.0.0.0/255.0.0.0) matches and sends
        // the ACK as a groupcast (dstL2Id=255); the vehicle with matching IP accepts it.
        const uint32_t rsuSlIf =
            rsuNodes.Get(r)->GetObject<Ipv4>()->GetInterfaceForDevice(
                slNetDevs.Get(nVehicles + r));
        rsuRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), rsuSlIf);
    }

    // Vehicles: host route to remotePC5Ip via each vehicle's nearest RSU SL IP.
    // Using nearest RSU ensures the vehicle is within SL range of its gateway RSU.
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        const uint32_t nearRsu = vehNearestRsu[i];
        const uint32_t vehSlIf =
            vehicleNodes.Get(i)->GetObject<Ipv4>()
                               ->GetInterfaceForDevice(slNetDevs.Get(i));
        Ptr<Ipv4StaticRouting> vehRoute =
            ipv4Routing.GetStaticRouting(vehicleNodes.Get(i)->GetObject<Ipv4>());
        vehRoute->AddHostRouteTo(remotePC5Ip,
                                 rsuSlIps[nearRsu].GetAddress(0),
                                 vehSlIf);
        NS_LOG_INFO("veh[" << i << "] PC5 gateway = RSU[" << nearRsu << "]");
    }

    // -------------------------------------------------------------------
    // Dynamic PC5 route updaters (one per vehicle)
    // Check nearest RSU every 1 s; swap vehicle and remoteHost routes on transition.
    // This fixes the ACK path failure when the vehicle moves away from the initial
    // nearest RSU (e.g., RSU0 at t=0 → RSU2 at t=90 s for 22.22 m/s, 120 s sim).
    // -------------------------------------------------------------------
    std::vector<Pc5RouteUpdater> pc5RouteUpdaters(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        auto& ru              = pc5RouteUpdaters[i];
        ru.vehIdx             = i;
        ru.vehNode            = vehicleNodes.Get(i);
        ru.vehSlDev           = slNetDevs.Get(i);
        ru.rxPort             = rxPort;
        // initialised to remotePC5Ip (= 10.10.2.2) so the first Update() at t=0
        // correctly removes the initial static route before adding the per-RSU one.
        ru.currentRemotePC5Ip = remotePC5Ip;
        // UINT32_MAX ensures first Update() always overwrites the initial route.
        ru.currentRsu         = std::numeric_limits<uint32_t>::max();
        // pc5Path set later (after mpquicSockets are created)
        for (uint32_t r = 0; r < nRsus; ++r)
        {
            ru.rsuPos.push_back(Vector(rsuPosX[r], rsuPosY[r], rsuPosZ[r]));
            ru.rsuSlIps.push_back(rsuSlIps[r].GetAddress(0));
            // Per-RSU PC5 server IP: remoteHost's IP on RSU r's backhaul subnet.
            // Sending to this IP avoids cross-subnet WeakEsModel delivery issues.
            ru.rsuPC5Ips.push_back(backhaulIfaces[r].GetAddress(1));
        }
        // Fire at t=0: Ns2MobilityHelper events (scheduled earlier) have already
        // placed vehicles at their trace positions, so nearest-RSU is correct.
        Simulator::Schedule(Seconds(0.0), &Pc5RouteUpdater::Update, &ru);
    }

    // -------------------------------------------------------------------
    // NR SL resource pool
    // -------------------------------------------------------------------
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100);
    ptrFactory->SetSlSelectionWindow(20); // max CTTC-supported; t2min=40 slots; T2_attr=200 dominates
    ptrFactory->SetSlFreqResourcePscch(10);
    ptrFactory->SetSlSubchannelSize(slSubchSize);  // default: 25 RB → 2 subchannels; smaller = more subchannels
    ptrFactory->SetSlMaxNumPerReserve(3);
    // RRI=50ms: max SPS_wait=50ms; RTT_max=50+15(Uu-ACK)=65ms < 100ms target for all cases.
    // ~69 candidate resources for T2=49 → ~13% collision rate for 5 vehicles (acceptable).
    // (Was 100ms: max RTT=115ms, exceeded 100ms target for vehicles with worst-case SPS phase.)
    std::list<uint16_t> slReservePeriodList = {50};
    ptrFactory->SetSlResourceReservePeriodList(slReservePeriodList);
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
    // slHarq=true: HARQ process tracking (crashes on groupcast — ns-3-nr limitation)
    // slHarq=false + slMaxTxTransNumPssch=1: one TX per SPS slot → effective SPS period = RRI = 50ms.
    // With RRI=50ms this gives RTT_max = 50+9.5 = 59.5ms < 100ms target.
    // (Was 2: blind retx occupied two consecutive SPS slots → effective new-data period = 100ms,
    //  making RTT_max = 100+9.5 = 109.5ms which exceeded the 100ms target.)
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

    nrSlHelper->InstallNrSlPreConfiguration(slNetDevs, slPreCfg);
    stream += nrSlHelper->AssignStreams(slNetDevs, stream);

    // -------------------------------------------------------------------
    // NR SL bearer activation
    // -------------------------------------------------------------------
    Time slBearerActivation = appStart - MilliSeconds(10);

    SidelinkInfo slInfo;
    slInfo.m_castType    = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id     = 255;
    slInfo.m_dynamic     = true;
    slInfo.m_rri         = MilliSeconds(50);  // reduced from 100ms; see slReservePeriodList comment
    slInfo.m_pdb         = Seconds(0);
    slInfo.m_harqEnabled = slHarq;

    // Each vehicle: BIDIRECTIONAL TFT (TX data to remotePC5Ip, RX ACKs from any RSU backhaul)
    // Use a /16 mask (10.10.0.0/255.255.0.0) so ACKs arriving with source IP 10.10.r.2
    // (any RSU's backhaul interface) are accepted by the SL TFT filter.
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        Ptr<LteSlTft> vehTft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL,
                                                 Ipv4Address("10.10.0.0"),
                                                 Ipv4Mask("255.255.0.0"),
                                                 slInfo);
        nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                       NetDeviceContainer(slNetDevs.Get(i)),
                                       vehTft);
    }

    // Each RSU: RECEIVE-ONLY TFT matching the entire vehicle SL subnet (7.0.0.0/8).
    // ACKs now route via Uu (PGW→gNB→vehicle Uu), so RSUs never TX on SL.
    // RECEIVE-only prevents RSUs from claiming SPS TX slots, freeing pool capacity
    // for vehicle uplink and eliminating RSU↔vehicle SPS slot collisions.
    for (uint32_t r = 0; r < nRsus; ++r)
    {
        Ptr<LteSlTft> rsuTft = Create<LteSlTft>(
            LteSlTft::Direction::RECEIVE,
            Ipv4Address("7.0.0.0"),
            Ipv4Mask("255.0.0.0"),
            slInfo);
        nrSlHelper->ActivateNrSlBearer(slBearerActivation,
                                       NetDeviceContainer(slNetDevs.Get(nVehicles + r)),
                                       rsuTft);
    }

    // -------------------------------------------------------------------
    // MPQUIC
    // -------------------------------------------------------------------
    // MPQUIC scheduler selection:
    //   Redundant    → MpQuicRedundantScheduler (sends on both paths; weight ignored)
    //   MinRtt       → MpQuicMinRttScheduler
    //   RoundRobin   → MpQuicRoundRobinScheduler
    MpQuicHelper::SchedulerType sched = MpQuicHelper::SchedulerType::RoundRobin;
    if      (schedulerType == "Redundant")   sched = MpQuicHelper::SchedulerType::Redundant;
    else if (schedulerType == "MinRtt")      sched = MpQuicHelper::SchedulerType::MinRtt;
    else if (schedulerType == "RoundRobin")  sched = MpQuicHelper::SchedulerType::RoundRobin;

    // Only the first nSenders vehicles have MPQUIC + video sender app.
    // The remaining (nVehicles - nSenders) vehicles are passive: they still
    // occupy NR Uu/PC5 radio resources and move with the trace, but send
    // no video — matching the realistic case where only some vehicles do
    // remote driving while others are conventional.
    MpQuicHelper mpquicHelper;
    std::vector<Ptr<MpQuicSocketBase>> mpquicSockets(nSenders);
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        mpquicSockets[i] = mpquicHelper.InstallWithTwoPaths(
            vehicleNodes.Get(senderIndices[i]),
            InetSocketAddress(remoteUuIp,  rxPort),   // path 0: Uu
            InetSocketAddress(remotePC5Ip, rxPort),   // path 1: PC5
            sched,
            vehicleUuIps[senderIndices[i]].GetAddress(0));  // bind PC5 sock to Uu IP → ACKs via Uu
    }

    // Wire pc5Path pointers into pc5RouteUpdaters now that mpquicSockets are ready.
    // Pc5RouteUpdater::Update() at t=0 will call pc5Path->SetRemote() to redirect
    // each vehicle's PC5 path to its nearest RSU's backhaul IP.
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        const auto& paths = mpquicSockets[i]->GetPaths();
        if (paths.size() >= 2)
            pc5RouteUpdaters[senderIndices[i]].pc5Path = paths[1];  // path 1 = PC5
    }

    // Receiver apps on remoteHost — one per vehicle on port rxPort+i
    // (shared port rxPort for all; receiver differentiates by streamId = vehicle index)
    // For simplicity: single receiver on 0.0.0.0:rxPort
    auto rxApp = CreateObject<MpQuicReceiverApp>();
    rxApp->SetAttribute("Port", UintegerValue(rxPort));
    remoteHost->AddApplication(rxApp);
    rxApp->SetStartTime(appStart);
    rxApp->SetStopTime(simTime);

    // Per-frame CSV log — single file for all vehicles in this run
    std::string frameCsvPath = outputDir + "/" + simTag + "_frames.csv";
    std::ofstream frameLog(frameCsvPath, std::ofstream::out | std::ofstream::trunc);
    frameLog << "veh_id,frame_id,path_id,capture_ts_ms,server_rx_ts_ms,"
             << "deadline_ts_ms,arrived,late_flag,frame_size_bytes\n";
    rxApp->SetFrameLogger(&frameLog, 150.0);

    // Sender apps on the selected nSenders vehicles only
    std::vector<Ptr<MpQuicSenderApp>> senderApps(nSenders);
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        senderApps[i] = CreateObject<MpQuicSenderApp>();
        senderApps[i]->SetMpQuic(mpquicSockets[i]);
        senderApps[i]->SetPacketSize(pktSize);
        senderApps[i]->SetInterval(pktInterval);
        senderApps[i]->SetConnId(static_cast<uint32_t>(i));  // unique per-vehicle connection ID
        vehicleNodes.Get(senderIndices[i])->AddApplication(senderApps[i]);
        senderApps[i]->SetStartTime(appStart);
        senderApps[i]->SetStopTime(simTime);
    }

    // Register permanent-drop callback: when a sender exhausts kMaxRetries for a
    // reliable frame, it calls SkipStreamSeqNo on the receiver so the reorder buffer
    // advances past the hole and queued subsequent frames can drain on time.
    // This is the simulation equivalent of RFC 9000 RESET_STREAM.
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        mpquicSockets[i]->SetPermanentDropCallback(
            MakeCallback(&MpQuicReceiverApp::SkipStreamSeqNo, rxApp));
    }

    // Sender-side scheduler trace: per-frame path selection log.
    // CSV: time_ms, conn_id, stream_seq, sched_label,
    //      uu_active, uu_blocked, uu_cwnd, uu_pending, uu_rtt_ms,
    //      pc5_active, pc5_blocked, pc5_cwnd, pc5_pending, pc5_rtt_ms,
    //      tx_path, deferred, dropped
    std::string schedCsvPath = outputDir + "/" + simTag + "_sched.csv";
    auto schedLogFile = std::make_shared<std::ofstream>(schedCsvPath,
                            std::ofstream::out | std::ofstream::trunc);
    *schedLogFile << "time_ms,conn_id,stream_seq,sched_label,"
                  << "uu_active,uu_blocked,uu_cwnd,uu_pending,uu_rtt_ms,"
                  << "pc5_active,pc5_blocked,pc5_cwnd,pc5_pending,pc5_rtt_ms,"
                  << "tx_path,deferred,dropped\n";
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        mpquicSockets[i]->SetSchedLog(schedLogFile);
    }

    // Diagnostic per-packet logs (--diagLog=true only)
    std::shared_ptr<std::ofstream> pktLogFile;
    std::shared_ptr<std::ofstream> rxTraceLogFile;
    if (diagLog)
    {
        std::string pktCsvPath = outputDir + "/" + simTag + "_pkt_trace.csv";
        pktLogFile = std::make_shared<std::ofstream>(pktCsvPath,
                         std::ofstream::out | std::ofstream::trunc);
        *pktLogFile << "t_ms,conn_id,stream_id,stream_seq,event,path,cwnd,pending\n";
        for (uint32_t i = 0; i < nSenders; ++i)
            mpquicSockets[i]->SetPktLog(pktLogFile);

        std::string rxCsvPath = outputDir + "/" + simTag + "_rx_trace.csv";
        rxTraceLogFile = std::make_shared<std::ofstream>(rxCsvPath,
                             std::ofstream::out | std::ofstream::trunc);
        *rxTraceLogFile << "t_ms,conn_id,stream_id,stream_seq,event,path_id,buf_size,next_expected,hol_ms\n";
        rxApp->SetRxTraceLog(rxTraceLogFile);
    }

    // -------------------------------------------------------------------
    // Trace hooks: Uu RSRP/SINR, handover per vehicle
    // -------------------------------------------------------------------
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        auto nrUeDev = DynamicCast<NrUeNetDevice>(vehicleNrDevs.Get(i));
        auto uePhy   = nrUeDev->GetPhy(0);

        bool ok1 = uePhy->TraceConnectWithoutContext(
            "ReportRsrp",
            MakeBoundCallback(&CbRsrp, i));
        bool ok2 = uePhy->TraceConnectWithoutContext(
            "DlDataSinr",
            MakeBoundCallback(&CbSinr, i));
        NS_LOG_INFO("Veh[" << i << "] trace connect: ReportRsrp=" << ok1
                    << " DlDataSinr=" << ok2);

        // Handover traces on NR UE RRC (via direct pointer, not Config path)
        Ptr<LteUeRrc> ueRrc = nrUeDev->GetRrc();
        if (ueRrc)
        {
            ueRrc->TraceConnectWithoutContext(
                "HandoverStart",
                MakeBoundCallback(&CbHandoverStart, i));
            ueRrc->TraceConnectWithoutContext(
                "HandoverEndOk",
                MakeBoundCallback(&CbHandoverEndOk, i));
        }
    }

    // -------------------------------------------------------------------
    // Per-vehicle CSV loggers
    // -------------------------------------------------------------------
    const Time snapshotInterval = MilliSeconds(100);

    std::vector<VehicleLogger> loggers(nSenders);
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        loggers[i].vehIdx     = senderIndices[i];
        loggers[i].connId     = i;
        loggers[i].mpquic     = mpquicSockets[i];
        loggers[i].rxApp      = rxApp;
        loggers[i].senderApp  = senderApps[i];
        loggers[i].vehMob     = vehicleNodes.Get(senderIndices[i])->GetObject<MobilityModel>();
        for (uint32_t r = 0; r < nRsus; ++r)
            loggers[i].rsuMobs.push_back(rsuNodes.Get(r)->GetObject<MobilityModel>());

        std::string csvPath = outputDir + "/" + simTag + "_veh" +
                              std::to_string(i) + "_telemetry.csv";
        loggers[i].csv.open(csvPath, std::ofstream::out | std::ofstream::trunc);
        loggers[i].csv
            << "time_s,veh_id,veh_x,veh_y,"
            << "uu_rsrp_dBm,uu_sinr_dB,uu_rtt_ms,uu_cwnd,uu_tx_pkts,uu_retx,"
            << "ho_flag,serving_gnb,"
            << "pc5_rtt_ms,pc5_cwnd,pc5_tx_pkts,pc5_retx,pc5_pending,"
            << "nearest_rsu,dist_rsu_m,angle_rsu_deg,"
            << "rx_pkts,delivery_pct,gamma,queue_depth\n";

        Simulator::Schedule(snapshotInterval, &TakeSnapshot,
                            &loggers[i], snapshotInterval, simTime);
    }

    // -------------------------------------------------------------------
    // Per-vehicle Epoch CSV loggers
    // -------------------------------------------------------------------
    const Time epochLen = MilliSeconds(epochLenMs);
    std::vector<EpochLogger> epochLoggers(nSenders);
    for (uint32_t i = 0; i < nSenders; ++i)
    {
        epochLoggers[i].vehIdx    = senderIndices[i];
        epochLoggers[i].connId    = i;                 // matches SetConnId(i) in senderApps
        epochLoggers[i].mpquic    = mpquicSockets[i];
        epochLoggers[i].rxApp     = rxApp;
        epochLoggers[i].senderApp = senderApps[i];
        epochLoggers[i].vehMob    = vehicleNodes.Get(senderIndices[i])->GetObject<MobilityModel>();
        epochLoggers[i].nRsuPos   = nRsus;
        for (uint32_t r = 0; r < nRsus; ++r)
        {
            epochLoggers[i].rsuPX[r] = rsuPosX[r];
            epochLoggers[i].rsuPY[r] = rsuPosY[r];
        }
        epochLoggers[i].nGnbPos = nGnbs;
        for (uint32_t g = 0; g < nGnbs && g < 9; ++g)
        {
            epochLoggers[i].gnbPX[g] = gnbPosX[g];
            epochLoggers[i].gnbPY[g] = gnbPosY[g];
        }

        std::string ePath = outputDir + "/" + simTag + "_veh" +
                            std::to_string(i) + "_epoch.csv";
        epochLoggers[i].csv.open(ePath, std::ofstream::out | std::ofstream::trunc);
        epochLoggers[i].csv
            << "epoch_idx,veh_id,epoch_end_s,"
            << "uu_completed_frames,uu_timeout_frames,uu_deadline_met_frames,"
            << "pc5_video_completed_frames,pc5_video_timeout_frames,pc5_video_deadline_met_frames,"
            << "uu_avg_delay_ms,pc5_video_avg_delay_ms,"
            << "e2e_deadline_met_frac,freeze_flag_e2e,"
            << "uu_rtt_ms,pc5_rtt_ms,"
            << "cam_tx_epoch,bsm_tx_epoch,"
            << "uu_p95_delay_ms,pc5_p95_delay_ms,"
            << "uu_jitter_ms,pc5_jitter_ms,"
            << "missing_frame_burst_len,"
            << "veh_x,veh_y,speed_mps,heading_deg,"
            << "dist_nearest_rsu_m,"
            << "dist_serving_gnb_m,handover_zone_flag,"
            << "nearby_vehicle_count,gps_capture_ts_ms,gps_server_rx_ts_ms,"
            << "uu_rx_frames,pc5_rx_frames,"
            << "tx_unique_frames\n";

        Simulator::Schedule(epochLen, &TakeEpochSnap,
                            &epochLoggers[i], epochLen, simTime);
    }

    // -------------------------------------------------------------------
    // [DIAGNOSTIC] PC5 hard-disable at configurable time relative to appStart
    //   disablePc5At < 0  → feature off (default)
    //   disablePc5At = -0.001 → disable just before first app payload (Run A/B)
    //   disablePc5At = 10.0   → disable 10 s after appStart (Run D: backlog persistence)
    // -------------------------------------------------------------------
    if (disablePc5At > -1.0 + 1e-9)  // sentinel = -1.0 (default off); strictly > -1.0 triggers
    {
        Time disableAt = appStart + Seconds(disablePc5At);
        // Clamp: must not be before t=0
        if (disableAt < Seconds(0)) disableAt = Seconds(0);

        for (uint32_t i = 0; i < nSenders; ++i)
        {
            Ptr<MpQuicSocketBase> sock = mpquicSockets[i];
            uint32_t              vIdx = senderIndices[i];
            Simulator::Schedule(disableAt, [sock, vIdx, disableAt]()
            {
                double nowMs = Simulator::Now().GetMilliSeconds();
                auto&  paths = sock->GetPaths();
                NS_LOG_UNCOND("[DIAG_PC5_DISABLE] veh=" << vIdx
                    << "  t=" << nowMs << " ms"
                    << "  nPaths=" << paths.size()
                    << "  scheduled_at=" << disableAt.GetMilliSeconds() << " ms");
                uint32_t nDisabled = 0;
                for (auto& p : paths)
                {
                    NS_LOG_UNCOND("[DIAG_PC5_DISABLE]   pathId=" << (int)p->GetPathId()
                        << "  type=" << (p->GetPathType() == MpQuicPathType::UU ? "UU" : "PC5")
                        << "  active=" << p->IsActive());
                    if (p->GetPathType() == MpQuicPathType::PC5)
                    {
                        sock->SetPathActive(p->GetPathId(), false);
                        ++nDisabled;
                        NS_LOG_UNCOND("[DIAG_PC5_DISABLE]   → DISABLED pathId=" << (int)p->GetPathId());
                    }
                }
                if (nDisabled == 0)
                {
                    NS_LOG_UNCOND("[DIAG_PC5_DISABLE]   WARNING: no PC5 path found at disable time!"
                        << " GetPaths().size()=" << paths.size()
                        << " — path may not be established yet.");
                }
            });
        }
        NS_LOG_UNCOND("[DIAG_PC5_DISABLE] scheduled for " << nSenders
            << " sender(s) at t=" << disableAt.GetMilliSeconds() << " ms"
            << "  (appStart=" << appStart.GetMilliSeconds() << " ms"
            << ", offset=" << disablePc5At << " s)");
    }

    // -------------------------------------------------------------------
    // CAM background broadcast — all nVehicles (V2X SL interference model)
    //   CAM: 10 Hz, 300 B  (ETSI EN 302 637-2)  — BSM disabled (CAM-only mode)
    // Each app binds to its vehicle's SL IP and sends to 255.255.255.255
    // on the SL interface, creating interference with video PC5 traffic.
    // -------------------------------------------------------------------
    std::vector<Ptr<NrSlCamBsmApp>> camBsmApps(nVehicles);
    for (uint32_t i = 0; i < nVehicles; ++i)
    {
        camBsmApps[i] = CreateObject<NrSlCamBsmApp>();
        Ipv4Address slIp = vehicleSlIps[i].GetAddress(0);
        camBsmApps[i]->SetSlAddress(slIp);
        vehicleNodes.Get(i)->AddApplication(camBsmApps[i]);
        camBsmApps[i]->SetStartTime(appStart);
        camBsmApps[i]->SetStopTime(simTime);
    }

    // Wire CAM/BSM app into epoch loggers for sender vehicles
    for (uint32_t i = 0; i < nSenders; ++i)
        epochLoggers[i].camBsmApp = camBsmApps[senderIndices[i]];

    NS_LOG_INFO("[CAM] installed on " << nVehicles << " vehicles"
                << "  CAM=300B@10Hz  SL broadcast (BSM disabled)");

    // -------------------------------------------------------------------
    // Background Uu uplink load — non-sender vehicles (nSenders..nVehicles-1)
    //
    // All non-sender vehicles already have NR Uu devices, Uu IP addresses,
    // and a default route via the EPC gateway.  We add uplink UDP apps to
    // create realistic gNB scheduler contention for managed vehicle traffic.
    //
    // Three traffic classes (user-configurable split):
    //   Class 1 (bgUuClass1Pct %): persistent low-rate     64 kbps  100 B @ 12 ms
    //   Class 2 (bgUuClass2Pct %): bursty medium-rate      512 kbps peak, Exp on/off
    //   Class 3 (remainder)      : quasi-persistent high    2 Mbps   1024 B @ 4 ms
    // -------------------------------------------------------------------
    if (bgUuEnabled && nSenders < nVehicles)
    {
        uint32_t nBg = static_cast<uint32_t>(bgIndices.size());
        // clamp percentages so they sum to ≤ 100
        uint32_t c1pct = std::min(bgUuClass1Pct, 100u);
        uint32_t c2pct = std::min(bgUuClass2Pct, 100u - c1pct);
        uint32_t nC1   = std::max(1u, nBg * c1pct / 100);
        uint32_t nC2   = std::max(0u, nBg * c2pct / 100);
        // Class 3: remainder (may be 0 if c1+c2 covers all)

        uint16_t bgPort = rxPort + 1000;  // separate from MPQUIC rx port

        // Single PacketSink on remoteHost receives all background Uu traffic
        PacketSinkHelper bgSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), bgPort));
        ApplicationContainer bgSinkApp = bgSink.Install(remoteHost);
        bgSinkApp.Start(appStart);
        bgSinkApp.Stop(simTime);

        for (uint32_t bgIdx = 0; bgIdx < nBg; ++bgIdx)
        {
            uint32_t nodeIdx = bgIndices[bgIdx];
            // Stagger start times by 20 ms per vehicle to avoid burst synchronisation
            Time bgStart = appStart + MilliSeconds(bgIdx * 20);

            if (bgIdx < nC1)
            {
                // Class 1: persistent low-rate  (64 kbps ≈ 100 B every 12.5 ms)
                UdpClientHelper c1h(remoteUuIp, bgPort);
                c1h.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
                c1h.SetAttribute("Interval",   TimeValue(MilliSeconds(12)));
                c1h.SetAttribute("PacketSize",  UintegerValue(100));
                ApplicationContainer c1app = c1h.Install(vehicleNodes.Get(nodeIdx));
                c1app.Start(bgStart);
                c1app.Stop(simTime);
            }
            else if (bgIdx < nC1 + nC2)
            {
                // Class 2: bursty medium-rate  (OnOff, 512 kbps on, Exp[500ms] on/off)
                OnOffHelper c2h("ns3::UdpSocketFactory",
                                InetSocketAddress(remoteUuIp, bgPort));
                c2h.SetAttribute("DataRate",   StringValue("512kbps"));
                c2h.SetAttribute("PacketSize", UintegerValue(512));
                c2h.SetAttribute("OnTime",
                    StringValue("ns3::ExponentialRandomVariable[Mean=0.5]"));
                c2h.SetAttribute("OffTime",
                    StringValue("ns3::ExponentialRandomVariable[Mean=0.5]"));
                ApplicationContainer c2app = c2h.Install(vehicleNodes.Get(nodeIdx));
                c2app.Start(bgStart);
                c2app.Stop(simTime);
            }
            else
            {
                // Class 3: quasi-persistent high-rate  (2 Mbps ≈ 1024 B every 4 ms)
                UdpClientHelper c3h(remoteUuIp, bgPort);
                c3h.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
                c3h.SetAttribute("Interval",   TimeValue(MilliSeconds(4)));
                c3h.SetAttribute("PacketSize",  UintegerValue(1024));
                ApplicationContainer c3app = c3h.Install(vehicleNodes.Get(nodeIdx));
                c3app.Start(bgStart);
                c3app.Stop(simTime);
            }
        }

        uint32_t nC3 = (nBg > nC1 + nC2) ? nBg - nC1 - nC2 : 0;
        std::cout << "  BgUu load:    enabled  bgVehicles=" << nBg
                  << "  direction=UL-only\n"
                  << "    C1(64kbps  persistent)=" << nC1
                  << "  C2(512kbps bursty)=" << nC2
                  << "  C3(2Mbps persistent)=" << nC3 << "\n";
    }
    else if (!bgUuEnabled)
    {
        std::cout << "  BgUu load:    disabled (--bgUuEnabled to enable)\n";
    }

    // -------------------------------------------------------------------
    // Info printout
    // -------------------------------------------------------------------
    std::cout << "\n=== NR Uu + NR SL PC5 + MPQUIC (" << topoMode << " mode) ===\n"
              << "  Vehicles:     " << nVehicles
              << (traceFile.empty() ? std::string("  (v=") + std::to_string(vehicleSpeed) + "m/s, ConstantVelocity)" : "  (SUMO trace)")
              << "  Senders: " << nSenders << "\n"
              << "  gNBs:         " << nGnbs << " (25 m height)\n";
    for (uint32_t j = 0; j < nGnbs; ++j)
        std::cout << "    gNB[" << j << "]: (" << gnbPosX[j] << "," << gnbPosY[j] << ")\n";
    std::cout << "  RSUs:         " << nRsus << " (6m height)\n";
    const char* rsuNames[] = {"RSU-A","RSU-B","RSU-C","RSU-D","RSU-E","RSU-F","RSU-G","RSU-H","RSU-I"};
    for (uint32_t r = 0; r < nRsus; ++r)
        std::cout << "    RSU[" << r << "]: (" << rsuPosX[r] << "," << rsuPosY[r]
                  << ")  " << rsuNames[r]
                  << "  backhaul=" << rsuBackhaulIps[r] << "\n";
    std::cout << "  Scheduler:    " << schedulerType << "\n"
              << "  NR SL HARQ:   " << (slHarq ? "on" : "off") << "\n"
              << "  Pkt interval: " << pktInterval.GetMilliSeconds() << " ms\n"
              << "  Sim time:     " << simTime.GetSeconds() << " s\n"
              << "  Output dir:   " << outputDir << "/\n\n"
              << "  remoteHost Uu:  " << remoteUuIp << ":" << rxPort << "\n"
              << "  remoteHost PC5: " << remotePC5Ip << ":" << rxPort << "\n\n";
    for (uint32_t i = 0; i < nVehicles; ++i)
        std::cout << "  Veh[" << i << "]  Uu=" << vehicleUuIps[i].GetAddress(0)
                  << "  SL=" << vehicleSlIps[i].GetAddress(0) << "\n";
    for (uint32_t r = 0; r < nRsus; ++r)
        std::cout << "  RSU[" << r << "] SL=" << rsuSlIps[r].GetAddress(0) << "\n";
    std::cout << "\n";

    // -------------------------------------------------------------------
    // Run  (periodic progress report every 1 simulated second)
    // -------------------------------------------------------------------
    // Schedule progress ticks
    std::function<void(double)> scheduleTick;
    // Use a plain function via lambda stored in a Ptr wrapper
    // (ns-3 MakeEvent requires a function pointer or Callback; use a static helper)
    struct ProgressTick {
        static void Print(double nextSec, Time stop) {
            std::cout << "[sim] t=" << std::fixed << std::setprecision(1)
                      << Simulator::Now().GetSeconds() << "s\n" << std::flush;
            if (Simulator::Now() + Seconds(1.0) <= stop)
                Simulator::Schedule(Seconds(1.0), &ProgressTick::Print,
                                    nextSec + 1.0, stop);
        }
    };
    Simulator::Schedule(Seconds(1.0), &ProgressTick::Print, 1.0, simTime);

    Simulator::Stop(simTime);
    Simulator::Run();

    // -------------------------------------------------------------------
    // Final summary – scheduler performance comparison
    // -------------------------------------------------------------------
    uint64_t totalRxAll = rxApp->GetTotalRxPackets();

    // Γ: average across all vehicle streams that received at least one packet
    {
        // computed below after per-vehicle loop
    }

    // Collect per-vehicle stats
    struct VehStats {
        uint64_t appTx{0};              // unique packets pushed by the app (no retx)
        uint64_t uuTx{0}, uuRetx{0};
        uint64_t slTx{0}, slRetx{0};
        double   uuRtt{0.0}, slRtt{0.0};
        uint64_t rxPkts{0};             // packets received for this vehicle's stream
        double   gamma{0.0};            // Γ for this vehicle's stream
    };
    std::vector<VehStats> vs(nSenders);

    uint64_t sumAppTx = 0;
    uint64_t sumUuTx = 0, sumUuRetx = 0;
    uint64_t sumSlTx = 0, sumSlRetx = 0;
    double   sumUuRtt = 0.0, sumSlRtt = 0.0;
    uint32_t cntUuRtt = 0,   cntSlRtt = 0;
    double   gammaSum = 0.0;
    uint32_t gammaCount = 0;

    for (uint32_t i = 0; i < nSenders; ++i)
    {
        vs[i].appTx  = senderApps[i]->GetTotalPacketsSent();
        vs[i].rxPkts = rxApp->GetConnRxPackets(i);
        vs[i].gamma  = rxApp->GetContinuity(i);
        sumAppTx += vs[i].appTx;
        gammaSum += vs[i].gamma;
        ++gammaCount;

        for (const auto& p : mpquicSockets[i]->GetPaths())
        {
            if (p->GetPathType() == MpQuicPathType::UU)
            {
                vs[i].uuTx   = p->GetTxPackets();
                vs[i].uuRetx = p->GetRetxPackets();
                vs[i].uuRtt  = p->GetRtt().GetMilliSeconds();
                if (p->IsRttInitialized()) { sumUuRtt += vs[i].uuRtt; ++cntUuRtt; }
            }
            else
            {
                vs[i].slTx   = p->GetTxPackets();
                vs[i].slRetx = p->GetRetxPackets();
                vs[i].slRtt  = p->GetRtt().GetMilliSeconds();
                // Only include paths that actually received an RTT sample (exclude initial 50ms default)
                if (p->IsRttInitialized()) { sumSlRtt += vs[i].slRtt; ++cntSlRtt; }
            }
        }
        sumUuTx   += vs[i].uuTx;   sumUuRetx += vs[i].uuRetx;
        sumSlTx   += vs[i].slTx;   sumSlRetx += vs[i].slRetx;
    }

    double gammaAll = (gammaCount > 0) ? gammaSum / gammaCount : 0.0;

    // Delivery rate: Rx / unique app-generated Tx (retransmissions excluded from denominator)
    uint64_t totalTx    = sumUuTx + sumSlTx;   // total on-wire (includes retx, for reference)
    uint64_t totalRetx  = sumUuRetx + sumSlRetx;
    double   delivPct   = (sumAppTx > 0)
                          ? 100.0 * static_cast<double>(totalRxAll) / static_cast<double>(sumAppTx)
                          : 0.0;
    double   retxRate   = (totalTx > 0)
                          ? 100.0 * static_cast<double>(totalRetx) / static_cast<double>(totalTx)
                          : 0.0;
    double   uuShare    = (totalTx > 0)
                          ? 100.0 * static_cast<double>(sumUuTx) / static_cast<double>(totalTx)
                          : 0.0;
    double   avgUuRtt   = (cntUuRtt > 0) ? sumUuRtt / cntUuRtt : 0.0;
    double   avgSlRtt   = (cntSlRtt > 0) ? sumSlRtt / cntSlRtt : 0.0;

    // --- header ---
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          MPQUIC Scheduler Performance Summary               ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    std::cout << "  Scheduler : " << schedulerType << "\n"
              << "  Vehicles  : " << nVehicles << "   simTime: "
              << simTime.GetSeconds() << " s\n\n";

    // --- per-vehicle table ---
    std::cout << std::left
              << std::setw(5)  << "Veh"
              << std::setw(8)  << "App-TX"
              << std::setw(8)  << "Rx"
              << std::setw(7)  << "PDR%"
              << std::setw(9)  << "Uu-TX"
              << std::setw(8)  << "Uu-retx"
              << std::setw(11) << "Uu-RTT(ms)"
              << std::setw(9)  << "PC5-TX"
              << std::setw(8)  << "PC5-retx"
              << std::setw(12) << "PC5-RTT(ms)"
              << "\n";
    std::cout << std::string(90, '-') << "\n";

    for (uint32_t i = 0; i < nSenders; ++i)
    {
        double vPdr = (vs[i].appTx > 0)
                      ? 100.0 * static_cast<double>(vs[i].rxPkts) / static_cast<double>(vs[i].appTx)
                      : 0.0;
        // Uu-rx and PC5-rx: approximate split by path share of retx-corrected TX
        // (exact per-path Rx not tracked separately; use total stream Rx)
        std::cout << std::left
                  << std::setw(5)  << i
                  << std::setw(8)  << vs[i].appTx
                  << std::setw(8)  << vs[i].rxPkts
                  << std::setw(7)  << std::fixed << std::setprecision(1) << vPdr
                  << std::setw(9)  << vs[i].uuTx
                  << std::setw(8)  << vs[i].uuRetx
                  << std::setw(11) << std::fixed << std::setprecision(1) << vs[i].uuRtt
                  << std::setw(9)  << vs[i].slTx
                  << std::setw(8)  << vs[i].slRetx
                  << std::setw(12) << std::fixed << std::setprecision(1) << vs[i].slRtt
                  << "\n";
    }

    // --- totals row ---
    std::cout << std::string(90, '-') << "\n";
    std::cout << std::left
              << std::setw(5)  << "ALL"
              << std::setw(8)  << sumAppTx
              << std::setw(8)  << totalRxAll
              << std::setw(7)  << std::fixed << std::setprecision(1) << delivPct
              << std::setw(9)  << sumUuTx
              << std::setw(8)  << sumUuRetx
              << std::setw(11) << std::fixed << std::setprecision(1) << avgUuRtt
              << std::setw(9)  << sumSlTx
              << std::setw(8)  << sumSlRetx
              << std::setw(12) << std::fixed << std::setprecision(1) << avgSlRtt
              << "\n\n";

    // --- key metrics ---
    std::cout << "┌─────────────────────────────────────────┐\n";
    std::cout << "│ Key Metrics                             │\n";
    std::cout << "├─────────────────────────────────────────┤\n";
    std::cout << "│ App-gen Tx    : " << std::setw(8) << sumAppTx
              << " pkts (no retx)    │\n";
    std::cout << "│ Wire Tx(+retx): " << std::setw(8) << totalTx
              << " pkts              │\n";
    std::cout << "│ Total Rx      : " << std::setw(8) << totalRxAll
              << " pkts              │\n";
    std::cout << "│ PDR (Rx/AppTx): " << std::setw(7) << std::fixed << std::setprecision(2)
              << delivPct << " %                │\n";
    std::cout << "│ Retx rate     : " << std::setw(7) << std::fixed << std::setprecision(2)
              << retxRate << " %                │\n";
    std::cout << "│ Uu  avg RTT   : " << std::setw(7) << std::fixed << std::setprecision(1)
              << avgUuRtt << " ms               │\n";
    std::cout << "│ PC5 avg RTT   : " << std::setw(7) << std::fixed << std::setprecision(1)
              << avgSlRtt << " ms               │\n";
    std::cout << "│ Uu path share : " << std::setw(7) << std::fixed << std::setprecision(1)
              << uuShare << " %                │\n";
    std::cout << "│ Γ (continuity): " << std::setw(7) << std::fixed << std::setprecision(4)
              << gammaAll << "                  │\n";
    std::cout << "└─────────────────────────────────────────┘\n\n";

    Simulator::Destroy();
    return 0;
}
