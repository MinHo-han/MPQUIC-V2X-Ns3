// SPDX-License-Identifier: GPL-2.0-only
#include "server-sink-app.h"

#include "v2x-headers.h"

#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/string.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <filesystem>
#include <iomanip>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ServerSinkApp");
NS_OBJECT_ENSURE_REGISTERED(ServerSinkApp);

TypeId
ServerSinkApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ServerSinkApp")
            .SetParent<Application>()
            .SetGroupName("V2xRemoteSupport")
            .AddConstructor<ServerSinkApp>()
            .AddAttribute("VideoPort",
                          "UDP port for video chunk datagrams",
                          UintegerValue(5000),
                          MakeUintegerAccessor(&ServerSinkApp::m_videoPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("MetaPort",
                          "UDP port for metadata datagrams",
                          UintegerValue(5001),
                          MakeUintegerAccessor(&ServerSinkApp::m_metaPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("CoopPort",
                          "UDP port for cooperative traffic datagrams",
                          UintegerValue(5002),
                          MakeUintegerAccessor(&ServerSinkApp::m_coopPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("OutputDir",
                          "Root output directory; per-vehicle logs go in OutputDir/vehicles/",
                          StringValue("."),
                          MakeStringAccessor(&ServerSinkApp::m_outputDir),
                          MakeStringChecker())
            .AddAttribute("FrameDeadline",
                          "Playout deadline: chunks arriving after sendTime+deadline are discarded (seconds)",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&ServerSinkApp::m_frameDeadlineSec),
                          MakeDoubleChecker<double>(0.001, 60.0))
            .AddAttribute("IFrameThreshold",
                          "Min useful-chunk ratio for I-frame acceptance [0,1]",
                          DoubleValue(0.90),
                          MakeDoubleAccessor(&ServerSinkApp::m_iFrameThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("PFrameThreshold",
                          "Min useful-chunk ratio for P-frame acceptance [0,1]",
                          DoubleValue(0.70),
                          MakeDoubleAccessor(&ServerSinkApp::m_pFrameThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EpochInterval",
                          "TiL decision epoch width T_d (seconds)",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&ServerSinkApp::m_epochIntervalSec),
                          MakeDoubleChecker<double>(0.01, 60.0));
    return tid;
}

ServerSinkApp::ServerSinkApp()
{
}

ServerSinkApp::~ServerSinkApp()
{
}

// ---------------------------------------------------------------------------
// Public API: infrastructure positions + handover hook
// ---------------------------------------------------------------------------
void
ServerSinkApp::SetInfrastructurePositions(const std::vector<std::pair<double, double>>& gnbXY,
                                           const std::vector<std::pair<double, double>>& rsuXY)
{
    m_gnbXY = gnbXY;
    m_rsuXY = rsuXY;
}

void
ServerSinkApp::NotifyHandover(uint32_t vehicleId)
{
    m_hoStates[vehicleId].lastHandoverTimeSec = Simulator::Now().GetSeconds();
    NS_LOG_INFO("ServerSinkApp: HO veh=" << vehicleId
                                         << " t=" << Simulator::Now().GetSeconds());
}

// ---------------------------------------------------------------------------
// Infrastructure distance helpers
// ---------------------------------------------------------------------------
double
ServerSinkApp::CalcDistToNearestGnb(double x, double y) const
{
    if (m_gnbXY.empty())
        return -1.0;
    double best = 1e18;
    for (const auto& [gx, gy] : m_gnbXY)
    {
        const double d = std::sqrt((x - gx) * (x - gx) + (y - gy) * (y - gy));
        if (d < best)
            best = d;
    }
    return best;
}

double
ServerSinkApp::CalcDistToNearestRsu(double x, double y) const
{
    if (m_rsuXY.empty())
        return -1.0;
    double best = 1e18;
    for (const auto& [rx, ry] : m_rsuXY)
    {
        const double d = std::sqrt((x - rx) * (x - rx) + (y - ry) * (y - ry));
        if (d < best)
            best = d;
    }
    return best;
}

// ---------------------------------------------------------------------------
// Per-vehicle file accessors — open with CSV header on first use
// ---------------------------------------------------------------------------
std::ofstream&
ServerSinkApp::VideoRawFile(uint32_t vehicleId)
{
    auto& f = m_videoRawFiles[vehicleId];
    if (!f.is_open())
    {
        f.open(m_vehiclesDir + "/veh_" + std::to_string(vehicleId) + "_video_raw.csv");
        f << "recv_time_s,frame_id,chunk_id,total_chunks,frame_type,"
             "payload_size,send_time_s,latency_ms,path\n";
    }
    return f;
}

std::ofstream&
ServerSinkApp::FramesFile(uint32_t vehicleId)
{
    auto& f = m_framesFiles[vehicleId];
    if (!f.is_open())
    {
        f.open(m_vehiclesDir + "/veh_" + std::to_string(vehicleId) + "_frames.csv");
        f << "frame_id,frame_type,total_chunks,recv_chunks,useful_chunks,"
             "send_time_s,first_arrival_s,last_arrival_s,"
             "delivery_latency_ms,deadline_met,path\n";
    }
    return f;
}

std::ofstream&
ServerSinkApp::MetaFile(uint32_t vehicleId)
{
    auto& f = m_metaFiles[vehicleId];
    if (!f.is_open())
    {
        f.open(m_vehiclesDir + "/veh_" + std::to_string(vehicleId) + "_meta.csv");
        f << "recv_time_s,frame_id,lat,lon,heading_deg,speed_ms,"
             "cpu_load,mem_usage_mb,send_time_s,latency_ms\n";
    }
    return f;
}

std::ofstream&
ServerSinkApp::CoopFile(uint32_t vehicleId)
{
    auto& f = m_coopFiles[vehicleId];
    if (!f.is_open())
    {
        f.open(m_vehiclesDir + "/veh_" + std::to_string(vehicleId) + "_coop.csv");
        f << "recv_time_s,app_type,lat,lon,speed_ms,heading_deg,accel_ms2,"
             "hazard_type,severity,send_time_s,latency_ms\n";
    }
    return f;
}

std::ofstream&
ServerSinkApp::EpochFile(uint32_t vehicleId)
{
    auto& f = m_epochFiles[vehicleId];
    if (!f.is_open())
    {
        f.open(m_vehiclesDir + "/veh_" + std::to_string(vehicleId) + "_epoch.csv");
        // Column names match the spec in server-sink-app.h exactly.
        f << "epoch_idx,"
             "epoch_start_ms,"
             "epoch_end_ms,"
             // y_recv^Uu
             "uu_completed_frames,"
             "uu_timeout_frames,"
             "uu_deadline_met_frames,"
             "uu_avg_delay_ms,"
             "uu_max_delay_ms,"
             // y_recv^PC5 video
             "pc5_video_completed_frames,"
             "pc5_video_timeout_frames,"
             "pc5_video_deadline_met_frames,"
             "pc5_video_avg_delay_ms,"
             "pc5_video_max_delay_ms,"
             // y_recv^coop
             "coop_rx_packets,"
             "coop_avg_delay_ms,"
             "coop_max_delay_ms,"
             // y_svc
             "freeze_flag_e2e,"
             "consecutive_freeze_epochs,"
             // r_veh
             "last_rx_x,"
             "last_rx_y,"
             "last_rx_speed,"
             "last_rx_heading,"
             "last_rx_acceleration,"
             "age_of_last_vehicle_state_ms,"
             // m_prior
             "dist_to_nearest_gnb,"
             "dist_to_nearest_rsu,"
             "recent_handover_flag,"
             "time_since_last_handover_ms,"
             // v_scene
             "num_i_frames,"
             "num_p_frames\n";
    }
    return f;
}

// ---------------------------------------------------------------------------
// WriteEpochSummaries — flush one row per video vehicle, then reschedule
// ---------------------------------------------------------------------------
void
ServerSinkApp::WriteEpochSummaries()
{
    const double nowSec = Simulator::Now().GetSeconds();
    const double nowMs  = nowSec * 1000.0;
    ++m_epochCount;

    for (auto& [vehicleId, es] : m_epochStates)
    {
        // Only write epoch rows for vehicles that send video
        if (!es.isVideoVehicle)
        {
            ++es.epochIdx;
            es.epochStartSec = nowSec;
            // reset per-epoch counters even for non-video vehicles
            es.uuCompletedFrames         = 0;
            es.uuTimeoutFrames           = 0;
            es.uuDeadlineMetFrames       = 0;
            es.uuLatencySum              = 0.0;
            es.uuLatencyMax              = -1.0;
            es.pc5VideoCompletedFrames   = 0;
            es.pc5VideoTimeoutFrames     = 0;
            es.pc5VideoDeadlineMetFrames = 0;
            es.pc5VideoLatencySum        = 0.0;
            es.pc5VideoLatencyMax        = -1.0;
            es.coopRxPackets             = 0;
            es.coopLatencySum            = 0.0;
            es.coopLatencyMax            = -1.0;
            es.numIFrames                = 0;
            es.numPFrames                = 0;
            continue;
        }

        // ── col 2-3: epoch timing ──────────────────────────────────────────
        const double epochStartMs = es.epochStartSec * 1000.0;
        const double epochEndMs   = nowMs;

        // ── col 7: uu_avg_delay_ms ────────────────────────────────────────
        const double uuAvgDelay =
            (es.uuCompletedFrames > 0) ? (es.uuLatencySum / es.uuCompletedFrames) : -1.0;

        // ── col 12: pc5_video_avg_delay_ms ───────────────────────────────
        const double pc5VideoAvgDelay =
            (es.pc5VideoCompletedFrames > 0)
                ? (es.pc5VideoLatencySum / es.pc5VideoCompletedFrames)
                : -1.0;

        // ── col 15: coop_avg_delay_ms ─────────────────────────────────────
        const double coopAvgDelay =
            (es.coopRxPackets > 0) ? (es.coopLatencySum / es.coopRxPackets) : -1.0;

        // ── col 17: freeze_flag_e2e ───────────────────────────────────────
        // E2E freeze = no frame (Uu or PC5) met the deadline this epoch
        const int freezeFlag =
            ((es.uuDeadlineMetFrames + es.pc5VideoDeadlineMetFrames) == 0) ? 1 : 0;

        // ── col 13: consecutive_freeze_epochs (carry-over) ────────────────
        if (freezeFlag)
            ++es.consecutiveFreezeEpochs;
        else
            es.consecutiveFreezeEpochs = 0;

        // ── col 19: age_of_last_vehicle_state_ms ─────────────────────────
        const double ageMs = (es.lastVehicleStateRecvSec >= 0.0)
                                 ? (nowSec - es.lastVehicleStateRecvSec) * 1000.0
                                 : -1.0;

        // ── col 20-21: infrastructure distances ───────────────────────────
        const double distGnb = CalcDistToNearestGnb(es.lastRxX, es.lastRxY);
        const double distRsu = CalcDistToNearestRsu(es.lastRxX, es.lastRxY);

        // ── col 22-23: handover info ──────────────────────────────────────
        const HandoverState& ho     = m_hoStates[vehicleId];
        const bool           recentHo =
            (ho.lastHandoverTimeSec >= 0.0 &&
             (nowSec - ho.lastHandoverTimeSec) <= kRecentHoWindowSec);
        const double timeSinceHoMs =
            (ho.lastHandoverTimeSec >= 0.0)
                ? (nowSec - ho.lastHandoverTimeSec) * 1000.0
                : -1.0;

        // ── write row ─────────────────────────────────────────────────────
        EpochFile(vehicleId)
            << std::fixed
            // cols 1-3
            << es.epochIdx << ","
            << std::setprecision(1) << epochStartMs << ","
            << epochEndMs << ","
            // cols 4-8  y_recv^Uu
            << es.uuCompletedFrames << ","
            << es.uuTimeoutFrames << ","
            << es.uuDeadlineMetFrames << ","
            << std::setprecision(3) << uuAvgDelay << ","
            << es.uuLatencyMax << ","
            // cols 9-13  y_recv^PC5 video
            << es.pc5VideoCompletedFrames << ","
            << es.pc5VideoTimeoutFrames << ","
            << es.pc5VideoDeadlineMetFrames << ","
            << pc5VideoAvgDelay << ","
            << es.pc5VideoLatencyMax << ","
            // cols 14-16  y_recv^coop
            << es.coopRxPackets << ","
            << coopAvgDelay << ","
            << es.coopLatencyMax << ","
            // cols 17-18  y_svc
            << freezeFlag << ","
            << es.consecutiveFreezeEpochs << ","
            // cols 19-24  r_veh
            << std::setprecision(3) << es.lastRxX << ","
            << es.lastRxY << ","
            << es.lastRxSpeed << ","
            << es.lastRxHeading << ","
            << es.lastRxAccel << ","
            << std::setprecision(1) << ageMs << ","
            // cols 25-28  m_prior
            << std::setprecision(2) << distGnb << ","
            << distRsu << ","
            << (recentHo ? 1 : 0) << ","
            << std::setprecision(1) << timeSinceHoMs << ","
            // cols 29-30  v_scene
            << es.numIFrames << ","
            << es.numPFrames << "\n";

        // ── reset per-epoch counters ───────────────────────────────────────
        ++es.epochIdx;
        es.epochStartSec             = nowSec;
        es.uuCompletedFrames         = 0;
        es.uuTimeoutFrames           = 0;
        es.uuDeadlineMetFrames       = 0;
        es.uuLatencySum              = 0.0;
        es.uuLatencyMax              = -1.0;
        es.pc5VideoCompletedFrames   = 0;
        es.pc5VideoTimeoutFrames     = 0;
        es.pc5VideoDeadlineMetFrames = 0;
        es.pc5VideoLatencySum        = 0.0;
        es.pc5VideoLatencyMax        = -1.0;
        es.coopRxPackets             = 0;
        es.coopLatencySum            = 0.0;
        es.coopLatencyMax            = -1.0;
        es.numIFrames                = 0;
        es.numPFrames                = 0;
        // consecutiveFreezeEpochs, lastRx*, lastVehicleStateRecvSec — carry-over
    }

    m_epochSummaryEvent =
        Simulator::Schedule(Seconds(m_epochIntervalSec), &ServerSinkApp::WriteEpochSummaries, this);
}

// ---------------------------------------------------------------------------
void
ServerSinkApp::StartApplication()
{
    m_vehiclesDir = m_outputDir + "/vehicles";
    std::filesystem::create_directories(m_vehiclesDir);

    m_videoSocket = MakeUdpSink(m_videoPort, MakeCallback(&ServerSinkApp::RecvVideo, this));
    m_metaSocket  = MakeUdpSink(m_metaPort,  MakeCallback(&ServerSinkApp::RecvMeta,  this));
    m_coopSocket  = MakeUdpSink(m_coopPort,  MakeCallback(&ServerSinkApp::RecvCoop,  this));

    m_deadlineCheckEvent =
        Simulator::Schedule(MilliSeconds(50), &ServerSinkApp::CheckFrameDeadlines, this);
    m_epochSummaryEvent =
        Simulator::Schedule(Seconds(m_epochIntervalSec), &ServerSinkApp::WriteEpochSummaries, this);

    std::cout << "[ServerSinkApp] Per-vehicle logs → " << m_vehiclesDir << "/\n"
              << "  Epoch T_d      : " << m_epochIntervalSec * 1000.0 << " ms\n"
              << "  Frame deadline : " << m_frameDeadlineSec * 1000.0 << " ms\n"
              << "  gNBs known     : " << m_gnbXY.size() << "\n"
              << "  RSUs known     : " << m_rsuXY.size() << "\n";
}

void
ServerSinkApp::StopApplication()
{
    m_deadlineCheckEvent.Cancel();
    m_epochSummaryEvent.Cancel();

    // Flush remaining in-flight frames as sim-end truncation (treated as timeout/rejected)
    for (auto it = m_frames.begin(); it != m_frames.end();)
    {
        const auto& [key, state] = *it;
        if (state.totalChunks == 0) { it = m_frames.erase(it); continue; }

        const double latMs =
            (state.lastArrivalSec >= 0.0)
                ? (state.lastArrivalSec - state.sendTimeSec) * 1000.0
                : -1.0;
        FramesFile(key.first) << std::fixed << std::setprecision(6)
                              << key.second << ","
                              << (int)state.frameType << ","
                              << state.totalChunks << ","
                              << state.receivedChunks << ","
                              << state.usefulChunks << ","
                              << state.sendTimeSec << ","
                              << state.firstArrivalSec << ","
                              << state.lastArrivalSec << ","
                              << std::setprecision(3) << latMs << ",0," << state.path << "\n";

        auto& es = m_epochStates[key.first];
        if (state.path == std::string("Uu"))
        {
            ++es.uuCompletedFrames;
            ++es.uuTimeoutFrames;
        }
        else
        {
            ++es.pc5VideoCompletedFrames;
            ++es.pc5VideoTimeoutFrames;
        }

        ++m_framesTimeout;
        it = m_frames.erase(it);
    }

    if (m_videoSocket) { m_videoSocket->Close(); m_videoSocket = nullptr; }
    if (m_metaSocket)  { m_metaSocket->Close();  m_metaSocket  = nullptr; }
    if (m_coopSocket)  { m_coopSocket->Close();  m_coopSocket  = nullptr; }

    std::cout << "[ServerSinkApp] Done.\n"
              << "  Video pkts : " << m_totalVideoPackets << "\n"
              << "  Meta pkts  : " << m_totalMetaPackets  << "\n"
              << "  Coop pkts  : " << m_totalCoopPackets  << "\n"
              << "  Frames OK  : " << m_framesComplete    << "\n"
              << "  Frames TO  : " << m_framesTimeout     << "\n";
}

// ---------------------------------------------------------------------------
Ptr<Socket>
ServerSinkApp::MakeUdpSink(uint16_t port, Callback<void, Ptr<Socket>> recvCb)
{
    Ptr<Socket> sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), port));
    sock->SetRecvCallback(recvCb);
    return sock;
}

// ---------------------------------------------------------------------------
// RecvVideo
// ---------------------------------------------------------------------------
void
ServerSinkApp::RecvVideo(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    Address     fromAddr;
    while ((pkt = socket->RecvFrom(fromAddr)))
    {
        ++m_totalVideoPackets;

        VideoPacketHeader hdr;
        pkt->RemoveHeader(hdr);

        const double recvTime = Simulator::Now().GetSeconds();
        const double latMs    = (recvTime - hdr.common.sendTimeSec) * 1000.0;

        // Path detection from pathFlag embedded by sender (1=Uu, 0=PC5).
        // IP-prefix detection is unreliable because both Uu and SL interfaces
        // are assigned 7.x.x.x addresses by epcHelper->AssignUeIpv4Address().
        const char* path = hdr.pathFlag ? "Uu" : "PC5";

        // Per-vehicle video raw log
        VideoRawFile(hdr.common.vehicleId)
            << std::fixed << std::setprecision(6)
            << recvTime << "," << hdr.frameId << ","
            << hdr.chunkId << "," << hdr.totalChunks << ","
            << (hdr.frameType ? 'I' : 'P') << ","
            << hdr.payloadSize << "," << hdr.common.sendTimeSec << ","
            << std::setprecision(3) << latMs << "," << path << "\n";

        // Mark as video vehicle on first chunk
        auto& es = m_epochStates[hdr.common.vehicleId];
        if (!es.isVideoVehicle)
        {
            es.isVideoVehicle = true;
            es.epochStartSec  = Simulator::Now().GetSeconds();
        }

        // Frame state tracking (playout-aware)
        const auto key   = std::make_pair(hdr.common.vehicleId, hdr.frameId);
        auto&      state = m_frames[key];
        if (state.totalChunks == 0)
        {
            state.totalChunks     = hdr.totalChunks;
            state.frameType       = hdr.frameType;
            state.sendTimeSec     = hdr.common.sendTimeSec;
            state.firstArrivalSec = recvTime;
            state.path            = path;
        }

        // Skip already-judged frames (deadline already fired)
        if (state.judged)
            return;

        ++state.receivedChunks;
        state.lastArrivalSec = recvTime;

        // Count as useful only if within playout deadline
        const double age = recvTime - state.sendTimeSec;
        if (age <= m_frameDeadlineSec)
            ++state.usefulChunks;

        NS_LOG_DEBUG("RecvVideo veh=" << hdr.common.vehicleId
                                      << " frame=" << hdr.frameId
                                      << " chunk=" << hdr.chunkId << "/" << hdr.totalChunks
                                      << " path=" << path << " lat=" << latMs << "ms");
    }
}

// ---------------------------------------------------------------------------
// RecvMeta — updates r_veh telemetry cache (x, y, speed, heading)
// ---------------------------------------------------------------------------
void
ServerSinkApp::RecvMeta(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    while ((pkt = socket->Recv()))
    {
        ++m_totalMetaPackets;

        MetaPacketHeader hdr;
        pkt->RemoveHeader(hdr);

        const double recvTime = Simulator::Now().GetSeconds();
        const double latMs    = (recvTime - hdr.common.sendTimeSec) * 1000.0;

        MetaFile(hdr.common.vehicleId)
            << std::fixed << std::setprecision(6)
            << recvTime << "," << hdr.frameId << ","
            << hdr.latitude << "," << hdr.longitude << ","
            << hdr.heading << "," << hdr.speed << ","
            << hdr.cpuLoad << "," << hdr.memUsage << ","
            << hdr.common.sendTimeSec << ","
            << std::setprecision(3) << latMs << "\n";

        // Update r_veh cache (latest received value wins within epoch)
        auto& es                     = m_epochStates[hdr.common.vehicleId];
        es.lastRxX                   = hdr.latitude;   // SUMO x stored as latitude
        es.lastRxY                   = hdr.longitude;  // SUMO y stored as longitude
        es.lastRxSpeed               = hdr.speed;
        es.lastRxHeading             = hdr.heading;
        es.lastVehicleStateRecvSec   = recvTime;
        // Note: lastRxAccel is updated from AwarenessMessageHeader (coop socket)

        NS_LOG_DEBUG("RecvMeta veh=" << hdr.common.vehicleId
                                     << " x=" << hdr.latitude << " y=" << hdr.longitude
                                     << " lat=" << latMs << "ms");
    }
}

// ---------------------------------------------------------------------------
// RecvCoop — dispatches awareness/hazard; updates r_veh acceleration + PC5 stats
// ---------------------------------------------------------------------------
void
ServerSinkApp::RecvCoop(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    while ((pkt = socket->Recv()))
    {
        ++m_totalCoopPackets;

        // Peek CommonUplinkFields (13 B) to read appType without consuming
        CommonUplinkFields common;
        {
            constexpr uint32_t kSz = 13;
            uint8_t            raw[kSz]{};
            pkt->CopyData(raw, kSz);
            Buffer       rawBuf;
            rawBuf.AddAtStart(kSz);
            Buffer::Iterator w = rawBuf.Begin();
            w.Write(raw, kSz);
            Buffer::Iterator r = rawBuf.Begin();
            common.Deserialize(r);
        }

        const double recvTime = Simulator::Now().GetSeconds();
        const double latMs    = (recvTime - common.sendTimeSec) * 1000.0;

        if (common.appType == V2X_APP_AWARENESS)
        {
            AwarenessMessageHeader hdr;
            pkt->RemoveHeader(hdr);

            CoopFile(hdr.common.vehicleId)
                << std::fixed << std::setprecision(6)
                << recvTime << ",awareness,"
                << hdr.latitude << "," << hdr.longitude << ","
                << hdr.speedMs << "," << hdr.headingDeg << "," << hdr.accelMs2
                << ",-1,-1,"
                << hdr.common.sendTimeSec << ","
                << std::setprecision(3) << latMs << "\n";

            // Update r_veh acceleration (only field not in MetaPacketHeader)
            m_epochStates[hdr.common.vehicleId].lastRxAccel = hdr.accelMs2;

            NS_LOG_DEBUG("RecvAwareness veh=" << hdr.common.vehicleId
                                              << " accel=" << hdr.accelMs2
                                              << " lat=" << latMs << "ms");
        }
        else if (common.appType == V2X_APP_HAZARD)
        {
            HazardEventHeader hdr;
            pkt->RemoveHeader(hdr);

            CoopFile(hdr.common.vehicleId)
                << std::fixed << std::setprecision(6)
                << recvTime << ",hazard,"
                << hdr.latitude << "," << hdr.longitude << ",-1,-1,-1,"
                << (int)hdr.hazardType << "," << (int)hdr.severity << ","
                << hdr.common.sendTimeSec << ","
                << std::setprecision(3) << latMs << "\n";

            NS_LOG_INFO("RecvHazard veh=" << hdr.common.vehicleId
                                          << " type=" << (int)hdr.hazardType
                                          << " sev=" << (int)hdr.severity);
        }
        else
        {
            NS_LOG_WARN("RecvCoop: unknown appType=" << (int)common.appType << " — discarding");
        }

        // Accumulate coop epoch stats (all coop message types)
        auto& es = m_epochStates[common.vehicleId];
        ++es.coopRxPackets;
        es.coopLatencySum += latMs;
        if (es.coopLatencyMax < 0.0 || latMs > es.coopLatencyMax)
            es.coopLatencyMax = latMs;
    }
}

// ---------------------------------------------------------------------------
// CheckFrameDeadlines — 50 ms periodic scan; judges frames at playout deadline
// ---------------------------------------------------------------------------
void
ServerSinkApp::CheckFrameDeadlines()
{
    const double now = Simulator::Now().GetSeconds();

    for (auto it = m_frames.begin(); it != m_frames.end();)
    {
        auto& [key, state] = *it;
        if (!state.judged && state.totalChunks > 0 &&
            (now - state.sendTimeSec) > m_frameDeadlineSec)
        {
            state.judged = true;

            // Playout-aware acceptance: usefulChunks / totalChunks >= threshold
            const double threshold = (state.frameType == 1) ? m_iFrameThreshold
                                                             : m_pFrameThreshold;
            const bool accepted =
                ((double)state.usefulChunks / state.totalChunks) >= threshold;

            const double latMs =
                (state.lastArrivalSec >= 0.0)
                    ? (state.lastArrivalSec - state.sendTimeSec) * 1000.0
                    : -1.0;

            FramesFile(key.first)
                << std::fixed << std::setprecision(6)
                << key.second << ","
                << (int)state.frameType << ","
                << state.totalChunks << ","
                << state.receivedChunks << ","
                << state.usefulChunks << ","
                << state.sendTimeSec << ","
                << state.firstArrivalSec << ","
                << state.lastArrivalSec << ","
                << std::setprecision(3) << latMs << ","
                << (accepted ? 1 : 0) << "," << state.path << "\n";

            // Epoch accumulation — split by path
            auto& es = m_epochStates[key.first];
            if (state.path == std::string("Uu"))
            {
                ++es.uuCompletedFrames;  // every judged Uu frame
                if (accepted)
                {
                    ++es.uuDeadlineMetFrames;
                    es.uuLatencySum += latMs;
                    if (es.uuLatencyMax < 0.0 || latMs > es.uuLatencyMax)
                        es.uuLatencyMax = latMs;
                }
                else
                {
                    ++es.uuTimeoutFrames;
                }
            }
            else // PC5
            {
                ++es.pc5VideoCompletedFrames;
                if (accepted)
                {
                    ++es.pc5VideoDeadlineMetFrames;
                    es.pc5VideoLatencySum += latMs;
                    if (es.pc5VideoLatencyMax < 0.0 || latMs > es.pc5VideoLatencyMax)
                        es.pc5VideoLatencyMax = latMs;
                }
                else
                {
                    ++es.pc5VideoTimeoutFrames;
                }
            }

            // v_scene: count accepted frames by type
            if (accepted)
            {
                if (state.frameType == 1)
                    ++es.numIFrames;
                else
                    ++es.numPFrames;
            }

            if (accepted)
                ++m_framesComplete;
            else
                ++m_framesTimeout;

            it = m_frames.erase(it);
        }
        else
        {
            ++it;
        }
    }

    m_deadlineCheckEvent =
        Simulator::Schedule(MilliSeconds(50), &ServerSinkApp::CheckFrameDeadlines, this);
}

} // namespace ns3
