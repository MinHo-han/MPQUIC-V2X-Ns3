#include "mpquic-socket-base.h"

#include "mpquic-header.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket-factory.h"

#include <iomanip>
#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicSocketBase");
NS_OBJECT_ENSURE_REGISTERED(MpQuicSocketBase);

TypeId
MpQuicSocketBase::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MpQuicSocketBase")
                            .SetParent<Object>()
                            .SetGroupName("MpQuic")
                            .AddConstructor<MpQuicSocketBase>();
    return tid;
}

MpQuicSocketBase::MpQuicSocketBase()
{
}

MpQuicSocketBase::~MpQuicSocketBase()
{
}

void
MpQuicSocketBase::Setup(Ptr<Node> node, Ptr<MpQuicScheduler> scheduler)
{
    NS_ASSERT_MSG(node, "Node must not be null");
    NS_ASSERT_MSG(scheduler, "Scheduler must not be null");
    m_node      = node;
    m_scheduler = scheduler;
    ScheduleRetxCheck();
}

void
MpQuicSocketBase::SetConnId(uint32_t id)
{
    m_connId = id;
}

uint8_t
MpQuicSocketBase::AddPath(InetSocketAddress remote,
                           MpQuicPathType    type,
                           Ipv4Address       localAddr)
{
    NS_ASSERT_MSG(m_node, "Call Setup() before AddPath()");

    Ptr<Socket> udpSock =
        Socket::CreateSocket(m_node, UdpSocketFactory::GetTypeId());
    if (localAddr == Ipv4Address::GetAny())
        udpSock->Bind();
    else
        udpSock->Bind(InetSocketAddress(localAddr, 0));

    udpSock->SetRecvCallback(MakeCallback(&MpQuicSocketBase::HandleRead, this));

    auto pathId = static_cast<uint8_t>(m_paths.size());
    auto path   = CreateObject<MpQuicPath>(pathId, type, udpSock, remote);
    m_paths.push_back(path);

    NS_LOG_INFO("MpQuic: added path " << static_cast<int>(pathId) << " ["
                                      << path->GetPathLabel() << "] -> "
                                      << remote.GetIpv4() << ":" << remote.GetPort());
    return pathId;
}

int
MpQuicSocketBase::Send(Ptr<Packet> packet, const MpQuicStreamInfo& info)
{
    NS_ASSERT_MSG(m_scheduler, "No scheduler — call Setup() first");
    NS_ASSERT_MSG(!m_paths.empty(), "No paths — call AddPath() first");

    // Capture frame-creation time once; never updated on retransmissions (Bug 1 fix).
    uint64_t captureTimeNs = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());

    // Assign a global per-stream sequence number before any path sends it.
    // Retransmissions will use the ORIGINAL streamSeqNo (see Retransmit()).
    uint8_t  sid = (info.streamId < kMaxStreams) ? info.streamId : 0;
    uint32_t streamSeq = m_streamSeq[sid]++;

    if (m_scheduler->IsRedundant())
    {
        // Compute per-path replication decision ONCE at Send() time (not at flush).
        // Stores which paths should carry this frame; prevents double credit consumption.
        auto replicateMap = m_scheduler->ComputeReplicationMap(m_paths);

        int ret = -1;
        for (auto& p : m_paths)
        {
            if (!p->IsActive() || p->IsBlocked())
                continue;
            // Honour advisory replication decision if available; default = replicate on all active.
            auto it = replicateMap.find(p->GetPathId());
            bool shouldSend = replicateMap.empty() ? true : (it != replicateMap.end() && it->second);
            if (shouldSend)
            {
                ret = p->Send(packet->Copy(), m_connId, info.streamId, streamSeq, info.isReliable, captureTimeNs);
                if (m_pktLog && m_pktLog->is_open())
                {
                    double t = Simulator::Now().GetMilliSeconds();
                    *m_pktLog << std::fixed << std::setprecision(3)
                        << t << "," << m_connId << "," << static_cast<int>(info.streamId)
                        << "," << streamSeq << ",TX," << p->GetPathLabel()
                        << "," << p->GetCwndPackets() << "," << p->GetPendingCount() << "\n";
                }
            }
        }
        if (ret < 0)
        {
            // All paths blocked (or all advisory-suppressed): defer rather than drop.
            if (m_deferredSends.size() < kMaxDeferredSends)
            {
                // Store replicateMap so FlushDeferredSends() can honour the same decision.
                m_deferredSends.push_back({packet, info, streamSeq, captureTimeNs, std::move(replicateMap)});
                NS_LOG_DEBUG("MpQuic: Redundant all-blocked, deferring streamSeq=" << streamSeq);
                if (m_pktLog && m_pktLog->is_open())
                {
                    double t = Simulator::Now().GetMilliSeconds();
                    *m_pktLog << std::fixed << std::setprecision(3)
                        << t << "," << m_connId << "," << static_cast<int>(info.streamId)
                        << "," << streamSeq << ",DROP_REDUNDANT_BLOCKED,none,0,0\n";
                }
                return static_cast<int>(packet->GetSize());
            }
            NS_LOG_WARN("MpQuic: Redundant all-blocked + deferred full, dropping packet");
        }
        return ret;
    }

    // ── Snapshot path state BEFORE send decision ─────────────────────────────
    // Indexed [0]=Uu [1]=PC5 by convention (path order in m_paths).
    struct PathSnap { bool active; bool blocked; uint32_t cwnd; uint32_t pending; double rtt_ms; std::string label; };
    auto snap = [](const Ptr<MpQuicPath>& p) -> PathSnap {
        return { p->IsActive(), p->IsBlocked(),
                 p->GetCwndPackets(), p->GetPendingCount(),
                 p->GetRtt().GetMilliSeconds(), p->GetPathLabel() };
    };
    PathSnap snapUu  = { false, true,  0, 0, 0.0, "uu"  };
    PathSnap snapPc5 = { false, true,  0, 0, 0.0, "pc5" };
    for (const auto& p : m_paths)
    {
        if (p->GetPathLabel().find("uu") != std::string::npos  ||
            p->GetPathLabel().find("Uu") != std::string::npos  ||
            p->GetPathLabel().find("UU") != std::string::npos)
            snapUu = snap(p);
        else
            snapPc5 = snap(p);
    }

    Ptr<MpQuicPath> selected = m_scheduler->SelectPath(m_paths, info);
    if (!selected)
    {
        // Scheduler may return nullptr to request a defer (e.g. Peekaboo A0 wait),
        // not necessarily a drop.  Check the scheduler's intent before discarding.
        if (m_scheduler->DeferOnNullPath())
        {
            if (m_schedLog && m_schedLog->is_open())
            {
                double t = Simulator::Now().GetMilliSeconds();
                *m_schedLog << std::fixed << std::setprecision(3)
                    << t << "," << m_connId << "," << streamSeq << ",none,"
                    << snapUu.active  << "," << snapUu.blocked  << "," << snapUu.cwnd  << "," << snapUu.pending  << "," << snapUu.rtt_ms  << ","
                    << snapPc5.active << "," << snapPc5.blocked << "," << snapPc5.cwnd << "," << snapPc5.pending << "," << snapPc5.rtt_ms << ","
                    << "deferred,1,0\n";
            }
            if (m_deferredSends.size() < kMaxDeferredSends)
            {
                m_deferredSends.push_back({packet, info, streamSeq, captureTimeNs});
                NS_LOG_DEBUG("MpQuic: scheduler-requested defer streamSeq=" << streamSeq);
                return static_cast<int>(packet->GetSize());
            }
            NS_LOG_WARN("MpQuic: scheduler defer requested but deferred queue full, dropping"
                        " streamSeq=" << streamSeq);
        }
        else
        {
            NS_LOG_WARN("MpQuic: no active path / deadline infeasible, dropping packet"
                        " streamSeq=" << streamSeq);
            if (m_schedLog && m_schedLog->is_open())
            {
                double t = Simulator::Now().GetMilliSeconds();
                *m_schedLog << std::fixed << std::setprecision(3)
                    << t << "," << m_connId << "," << streamSeq << ",none,"
                    << snapUu.active  << "," << snapUu.blocked  << "," << snapUu.cwnd  << "," << snapUu.pending  << "," << snapUu.rtt_ms  << ","
                    << snapPc5.active << "," << snapPc5.blocked << "," << snapPc5.cwnd << "," << snapPc5.pending << "," << snapPc5.rtt_ms << ","
                    << "none,0,1\n";
            }
        }
        if (!m_permanentDropCb.IsNull())
            m_permanentDropCb(m_connId, info.streamId, streamSeq);
        return -1;
    }

    // Phase 3: if preferred path is CWND-blocked, try other active paths
    std::string schedLabel = selected->GetPathLabel();  // scheduler's first choice
    if (selected->IsBlocked())
    {
        Ptr<MpQuicPath> alt = nullptr;
        for (auto& p : m_paths)
        {
            if (p->IsActive() && !p->IsBlocked() && p != selected)
            {
                alt = p;
                break;
            }
        }
        if (alt)
        {
            NS_LOG_DEBUG("MpQuic: path " << static_cast<int>(selected->GetPathId())
                         << " CWND-blocked, rerouting to path "
                         << static_cast<int>(alt->GetPathId()));
            selected = alt;
        }
        else
        {
            // All paths CWND-blocked: defer rather than drop.
            if (m_schedLog && m_schedLog->is_open())
            {
                double t = Simulator::Now().GetMilliSeconds();
                *m_schedLog << std::fixed << std::setprecision(3)
                    << t << "," << m_connId << "," << streamSeq << "," << schedLabel << ","
                    << snapUu.active  << "," << snapUu.blocked  << "," << snapUu.cwnd  << "," << snapUu.pending  << "," << snapUu.rtt_ms  << ","
                    << snapPc5.active << "," << snapPc5.blocked << "," << snapPc5.cwnd << "," << snapPc5.pending << "," << snapPc5.rtt_ms << ","
                    << "deferred,1,0\n";
            }
            if (m_deferredSends.size() < kMaxDeferredSends)
            {
                m_deferredSends.push_back({packet, info, streamSeq, captureTimeNs});
                NS_LOG_DEBUG("MpQuic: all paths CWND-blocked, deferring streamSeq=" << streamSeq);
                return static_cast<int>(packet->GetSize());
            }
            NS_LOG_WARN("MpQuic: all paths CWND-blocked + deferred full, dropping packet"
                        " streamSeq=" << streamSeq);
            if (!m_permanentDropCb.IsNull())
                m_permanentDropCb(m_connId, info.streamId, streamSeq);
            return -1;
        }
    }

    // Write sender-side trace: scheduler choice vs actual TX path
    if (m_schedLog && m_schedLog->is_open())
    {
        double t = Simulator::Now().GetMilliSeconds();
        *m_schedLog << std::fixed << std::setprecision(3)
            << t << "," << m_connId << "," << streamSeq << "," << schedLabel << ","
            << snapUu.active  << "," << snapUu.blocked  << "," << snapUu.cwnd  << "," << snapUu.pending  << "," << snapUu.rtt_ms  << ","
            << snapPc5.active << "," << snapPc5.blocked << "," << snapPc5.cwnd << "," << snapPc5.pending << "," << snapPc5.rtt_ms << ","
            << selected->GetPathLabel() << ",0,0\n";
    }
    return selected->Send(packet, m_connId, info.streamId, streamSeq, info.isReliable, captureTimeNs);
}

void
MpQuicSocketBase::SetRecvCallback(Callback<void, uint8_t, Ptr<Packet>, Address> cb)
{
    m_recvCb = cb;
}

void
MpQuicSocketBase::SetPathActive(uint8_t pathId, bool active)
{
    for (auto& p : m_paths)
    {
        if (p->GetPathId() == pathId)
        {
            p->SetActive(active);
            NS_LOG_INFO("MpQuic: path " << static_cast<int>(pathId) << " ["
                                        << p->GetPathLabel() << "] active=" << active);
            return;
        }
    }
    NS_LOG_WARN("MpQuic: SetPathActive — path " << static_cast<int>(pathId) << " not found");
}

void
MpQuicSocketBase::UpdatePathRtt(uint8_t pathId, Time rtt)
{
    for (auto& p : m_paths)
    {
        if (p->GetPathId() == pathId)
        {
            p->UpdateRtt(rtt);
            return;
        }
    }
    NS_LOG_WARN("MpQuic: UpdatePathRtt — path " << static_cast<int>(pathId) << " not found");
}

void
MpQuicSocketBase::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address     from;
    while ((packet = socket->RecvFrom(from)))
    {
        // Every MpQuic packet carries a MpQuicHeader (34 bytes)
        if (packet->GetSize() < MpQuicHeader::kSize)
        {
            NS_LOG_WARN("MpQuic: HandleRead — packet too small (" << packet->GetSize()
                        << " B), skipping");
            continue;
        }

        MpQuicHeader hdr;
        packet->RemoveHeader(hdr);

        for (auto& p : m_paths)
        {
            if (p->GetSocket() != socket)
                continue;

            if (hdr.IsAck())
            {
                // Phase 2: remove from pending retx buffer.
                // Only update RTT when the pktNo was actually in pending (not a stale/
                // duplicate ACK for a packet already removed by an earlier copy's ACK).
                bool found = p->OnAck(hdr.GetPktNumber());
                if (found)
                {
                    Time sendTime = NanoSeconds(static_cast<int64_t>(hdr.GetSendTimeNs()));
                    Time rtt      = Simulator::Now() - sendTime;
                    if (rtt.IsStrictlyPositive())
                    {
                        p->UpdateRtt(rtt);
                    }
                    // An ACK freed a CWND slot — try to flush the deferred send queue.
                    FlushDeferredSends();
                }
            }
            else
            {
                // Data packet (unexpected on sender-only node, but handle gracefully)
                p->IncrementRx(packet->GetSize());
                NS_LOG_DEBUG("MpQuic: DATA RX on path "
                             << static_cast<int>(p->GetPathId())
                             << " [" << p->GetPathLabel() << "] "
                             << packet->GetSize() << " B");
                if (!m_recvCb.IsNull())
                {
                    m_recvCb(p->GetPathId(), packet, from);
                }
            }
            break;
        }
    }
}

void
MpQuicSocketBase::PrintStats() const
{
    std::cout << "=== MpQuic Path Statistics ===" << std::endl;
    for (const auto& p : m_paths)
    {
        std::cout << "  Path " << static_cast<int>(p->GetPathId())
                  << " [" << p->GetPathLabel() << "]"
                  << "  active=" << p->IsActive()
                  << "  RTT=" << p->GetRtt().GetMilliSeconds() << " ms"
                  << "  cwnd=" << p->GetCwndPackets()
                  << "  ssthresh=" << p->GetSsthresh()
                  << "  pending=" << p->GetPendingCount()
                  << "  TX=" << p->GetTxPackets() << " pkts / " << p->GetTxBytes() << " B"
                  << "  RX=" << p->GetRxPackets() << " pkts / " << p->GetRxBytes() << " B"
                  << "  retx=" << p->GetRetxPackets() << " pkts"
                  << std::endl;
    }
}

uint64_t
MpQuicSocketBase::GetTotalRetxPackets() const
{
    uint64_t total = 0;
    for (const auto& p : m_paths)
        total += p->GetRetxPackets();
    return total;
}

// ---------------------------------------------------------------------------
// Deferred send queue: drain buffered packets once a CWND slot becomes free
// ---------------------------------------------------------------------------

void
MpQuicSocketBase::FlushDeferredSends()
{
    while (!m_deferredSends.empty())
    {
        auto& entry = m_deferredSends.front();

        if (m_scheduler->IsRedundant())
        {
            // Use the replicateMap computed at original Send() time.
            // Condition 1: send on paths that are both permitted and available.
            bool sent = false;
            for (auto& p : m_paths)
            {
                if (!p->IsActive() || p->IsBlocked())
                    continue;
                auto it = entry.replicateMap.find(p->GetPathId());
                bool permitted = entry.replicateMap.empty()
                                 ? true
                                 : (it != entry.replicateMap.end() && it->second);
                if (permitted)
                {
                    p->Send(entry.packet->Copy(), m_connId, entry.info.streamId, entry.streamSeq, entry.info.isReliable, entry.captureTimeNs);
                    sent = true;
                }
            }
            // Condition 2: fallback — if advisory suppressed all available paths,
            // send on any single active non-blocked path to guarantee delivery.
            if (!sent)
            {
                for (auto& p : m_paths)
                {
                    if (p->IsActive() && !p->IsBlocked())
                    {
                        p->Send(entry.packet->Copy(), m_connId, entry.info.streamId, entry.streamSeq, entry.info.isReliable, entry.captureTimeNs);
                        sent = true;
                        NS_LOG_DEBUG("MpQuic: Redundant fallback (advisory map blocked all) → path "
                                     << static_cast<int>(p->GetPathId()));
                        break;
                    }
                }
            }
            if (!sent)
                break; // still all blocked
        }
        else
        {
            Ptr<MpQuicPath> selected = m_scheduler->SelectPath(m_paths, entry.info);
            if (selected && selected->IsBlocked())
            {
                // Preferred path blocked — try any non-blocked path
                for (auto& p : m_paths)
                {
                    if (p->IsActive() && !p->IsBlocked())
                    {
                        selected = p;
                        break;
                    }
                }
            }
            if (!selected || selected->IsBlocked())
                break; // still all blocked
            selected->Send(entry.packet, m_connId, entry.info.streamId, entry.streamSeq, entry.info.isReliable, entry.captureTimeNs);
        }

        m_deferredSends.pop_front();
    }
}

// ---------------------------------------------------------------------------
// Phase 2: periodic loss detection and retransmission
// ---------------------------------------------------------------------------

void
MpQuicSocketBase::ScheduleRetxCheck()
{
    m_retxEvent = Simulator::Schedule(m_retxCheckInterval,
                                      &MpQuicSocketBase::CheckRetransmissions,
                                      this);
}

void
MpQuicSocketBase::CheckRetransmissions()
{
    Time now = Simulator::Now();
    bool anyLost = false;
    for (auto& p : m_paths)
    {
        std::vector<MpQuicPath::DroppedEntry> droppedEntries;
        auto lostEntries = p->CollectLost(now, droppedEntries);
        if (!lostEntries.empty())
            anyLost = true;
        for (auto& entry : lostEntries)
        {
            NS_LOG_DEBUG("MpQuic: retransmitting pktNo=" << entry.pktNo
                         << " stream=" << static_cast<int>(entry.streamId)
                         << " streamSeq=" << entry.streamSeqNo
                         << " payload=" << entry.payload->GetSize() << " B"
                         << " from path " << static_cast<int>(p->GetPathId()));
            // Use the ORIGINAL pktNo — no new pending entry created.
            // This prevents the retransmission cascade where Send() would assign
            // a new pktNo and add another pending entry that then times out again.
            if (p->IsActive())
                p->SendRetransmission(entry.pktNo, entry.payload,
                                      entry.connId, entry.streamId, entry.streamSeqNo,
                                      entry.captureTimeNs);
            // If path is inactive, the entry will be pruned by kMaxRetries.
        }
        // Notify receiver for permanently dropped frames (kMaxRetries exhausted).
        // Guard: suppress the callback when another path still carries the same frame
        // (Redundant scheduler sends identical copies on both Uu and PC5).
        // If any other path's m_pending still holds the streamSeqNo, that path can
        // still deliver the frame — firing SkipStreamSeqNo now would wrongly advance
        // expectedSeq past a frame the receiver has not yet seen.
        // The callback is fired only when ALL paths have given up on the frame.
        if (!m_permanentDropCb.IsNull())
        {
            for (const auto& d : droppedEntries)
            {
                bool coveredElsewhere = false;
                for (const auto& other : m_paths)
                {
                    if (other == p) continue;
                    if (other->HasStreamSeqPending(d.connId, d.streamId, d.streamSeqNo))
                    {
                        coveredElsewhere = true;
                        break;
                    }
                }
                if (!coveredElsewhere)
                    m_permanentDropCb(d.connId, d.streamId, d.streamSeqNo);
                else
                    NS_LOG_WARN("MpQuicSocketBase: path " << static_cast<int>(p->GetPathId())
                                << " dropped streamSeq=" << d.streamSeqNo
                                << " but another path still pending — skip suppressed");
            }
        }
    }
    // CollectLost may have freed CWND slots (first-timeout entries leave m_inFlightCount).
    if (anyLost)
        FlushDeferredSends();
    ScheduleRetxCheck();
}

void
MpQuicSocketBase::Retransmit(Ptr<Packet> payload, uint8_t streamId, uint32_t streamSeqNo)
{
    MpQuicStreamInfo info;
    info.streamId = streamId;
    Ptr<MpQuicPath> selected = m_scheduler->SelectPath(m_paths, info);
    if (!selected)
    {
        NS_LOG_WARN("MpQuic::Retransmit — no active path, dropping");
        return;
    }
    // Preserve the original streamSeqNo so the receiver reorder buffer can
    // fill the hole without allocating a new sequence slot.
    selected->Send(payload, m_connId, streamId, streamSeqNo);
}

const std::vector<Ptr<MpQuicPath>>&
MpQuicSocketBase::GetPaths() const
{
    return m_paths;
}

uint32_t
MpQuicSocketBase::GetTotalQueueDepth() const
{
    uint32_t depth = 0;
    for (const auto& p : m_paths)
        depth += p->GetPendingCount();
    return depth;
}

} // namespace ns3
