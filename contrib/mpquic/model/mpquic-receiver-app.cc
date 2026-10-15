#include "mpquic-receiver-app.h"

#include "mpquic-header.h"

#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicReceiverApp");
NS_OBJECT_ENSURE_REGISTERED(MpQuicReceiverApp);

TypeId
MpQuicReceiverApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MpQuicReceiverApp")
            .SetParent<Application>()
            .SetGroupName("MpQuic")
            .AddConstructor<MpQuicReceiverApp>()
            .AddAttribute("Port",
                          "UDP port to listen on",
                          UintegerValue(4000),
                          MakeUintegerAccessor(&MpQuicReceiverApp::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("DelayThreshold",
                          "One-way delay threshold D̄ for continuity metric",
                          TimeValue(MilliSeconds(150)),
                          MakeTimeAccessor(&MpQuicReceiverApp::m_delayThreshold),
                          MakeTimeChecker())
            .AddAttribute("LossThreshold",
                          "Per-epoch loss rate threshold γ_L (0–1)",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&MpQuicReceiverApp::m_lossThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("FreezeThreshold",
                          "Maximum tolerable gap between consecutive in-order deliveries",
                          TimeValue(MilliSeconds(300)),
                          MakeTimeAccessor(&MpQuicReceiverApp::m_freezeThreshold),
                          MakeTimeChecker())
            .AddAttribute("EpochInterval",
                          "Duration of each Γ_t measurement epoch",
                          TimeValue(MilliSeconds(1000)),
                          MakeTimeAccessor(&MpQuicReceiverApp::m_epochInterval),
                          MakeTimeChecker());
    return tid;
}

MpQuicReceiverApp::MpQuicReceiverApp()
{
}

MpQuicReceiverApp::~MpQuicReceiverApp()
{
}

void
MpQuicReceiverApp::StartApplication()
{
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
    m_socket->SetRecvCallback(MakeCallback(&MpQuicReceiverApp::HandleRead, this));
    NS_LOG_INFO("MpQuicReceiverApp listening on port " << m_port);

    m_epochTimer = Simulator::Schedule(m_epochInterval,
                                       &MpQuicReceiverApp::ComputeEpochGamma, this);
}

void
MpQuicReceiverApp::StopApplication()
{
    m_epochTimer.Cancel();
    if (m_socket)
    {
        m_socket->Close();
        m_socket = nullptr;
    }
}

// ---------------------------------------------------------------------------
// HandleRead
// ---------------------------------------------------------------------------

void
MpQuicReceiverApp::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    Address     from;
    while ((pkt = socket->RecvFrom(from)))
    {
        if (pkt->GetSize() < MpQuicHeader::kSize)
        {
            NS_LOG_WARN("MpQuicReceiverApp: packet too small (" << pkt->GetSize()
                        << " B), dropping");
            continue;
        }

        MpQuicHeader hdr;
        pkt->RemoveHeader(hdr);

        if (hdr.IsAck())
        {
            NS_LOG_WARN("MpQuicReceiverApp: unexpected ACK, ignoring");
            continue;
        }

        uint32_t connId      = hdr.GetConnId();
        uint8_t  streamId    = hdr.GetStreamId();
        uint64_t pktNo       = hdr.GetPktNumber();
        uint32_t streamSeqNo = hdr.GetStreamSeqNo();
        uint64_t sendNs      = hdr.GetSendTimeNs();
        uint64_t captureNs   = hdr.GetCaptureTimeNs();

        double owdMs =
            (Simulator::Now() - NanoSeconds(static_cast<int64_t>(sendNs))).GetMilliSeconds();
        owdMs = std::max(0.0, owdMs);

        InetSocketAddress fromDbg = InetSocketAddress::ConvertFrom(from);
        NS_LOG_DEBUG("MpQuicReceiverApp RX conn=" << connId
                     << " stream=" << static_cast<int>(streamId)
                     << " pktNo=" << pktNo
                     << " streamSeq=" << streamSeqNo
                     << " payload=" << pkt->GetSize() << " B"
                     << " OWD=" << owdMs << " ms"
                     << " from=" << fromDbg.GetIpv4());

        // Frame log written inside BufferAndDeliver on first-seen streamSeqNo,
        // deduplicating by streamSeqNo across multi-RSU PC5 copies.
        BufferAndDeliver(connId, streamId, streamSeqNo, pkt, owdMs,
                         hdr.GetPathId(), captureNs);

        // ACK deduplication: NR SL groupcast causes every RSU in range to
        // forward the same PC5 data to remoteHost, generating N_RSU copies.
        // All ACKs route back via the nearest RSU's SL TX (capacity-limited).
        // Only send an ACK for the first copy of each (srcIpv4, pktNo, sendNs) triple.
        // sendNs distinguishes retransmissions (new sendNs) from multi-RSU duplicates
        // (same sendNs), so retransmission ACKs are never suppressed (Bug 2 fix).
        InetSocketAddress fromAddr = InetSocketAddress::ConvertFrom(from);
        uint32_t srcIpv4 = fromAddr.GetIpv4().Get();
        AckDedupKey dedupKey{srcIpv4, pktNo, sendNs};
        bool isDuplicate = (m_ackedPktKeys.count(dedupKey) != 0);
        if (!isDuplicate)
        {
            m_ackedPktKeys.insert(dedupKey);
            m_ackedPktOrder.push_back(dedupKey);
            // Evict oldest entries when window overflows
            while (m_ackedPktOrder.size() > kAckDedupWindow)
            {
                m_ackedPktKeys.erase(m_ackedPktOrder.front());
                m_ackedPktOrder.erase(m_ackedPktOrder.begin());
            }

            // Send ACK: echo connId + streamId + pktNo for sender's RTT/pending-map update
            MpQuicHeader ackHdr;
            ackHdr.SetIsAck(true);
            ackHdr.SetConnId(connId);
            ackHdr.SetStreamId(streamId);
            ackHdr.SetPktNumber(pktNo);
            ackHdr.SetSendTimeNs(sendNs);
            ackHdr.SetStreamSeqNo(streamSeqNo);

            Ptr<Packet> ackPkt = Create<Packet>(0);
            ackPkt->AddHeader(ackHdr);
            socket->SendTo(ackPkt, 0, from);
        }
        else
        {
            NS_LOG_DEBUG("MpQuicReceiverApp: duplicate pktNo=" << pktNo
                         << " sendNs=" << sendNs
                         << " from " << fromAddr.GetIpv4() << " — ACK suppressed");
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 1-A: reorder buffer (per connId+streamId)
// ---------------------------------------------------------------------------

void
MpQuicReceiverApp::BufferAndDeliver(uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                                    Ptr<Packet> payload, double owdMs,
                                    uint8_t pathId, uint64_t captureNs)
{
    uint64_t key = MakeKey(connId, streamId);
    auto& st = m_streams[key];   // creates default-constructed entry if new

    // ── Per-path unique delivery counting ──────────────────────────────────────
    // Dedup by (streamSeqNo, pathId): each unique (frame, path) is counted ONCE.
    // Multiple RSU copies of a PC5 groupcast frame share (streamSeqNo, pathId=1)
    // and all arrive at remoteHost via different backhaul IPs, but we count only once.
    // seenUu / seenPc5 are bounded sets; evict smallest entry (oldest streamSeqNo) when full.
    {
        auto& seen   = (pathId == 0) ? st.seenUu        : st.seenPc5;
        auto& rxCnt  = (pathId == 0) ? st.rxUuFrames    : st.rxPc5Frames;
        auto& onCnt  = (pathId == 0) ? st.rxUuOnTime    : st.rxPc5OnTime;
        if (seen.count(streamSeqNo) == 0)
        {
            seen.insert(streamSeqNo);
            while (seen.size() > kPathSeenWindow)
                seen.erase(seen.begin());  // oldest (smallest seqNo) first
            ++rxCnt;
            double capTs = static_cast<double>(captureNs) / 1e6;
            double rxTs  = Simulator::Now().GetMilliSeconds();
            if (rxTs <= capTs + m_deadlineMs)
                ++onCnt;
        }
    }

    if (!st.seen)
    {
        st.avgOwdMs     = owdMs;
        st.expectedSeq  = streamSeqNo;
        st.maxSeenSeq   = streamSeqNo;
        st.seen         = true;
        st.lastDelivery = Simulator::Now();
    }
    else
    {
        st.avgOwdMs = 0.1 * owdMs + 0.9 * st.avgOwdMs;
    }

    // Already delivered
    if (streamSeqNo < st.expectedSeq)
    {
        NS_LOG_DEBUG("MpQuicReceiverApp: conn=" << connId
                     << " stream=" << static_cast<int>(streamId)
                     << " duplicate/late seq=" << streamSeqNo
                     << " (expected " << st.expectedSeq << ")");
        return;
    }

    // Already buffered
    if (st.reorderBuf.count(streamSeqNo))
    {
        NS_LOG_DEBUG("MpQuicReceiverApp: conn=" << connId
                     << " stream=" << static_cast<int>(streamId)
                     << " already buffered seq=" << streamSeqNo);
        return;
    }

    // Update expected-packet counter for loss rate estimation
    if (streamSeqNo > st.maxSeenSeq)
    {
        st.epochExpected += streamSeqNo - st.maxSeenSeq;
        st.maxSeenSeq     = streamSeqNo;
    }

    if (streamSeqNo != st.expectedSeq)
        ++m_oooCount;

    // Frame log and E2E on-time are both written at the reorder-buffer DRAIN point
    // (application delivery), not here at network arrival.  This ensures that the
    // server_rx_ts_ms column in frames.csv is the actual drain timestamp, and that
    // the late_flag reflects whether the application received the frame in time.

    ++m_rxPackets;
    ++st.rxPackets;
    ++st.epochDelivered;
    m_rxBytes += payload->GetSize();
    st.lastDelivery = Simulator::Now();

    NS_LOG_DEBUG("MpQuicReceiverApp: conn=" << connId
                 << " stream=" << static_cast<int>(streamId)
                 << " seq=" << streamSeqNo
                 << (streamSeqNo != st.expectedSeq ? " (OOO)" : ""));

    // Store (payload, captureNs, pathId, arrivalNs) so the drain loop can write frames.csv
    // at drain time and compute HOL blocking duration (drainNs - arrivalNs).
    uint64_t arrivalNs = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
    st.reorderBuf[streamSeqNo] = std::make_tuple(payload, captureNs, pathId, arrivalNs);

    if (m_rxTraceLog && m_rxTraceLog->is_open())
    {
        *m_rxTraceLog << std::fixed << std::setprecision(3)
            << (arrivalNs / 1e6) << "," << connId << "," << static_cast<int>(streamId)
            << "," << streamSeqNo << ",ARRIVE," << static_cast<int>(pathId)
            << "," << st.reorderBuf.size() << "," << st.expectedSeq << ",0.000\n";
    }

    // Overflow guard
    if (st.reorderBuf.size() > kMaxBufSize)
    {
        NS_LOG_WARN("MpQuicReceiverApp: conn=" << connId
                    << " reorder buffer overflow, skipping ahead");
        st.expectedSeq = st.reorderBuf.begin()->first;
    }

    // Drain consecutive entries — application delivery point.
    DrainConsecutive(st, connId, streamId);
}

// ---------------------------------------------------------------------------
// DrainConsecutive: drain buffered in-order frames, then advance past any
// pre-stored pending skips so buffered frames behind the skip are released.
// ---------------------------------------------------------------------------

void
MpQuicReceiverApp::DrainConsecutive(PerConnStreamState& st,
                                    uint32_t connId, uint8_t streamId)
{
    auto& buf = st.reorderBuf;
    bool advanced = true;
    while (advanced)
    {
        advanced = false;

        // Drain consecutive in-order buffered frames
        while (!buf.empty() && buf.begin()->first == st.expectedSeq)
        {
            Ptr<Packet> drainPkt       = std::get<0>(buf.begin()->second);
            uint64_t    drainCaptureNs = std::get<1>(buf.begin()->second);
            uint8_t     drainPathId    = std::get<2>(buf.begin()->second);
            uint64_t    drainArrivalNs = std::get<3>(buf.begin()->second);

            double drainCapTs = static_cast<double>(drainCaptureNs) / 1e6;
            double drainTs    = Simulator::Now().GetMilliSeconds();
            bool   drainOk    = (drainTs <= drainCapTs + m_deadlineMs);
            if (drainOk)
                ++st.rxE2eOnTime;

            if (m_frameLog && m_frameLog->is_open())
            {
                *m_frameLog << connId << "," << st.expectedSeq << ","
                            << static_cast<int>(drainPathId) << ","
                            << std::fixed << std::setprecision(3)
                            << drainCapTs << "," << drainTs << ","
                            << (drainCapTs + m_deadlineMs) << ","
                            << 1 << "," << (drainOk ? 0 : 1) << ","
                            << drainPkt->GetSize() << "\n";
            }

            if (m_rxTraceLog && m_rxTraceLog->is_open())
            {
                uint64_t drainNs = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
                double holMs = static_cast<double>(drainNs - drainArrivalNs) / 1e6;
                *m_rxTraceLog << std::fixed << std::setprecision(3)
                    << (drainNs / 1e6) << "," << connId << "," << static_cast<int>(streamId)
                    << "," << st.expectedSeq << ",DRAIN," << static_cast<int>(drainPathId)
                    << "," << (buf.size() - 1) << "," << (st.expectedSeq + 1) << "," << holMs << "\n";
            }

            buf.erase(buf.begin());
            ++st.expectedSeq;
            advanced = true;
        }

        // Advance past a pre-announced sender drop at the new head.
        // This releases buffered frames that were HOL-blocked behind it.
        if (st.pendingSkips.count(st.expectedSeq))
        {
            NS_LOG_WARN("MpQuicReceiverApp: conn=" << connId
                        << " stream=" << static_cast<int>(streamId)
                        << " advancing past pre-stored skip seq=" << st.expectedSeq);
            if (m_rxTraceLog && m_rxTraceLog->is_open())
            {
                double tMs = Simulator::Now().GetMilliSeconds();
                *m_rxTraceLog << std::fixed << std::setprecision(3)
                    << tMs << "," << connId << "," << static_cast<int>(streamId)
                    << "," << st.expectedSeq << ",SKIP,255"
                    << "," << buf.size() << "," << (st.expectedSeq + 1) << ",0.000\n";
            }
            st.pendingSkips.erase(st.expectedSeq);
            buf.erase(st.expectedSeq);  // clean up if somehow buffered
            ++st.expectedSeq;
            advanced = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 1-B: Γ_t continuity metric
// ---------------------------------------------------------------------------

void
MpQuicReceiverApp::ComputeEpochGamma()
{
    Time now = Simulator::Now();

    for (auto& [key, st] : m_streams)
    {
        if (!st.seen)
            continue;

        double lossRate = 0.0;
        if (st.epochExpected > 0)
        {
            lossRate = 1.0 - static_cast<double>(st.epochDelivered)
                             / static_cast<double>(st.epochExpected);
            lossRate = std::max(0.0, lossRate);
        }

        bool frozen  = (now - st.lastDelivery) > m_freezeThreshold;
        st.frozen    = frozen;

        bool delayOk = (st.avgOwdMs <= m_delayThreshold.GetMilliSeconds());
        bool lossOk  = (lossRate <= m_lossThreshold);

        st.gamma = (delayOk && lossOk && !frozen) ? 1.0 : 0.0;

        uint32_t connId   = static_cast<uint32_t>(key >> 8);
        uint8_t  streamId = static_cast<uint8_t>(key & 0xFF);
        NS_LOG_INFO("[Γ] t=" << now.GetSeconds() << "s"
                    << "  conn=" << connId
                    << "  stream=" << static_cast<int>(streamId)
                    << "  Γ=" << st.gamma
                    << "  owd=" << st.avgOwdMs << "ms"
                    << "  loss=" << lossRate * 100.0 << "%"
                    << "  freeze=" << (frozen ? "yes" : "no")
                    << "  delivered=" << st.epochDelivered
                    << "  expected=" << st.epochExpected);

        st.epochDelivered = 0;
        st.epochExpected  = 0;
    }

    m_epochTimer = Simulator::Schedule(m_epochInterval,
                                       &MpQuicReceiverApp::ComputeEpochGamma, this);
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

uint64_t MpQuicReceiverApp::GetTotalRxPackets()  const { return m_rxPackets; }
uint64_t MpQuicReceiverApp::GetTotalRxBytes()    const { return m_rxBytes;   }
uint64_t MpQuicReceiverApp::GetOutOfOrderCount() const { return m_oooCount;  }

uint64_t
MpQuicReceiverApp::GetConnRxPackets(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.rxPackets : 0;
}

double
MpQuicReceiverApp::GetContinuity(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.gamma : 0.0;
}

uint64_t MpQuicReceiverApp::GetConnRxUuFrames(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.rxUuFrames : 0;
}

uint64_t MpQuicReceiverApp::GetConnRxPc5Frames(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.rxPc5Frames : 0;
}

uint64_t MpQuicReceiverApp::GetConnRxUuOnTime(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.rxUuOnTime : 0;
}

uint64_t MpQuicReceiverApp::GetConnRxPc5OnTime(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.rxPc5OnTime : 0;
}

uint64_t MpQuicReceiverApp::GetConnRxE2eOnTime(uint32_t connId, uint8_t streamId) const
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    return (it != m_streams.end()) ? it->second.rxE2eOnTime : 0;
}

void
MpQuicReceiverApp::SetFrameLogger(std::ofstream* log, double deadlineMs)
{
    m_frameLog   = log;
    m_deadlineMs = deadlineMs;
}

// ---------------------------------------------------------------------------
// SkipStreamSeqNo: advance expectedSeq past a permanently-dropped frame
// (RFC 9000 RESET_STREAM equivalent for a single-frame drop)
// ---------------------------------------------------------------------------

void
MpQuicReceiverApp::SkipStreamSeqNo(uint32_t connId, uint8_t streamId, uint32_t streamSeqNo)
{
    auto it = m_streams.find(MakeKey(connId, streamId));
    if (it == m_streams.end())
        return;
    auto& st = it->second;

    // Case 1: already past — another path delivered it before the drop was declared.
    if (streamSeqNo < st.expectedSeq)
    {
        NS_LOG_INFO("MpQuicReceiverApp: SkipStreamSeqNo NOOP conn=" << connId
                    << " streamSeq=" << streamSeqNo
                    << " expectedSeq=" << st.expectedSeq
                    << " (already delivered)");
        return;
    }

    // Case 2: ahead of the current head — store for when expectedSeq reaches it.
    // Without this, the drop notification would be silently discarded and the
    // receiver would HOL-block again once it advances to that sequence number.
    if (streamSeqNo > st.expectedSeq)
    {
        NS_LOG_WARN("MpQuicReceiverApp: conn=" << connId
                    << " streamSeq=" << streamSeqNo
                    << " > expectedSeq=" << st.expectedSeq
                    << " — storing as future skip");
        st.pendingSkips.insert(streamSeqNo);
        return;
    }

    // Case 3: exactly the current head — skip it and drain what follows.
    NS_LOG_WARN("MpQuicReceiverApp: conn=" << connId
                << " streamSeq=" << streamSeqNo
                << " permanently dropped by sender — advancing expectedSeq");

    st.reorderBuf.erase(streamSeqNo);  // clean up if somehow buffered
    ++st.expectedSeq;

    // Drain in-order buffered frames and advance past any additional pending skips.
    DrainConsecutive(st, connId, streamId);
}

} // namespace ns3
