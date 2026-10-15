#include "mpquic-path.h"

#include "mpquic-header.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicPath");
NS_OBJECT_ENSURE_REGISTERED(MpQuicPath);

TypeId
MpQuicPath::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MpQuicPath").SetParent<Object>().SetGroupName("MpQuic");
    return tid;
}

MpQuicPath::MpQuicPath(uint8_t pathId,
                       MpQuicPathType type,
                       Ptr<Socket> socket,
                       InetSocketAddress remote)
    : m_pathId(pathId),
      m_type(type),
      m_socket(socket),
      m_remote(remote)
{
    if (m_type == MpQuicPathType::PC5)
    {
        // RFC 9002 §6.2.2: use kInitialRtt = 333 ms in general, but for NR SL PC5 paths
        // with nearby RSUs (< 200 m), the actual E2E RTT is ~50–80 ms (SL SPS queueing
        // ~50 ms + Uu ACK path ~15 ms).  Starting at 333 ms inflates the PTO to ~1 s,
        // causing very slow EWMA convergence in the 5-vehicle scenario.  60 ms is a
        // conservative but realistic starting estimate that gives PTO ≈ 200 ms and
        // allows convergence within a few ACKs.
        m_rttEstimate   = MilliSeconds(60);
        m_rttVar        = MilliSeconds(30); // 60 / 2 = 30 ms
        m_maxAckDelayMs = kPc5MaxAckDelayMs;
        m_minCwnd       = kMinCwndPc5;
        m_cwnd          = kMinCwndPc5; // start at 1: no initial burst into the SL MAC queue
        m_ssthresh      = kMaxCwndPc5; // enter CA immediately; no slow-start for SPS paths
    }
    else
    {
        // NR Uu: conservative start for initial scheduling pipeline convergence.
        m_rttEstimate   = MilliSeconds(50);
        m_rttVar        = MilliSeconds(25);
        m_maxAckDelayMs = kUuMaxAckDelayMs;
        m_minCwnd       = kMinCwnd;
    }
}

MpQuicPath::~MpQuicPath()
{
}

uint8_t
MpQuicPath::GetPathId() const
{
    return m_pathId;
}

MpQuicPathType
MpQuicPath::GetPathType() const
{
    return m_type;
}

std::string
MpQuicPath::GetPathLabel() const
{
    return (m_type == MpQuicPathType::UU) ? "Uu" : "NR-V2X";
}

bool
MpQuicPath::IsActive() const
{
    return m_active;
}

void
MpQuicPath::SetActive(bool active)
{
    m_active = active;
}

int
MpQuicPath::Send(Ptr<Packet> p, uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                 bool isReliable, uint64_t captureTimeNs)
{
    uint64_t nowNs = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
    if (captureTimeNs == 0)
        captureTimeNs = nowNs;

    // Store payload copy BEFORE header is prepended (Phase 2 retx buffer)
    uint64_t pktNo = m_nextPktNumber;
    AddPending(pktNo, p, Simulator::Now(), connId, streamId, streamSeqNo, isReliable, captureTimeNs);

    // Prepend MpQuicHeader: path-local pktNo (ACK matching) + global streamSeqNo
    MpQuicHeader hdr;
    hdr.SetIsAck(false);
    hdr.SetConnId(connId);
    hdr.SetStreamId(streamId);
    hdr.SetPktNumber(m_nextPktNumber++);
    hdr.SetSendTimeNs(nowNs);
    hdr.SetCaptureTimeNs(captureTimeNs);
    hdr.SetStreamSeqNo(streamSeqNo);
    hdr.SetPathId(static_cast<uint8_t>(m_type == MpQuicPathType::PC5 ? 1 : 0));
    p->AddHeader(hdr);

    int ret = m_socket->SendTo(p, 0, m_remote);
    if (ret >= 0)
    {
        ++m_txPackets;
        m_txBytes += p->GetSize();

        // Update EWMA TX rate (bytes/ms), α=0.1
        Time now = Simulator::Now();
        if (!m_lastSendTime.IsZero())
        {
            double dt_ms = (now - m_lastSendTime).GetMilliSeconds();
            if (dt_ms > 0.0)
            {
                double instantRate = p->GetSize() / dt_ms;
                m_txRateEwmaBytesPerMs =
                    0.1 * instantRate + 0.9 * m_txRateEwmaBytesPerMs;
            }
        }
        m_lastSendTime = now;
        NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId) << " [" << GetPathLabel()
                             << "] TX " << p->GetSize() << " B");
    }
    else
    {
        NS_LOG_WARN("Path " << static_cast<int>(m_pathId) << " [" << GetPathLabel()
                            << "] SendTo FAILED ret=" << ret
                            << " errno=" << m_socket->GetErrno()
                            << " pktNo=" << (m_nextPktNumber - 1));
    }
    return ret;
}

int
MpQuicPath::SendRetransmission(uint64_t pktNo, Ptr<Packet> payload,
                               uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                               uint64_t captureTimeNs)
{
    // Use the ORIGINAL pktNo so the ACK removes the existing pending entry.
    // Do NOT call AddPending — CollectLost already updated sendTime/retries.
    MpQuicHeader hdr;
    hdr.SetIsAck(false);
    hdr.SetConnId(connId);
    hdr.SetStreamId(streamId);
    hdr.SetPktNumber(pktNo);   // original pktNo, not m_nextPktNumber
    hdr.SetSendTimeNs(static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()));
    hdr.SetCaptureTimeNs(captureTimeNs);
    hdr.SetStreamSeqNo(streamSeqNo);
    hdr.SetPathId(static_cast<uint8_t>(m_type == MpQuicPathType::PC5 ? 1 : 0));
    payload->AddHeader(hdr);

    int ret = m_socket->SendTo(payload, 0, m_remote);
    if (ret >= 0)
    {
        ++m_txPackets;
        m_txBytes += payload->GetSize();
        NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId) << " [" << GetPathLabel()
                             << "] RETX pktNo=" << pktNo << " " << payload->GetSize() << " B");
    }
    return ret;
}

void
MpQuicPath::UpdateRtt(Time rtt)
{
    // RFC 9002 §5.3 — on first sample bootstrap smoothed_rtt and rttvar
    if (!m_rttInitialized)
    {
        m_rttEstimate    = rtt;
        m_rttVar         = rtt * 0.5;   // rttvar = smoothed_rtt / 2
        m_rttInitialized = true;
    }
    else
    {
        // rttvar = (1 - β) × rttvar + β × |smoothed_rtt − rtt_sample|
        double smoothedMs = m_rttEstimate.GetMilliSeconds();
        double sampleMs   = rtt.GetMilliSeconds();
        double varMs      = m_rttVar.GetMilliSeconds();
        varMs             = (1.0 - kRttVarBeta) * varMs
                            + kRttVarBeta * std::abs(smoothedMs - sampleMs);
        m_rttVar          = MilliSeconds(varMs);

        // smoothed_rtt = (1 − α) × smoothed_rtt + α × rtt_sample  (α = 1/8)
        double updatedMs  = 0.875 * smoothedMs + 0.125 * sampleMs;
        m_rttEstimate     = MilliSeconds(updatedMs);
    }
    NS_LOG_INFO("Path " << static_cast<int>(m_pathId) << " [" << GetPathLabel()
                        << "] srtt=" << m_rttEstimate.GetMilliSeconds() << " ms"
                        << "  rttvar=" << m_rttVar.GetMilliSeconds() << " ms");
}

Time
MpQuicPath::GetRtt() const
{
    return m_rttEstimate;
}

Time
MpQuicPath::GetRttVar() const
{
    return m_rttVar;
}

bool
MpQuicPath::IsRttInitialized() const
{
    return m_rttInitialized;
}

double   MpQuicPath::GetTxRateBytesPerMs() const { return m_txRateEwmaBytesPerMs; }

uint64_t MpQuicPath::GetTxPackets() const { return m_txPackets; }
uint64_t MpQuicPath::GetTxBytes()   const { return m_txBytes;   }
uint64_t MpQuicPath::GetRxPackets() const { return m_rxPackets; }
uint64_t MpQuicPath::GetRxBytes()   const { return m_rxBytes;   }

void
MpQuicPath::IncrementRx(uint32_t bytes)
{
    ++m_rxPackets;
    m_rxBytes += bytes;
}

Ptr<Socket>
MpQuicPath::GetSocket() const
{
    return m_socket;
}

InetSocketAddress
MpQuicPath::GetRemoteAddress() const
{
    return m_remote;
}

void
MpQuicPath::SetRemote(InetSocketAddress remote)
{
    if (m_remote.GetIpv4() != remote.GetIpv4())
    {
        // RSU handover: clear the pending buffer so that ACKs for packets sent to the
        // OLD RSU (which may arrive late with inflated sendTimeNs) are silently discarded
        // instead of corrupting the EWMA.  Fresh packets to the new RSU will have
        // up-to-date sendTimeNs and produce accurate RTT samples.
        m_pending.clear();
    }
    m_remote = remote;
}

// ---------------------------------------------------------------------------
// Phase 2: loss detection & retransmission buffer
// ---------------------------------------------------------------------------

void
MpQuicPath::AddPending(uint64_t pktNo, Ptr<Packet> payload, Time sendTime,
                       uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                       bool isReliable, uint64_t captureTimeNs)
{
    if (m_pending.size() >= kMaxPending)
    {
        NS_LOG_WARN("Path " << static_cast<int>(m_pathId)
                            << ": pending buffer full, dropping oldest entry");
        m_pending.erase(m_pending.begin());
    }
    m_pending[pktNo] = {payload->Copy(), sendTime, 0, connId, streamId, streamSeqNo, isReliable, captureTimeNs};
}

bool
MpQuicPath::OnAck(uint64_t pktNo)
{
    auto it = m_pending.find(pktNo);
    if (it == m_pending.end())
    {
        // Stale / duplicate ACK — entry already removed (by earlier copy or retransmission ACK).
        // Do NOT update RTT; return false so caller skips RTT update.
        NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId)
                             << ": stale ACK pktNo=" << pktNo << " (not in pending)");
        return false;
    }
    {
        m_pending.erase(it);
        NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId)
                             << ": ACKed pktNo=" << pktNo
                             << "  pending=" << m_pending.size()
                             << "  cwnd=" << m_cwnd);

        // Phase 3: AIMD congestion control — increase CWND on ACK
        m_lossInRound = false; // new ACK means we're out of the loss event

        if (m_cwnd < m_ssthresh)
        {
            // Slow start: +1 per ACK (exponential growth)
            ++m_cwnd;
            NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId)
                                 << ": SlowStart cwnd=" << m_cwnd
                                 << " ssthresh=" << m_ssthresh);
        }
        else
        {
            // Congestion avoidance: +1/cwnd per ACK (≈ +1 per RTT)
            m_cwndFrac += 1.0 / static_cast<double>(m_cwnd);
            if (m_cwndFrac >= 1.0)
            {
                ++m_cwnd;
                m_cwndFrac -= 1.0;
                NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId)
                                     << ": CongAvoid cwnd=" << m_cwnd);
            }
        }
        // PC5: hard ceiling prevents slow-start from building an SPS MAC queue.
        if (m_type == MpQuicPathType::PC5)
            m_cwnd = std::min(m_cwnd, kMaxCwndPc5);
        return true;
    }
}

std::vector<MpQuicPath::RetxEntry>
MpQuicPath::CollectLost(Time now, std::vector<DroppedEntry>& dropped)
{
    std::vector<RetxEntry> lost;

    // RFC 9002 §6.2.1: PTO = smoothed_rtt + max(4×rttvar, kGranularity) + max_ack_delay
    // m_maxAckDelayMs: 25 ms for both Uu and PC5 (RFC 9000 default max_ack_delay).
    double srttMs   = m_rttEstimate.GetMilliSeconds();
    double rttVarMs = m_rttVar.GetMilliSeconds();
    double ptoMs    = srttMs
                      + std::max(4.0 * rttVarMs, kGranularityMs)
                      + m_maxAckDelayMs;
    Time   timeout  = MilliSeconds(ptoMs);

    bool anyLost = false;

    for (auto it = m_pending.begin(); it != m_pending.end();)
    {
        if (now - it->second.sendTime < timeout)
        {
            ++it;
            continue;
        }

        anyLost = true;

        // MPR-QUIC §5.1: unreliable (Datagram) packets are dropped on loss — no retransmission.
        // Loss is still counted for CWND halving (ACK-based congestion feedback is preserved).
        if (!it->second.isReliable)
        {
            NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId)
                                 << ": unreliable pktNo=" << it->first
                                 << " dropped (no retx, MPR-QUIC Datagram)");
            it = m_pending.erase(it);
            continue;
        }

        if (it->second.retries < kMaxRetries)
        {
            ++it->second.retries;
            it->second.sendTime = now;
            lost.push_back({it->first, it->second.payload->Copy(), it->second.connId, it->second.streamId, it->second.streamSeqNo, it->second.captureTimeNs});
            ++m_retxPackets;
            NS_LOG_DEBUG("Path " << static_cast<int>(m_pathId)
                                 << ": loss detected pktNo=" << it->first
                                 << "  retry=" << static_cast<int>(it->second.retries)
                                 << "  stream=" << static_cast<int>(it->second.streamId)
                                 << "  cwnd=" << m_cwnd);
            ++it;
        }
        else
        {
            NS_LOG_WARN("Path " << static_cast<int>(m_pathId)
                                << ": pktNo=" << it->first
                                << " streamSeq=" << it->second.streamSeqNo
                                << " dropped after " << static_cast<int>(kMaxRetries)
                                << " retries — signalling receiver to skip");
            dropped.push_back({it->second.connId, it->second.streamId, it->second.streamSeqNo});
            it = m_pending.erase(it);
        }
    }

    // Phase 3: AIMD multiplicative decrease — once per loss detection round
    if (anyLost && !m_lossInRound)
    {
        m_lossInRound = true;
        m_ssthresh    = std::max(m_cwnd / 2u, m_minCwnd);
        m_cwnd        = m_ssthresh;
        m_cwndFrac    = 0.0;
        NS_LOG_INFO("Path " << static_cast<int>(m_pathId)
                            << ": CWND halved → cwnd=" << m_cwnd
                            << "  ssthresh=" << m_ssthresh);
    }

    return lost;
}

uint64_t
MpQuicPath::GetRetxPackets() const
{
    return m_retxPackets;
}

uint32_t
MpQuicPath::GetPendingCount() const
{
    return static_cast<uint32_t>(m_pending.size());
}

bool
MpQuicPath::HasStreamSeqPending(uint32_t connId, uint8_t streamId, uint32_t streamSeqNo) const
{
    for (const auto& kv : m_pending)
    {
        const auto& e = kv.second;
        if (e.connId == connId && e.streamId == streamId && e.streamSeqNo == streamSeqNo)
            return true;
    }
    return false;
}

bool
MpQuicPath::IsBlocked() const
{
    return static_cast<uint32_t>(m_pending.size()) >= m_cwnd;
}

uint32_t
MpQuicPath::GetCwndPackets() const
{
    return m_cwnd;
}

uint32_t
MpQuicPath::GetSsthresh() const
{
    return m_ssthresh;
}

} // namespace ns3
