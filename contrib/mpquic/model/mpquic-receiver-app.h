#ifndef MPQUIC_RECEIVER_APP_H
#define MPQUIC_RECEIVER_APP_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/socket.h"

#include <deque>
#include <fstream>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>

namespace ns3
{

/**
 * \ingroup mpquic
 *
 * Server-side MPQUIC receiver application.
 *
 * **Stage 1-A: Per-(connId, streamId) reordering buffer.**
 * Each (connection, stream) pair maintains an independent map<streamSeqNo, payload>.
 * On every packet arrival consecutive in-order entries are drained.  Out-of-order
 * arrivals are held until their predecessors appear or the buffer overflows
 * (kMaxBufSize entries), at which point the expected sequence is advanced.
 *
 * connId is read from the MpQuicHeader embedded in each packet and uniquely
 * identifies the sender (e.g. vehicle index).  This allows multiple senders to
 * share a single receiver port without sequence-number collisions even when all
 * senders use the same streamId.
 *
 * **Stage 1-B: Continuity metric Γ_t per (connId, streamId).**
 * Every EpochInterval (default 1 s) the receiver evaluates:
 *   Γ_t = 1  iff  OWD ≤ D̄  AND  loss_rate ≤ γ_L  AND  no_freeze
 *
 * All thresholds are configurable via ns-3 attributes.
 */
class MpQuicReceiverApp : public Application
{
  public:
    static TypeId GetTypeId();
    MpQuicReceiverApp();
    ~MpQuicReceiverApp() override;

    uint64_t GetTotalRxPackets() const;   ///< all-connection total received packets
    uint64_t GetTotalRxBytes() const;
    uint64_t GetOutOfOrderCount() const;  ///< packets that arrived OOO

    /// Packets received for a specific (connId, streamId) pair.
    uint64_t GetConnRxPackets(uint32_t connId, uint8_t streamId = 0) const;

    /// Γ_t continuity indicator for (connId, streamId). Returns 0 if unseen.
    double GetContinuity(uint32_t connId, uint8_t streamId = 0) const;

    /// Per-path unique frame delivery counts (cumulative).
    /// Dedup key: (streamSeqNo, pathId) — multiple RSU copies of PC5 counted once.
    uint64_t GetConnRxUuFrames  (uint32_t connId, uint8_t streamId = 0) const;
    uint64_t GetConnRxPc5Frames (uint32_t connId, uint8_t streamId = 0) const;
    uint64_t GetConnRxUuOnTime  (uint32_t connId, uint8_t streamId = 0) const;
    uint64_t GetConnRxPc5OnTime (uint32_t connId, uint8_t streamId = 0) const;
    /// E2E on-time: frames whose first delivery (either path) was within deadline.
    uint64_t GetConnRxE2eOnTime (uint32_t connId, uint8_t streamId = 0) const;

    /// Advance expectedSeq past a permanently-lost streamSeqNo (RFC 9000 RESET_STREAM equivalent).
    /// Called by MpQuicSocketBase when kMaxRetries is exhausted for a reliable frame.
    /// Drains any now-consecutive frames from the reorder buffer at the current time.
    void SkipStreamSeqNo(uint32_t connId, uint8_t streamId, uint32_t streamSeqNo);

    /// Wire a frame-level CSV log (caller owns the ofstream).
    void SetFrameLogger(std::ofstream* log, double deadlineMs);

    /// Wire diagnostic RX trace log: ARRIVE/DRAIN/SKIP per streamSeqNo with HOL duration.
    void SetRxTraceLog(std::shared_ptr<std::ofstream> log) { m_rxTraceLog = std::move(log); }

  private:
    void StartApplication() override;
    void StopApplication() override;
    void HandleRead(Ptr<Socket> socket);

    // Stage 1-A: per-(connId,streamId) state
    struct PerConnStreamState
    {
        // Reorder buffer stores (payload, captureNs, pathId, arrivalNs) so the drain loop
        // can compute application-layer delivery latency and HOL blocking duration.
        std::map<uint32_t, std::tuple<Ptr<Packet>, uint64_t, uint8_t, uint64_t>> reorderBuf;
        uint32_t  expectedSeq{0};
        uint32_t  maxSeenSeq{0};
        bool      seen{false};
        double    avgOwdMs{0.0};
        Time      lastDelivery{Seconds(0)};
        bool      frozen{false};
        double    gamma{1.0};
        uint64_t  epochDelivered{0};
        uint64_t  epochExpected{0};
        uint64_t  rxPackets{0};

        // Per-path unique delivery tracking.
        // seenUu / seenPc5: bounded sliding window; evict smallest (oldest) when full.
        // Multiple RSU copies of the same PC5 frame share (streamSeqNo, path_id=1) → one count.
        std::set<uint32_t>  seenUu;
        std::set<uint32_t>  seenPc5;
        uint64_t rxUuFrames{0};   ///< unique frames received via Uu (cumulative)
        uint64_t rxPc5Frames{0};  ///< unique frames received via PC5 (cumulative)
        uint64_t rxUuOnTime{0};   ///< Uu frames within deadline (cumulative)
        uint64_t rxPc5OnTime{0};  ///< PC5 frames within deadline (cumulative)
        // E2E first-delivery on-time (tracked alongside frame log write)
        uint64_t rxE2eOnTime{0};

        // Sequence numbers the sender pre-declared as permanently dropped before
        // the receiver's expectedSeq reached them.  When expectedSeq advances to
        // a value in this set, it is skipped automatically without HOL blocking.
        std::set<uint32_t> pendingSkips;
    };

    static constexpr uint32_t kPathSeenWindow = 512;

    // key = (uint64_t(connId) << 8) | streamId
    static uint64_t MakeKey(uint32_t connId, uint8_t streamId)
    {
        return (static_cast<uint64_t>(connId) << 8) | streamId;
    }

    // pathId and captureNs are passed so frame log can be written on first-seen packet,
    // deduplicating by streamSeqNo (not by pktNo) to suppress multi-RSU PC5 copies.
    void BufferAndDeliver(uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                          Ptr<Packet> payload, double owdMs,
                          uint8_t pathId, uint64_t captureNs);

    // Stage 1-B: periodic Γ_t computation
    void ComputeEpochGamma();

    // Drain consecutive in-order entries from st.reorderBuf, advancing
    // st.expectedSeq, and then skip any pending-skip heads so that a
    // pre-announced sender drop never blocks subsequent buffered frames.
    void DrainConsecutive(PerConnStreamState& st, uint32_t connId, uint8_t streamId);

    // ---- Configurable attributes ----
    uint16_t m_port{4000};
    Time     m_delayThreshold{MilliSeconds(150)};
    double   m_lossThreshold{0.05};
    Time     m_freezeThreshold{MilliSeconds(300)};
    Time     m_epochInterval{MilliSeconds(1000)};

    // ---- Socket ----
    Ptr<Socket> m_socket;

    // ---- Global counters ----
    uint64_t m_rxPackets{0};
    uint64_t m_rxBytes{0};
    uint64_t m_oooCount{0};

    // ---- Per-(connId,streamId) state — grows dynamically ----
    static constexpr uint32_t kMaxBufSize = 256;
    std::unordered_map<uint64_t, PerConnStreamState> m_streams;

    // ---- ACK deduplication: (srcIpv4, pktNo, sendNs) → ACK already sent ----
    // In NR SL Mode 2 (groupcast), all RSUs within range receive every data
    // packet and forward it to remoteHost via their own backhaul links.  This
    // causes remoteHost to receive N_RSU copies of every PC5 packet, generating
    // N_RSU ACKs.  All ACKs route back via the nearest RSU's SL interface,
    // overloading its SL TX capacity and causing ACK loss → pending fills to
    // cwnd limit → IsBlocked() permanently true.  Only ACK the first copy.
    // sendNs is included so that retransmissions (new sendNs, same pktNo) are
    // not silently suppressed — each retransmission attempt must be ACKable.
    // Window bounded by kAckDedupWindow to prevent unbounded memory growth.
    struct AckDedupKey {
        uint32_t srcIpv4;
        uint64_t pktNo;
        uint64_t sendNs;
        bool operator<(const AckDedupKey& o) const {
            if (srcIpv4 != o.srcIpv4) return srcIpv4 < o.srcIpv4;
            if (pktNo   != o.pktNo  ) return pktNo   < o.pktNo;
            return sendNs < o.sendNs;
        }
    };
    static constexpr uint32_t kAckDedupWindow = 2048;
    std::set<AckDedupKey>    m_ackedPktKeys;
    std::vector<AckDedupKey> m_ackedPktOrder; // FIFO for eviction

    // Per-frame CSV log (optional; caller-owned)
    std::ofstream* m_frameLog{nullptr};
    double         m_deadlineMs{150.0};

    // Diagnostic RX trace log (optional): ARRIVE / DRAIN / SKIP events per streamSeqNo
    std::shared_ptr<std::ofstream> m_rxTraceLog;

    // Epoch timer
    EventId m_epochTimer;
};

} // namespace ns3
#endif // MPQUIC_RECEIVER_APP_H
