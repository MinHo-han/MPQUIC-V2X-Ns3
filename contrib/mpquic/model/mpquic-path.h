#ifndef MPQUIC_PATH_H
#define MPQUIC_PATH_H

#include "ns3/inet-socket-address.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/socket.h"

#include <map>
#include <vector>

namespace ns3
{

enum class MpQuicPathType
{
    UU,    ///< NR Uu (5G cellular uplink)
    PC5    ///< NR-V2X sidelink (3GPP Rel. 16, PC5 Mode 2 sensing-based resource allocation)
};

/**
 * \ingroup mpquic
 *
 * Represents one network path used by the MPQUIC transport.
 * Owns a single UDP socket and tracks per-path statistics
 * (TX/RX counters, RTT estimate via EWMA).
 */
class MpQuicPath : public Object
{
  public:
    static TypeId GetTypeId();

    MpQuicPath(uint8_t pathId,
               MpQuicPathType type,
               Ptr<Socket> socket,
               InetSocketAddress remote);
    ~MpQuicPath() override;

    uint8_t        GetPathId() const;
    MpQuicPathType GetPathType() const;
    std::string    GetPathLabel() const;

    bool IsActive() const;
    void SetActive(bool active);

    /**
     * Send a packet on this path.
     * \param p           raw payload (no MpQuicHeader yet)
     * \param streamId    stream identifier embedded in the header
     * \param streamSeqNo global per-stream monotone sequence assigned by
     *                    MpQuicSocketBase; used by the receiver reorder buffer.
     *                    Retransmissions pass the ORIGINAL streamSeqNo.
     * \return bytes sent, or -1 on error
     */
    int Send(Ptr<Packet> p, uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
             bool isReliable = true, uint64_t captureTimeNs = 0);

    /// Update the EWMA RTT estimate with a new sample (alpha = 0.125).
    void UpdateRtt(Time rtt);
    Time GetRtt() const;
    Time GetRttVar() const;
    /// True once at least one RTT sample has been received (false = still showing initial value).
    bool IsRttInitialized() const;

    // --- Per-path counters ---
    uint64_t GetTxPackets() const;
    uint64_t GetTxBytes() const;
    uint64_t GetRxPackets() const;
    uint64_t GetRxBytes() const;
    void     IncrementRx(uint32_t bytes);

    Ptr<Socket>       GetSocket() const;
    InetSocketAddress GetRemoteAddress() const;
    /// Update the remote endpoint (called by Pc5RouteUpdater on RSU handover).
    void              SetRemote(InetSocketAddress remote);

    /// Instantaneous TX rate in bytes/ms (EWMA, α=0.1). Used by Peekaboo features.
    double GetTxRateBytesPerMs() const;

    // ---- Phase 2: loss detection & retransmission -------------------------

    /// Called by Send() to register an unACKed payload.
    /// payload must be the raw packet BEFORE the MpQuicHeader is prepended.
    void AddPending(uint64_t pktNo, Ptr<Packet> payload, Time sendTime,
                    uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                    bool isReliable, uint64_t captureTimeNs);

    /// Called when an ACK for pktNo arrives. Removes the entry.
    /// Returns true if the entry was found (and removed), false if it was already gone
    /// (duplicate or stale ACK).  Callers should only update RTT when this returns true.
    bool OnAck(uint64_t pktNo);

    /**
     * Scan the pending map and return payloads that have timed out.
     * Timed-out entries have their retry counter incremented and sendTime reset.
     * Entries that exceed kMaxRetries are removed (dropped).
     * \return list of payload copies to be retransmitted by the caller.
     */
    /// Retransmission entry: original pktNo + payload copy + original connection/stream metadata.
    struct RetxEntry { uint64_t pktNo; Ptr<Packet> payload; uint32_t connId; uint8_t streamId; uint32_t streamSeqNo; uint64_t captureTimeNs; };
    /// Permanently-dropped entry: emitted when kMaxRetries is exhausted for a reliable frame.
    struct DroppedEntry { uint32_t connId; uint8_t streamId; uint32_t streamSeqNo; };
    /// \param[out] dropped  Entries permanently dropped after kMaxRetries exhaustion.
    std::vector<RetxEntry> CollectLost(Time now, std::vector<DroppedEntry>& dropped);

    /**
     * Send a retransmission using the ORIGINAL packet number.
     * Does NOT create a new pending entry — the existing pending entry already
     * has its sendTime reset by CollectLost(). This prevents the retransmission
     * cascade that would occur if Send() were used (which assigns a new pktNo
     * and creates another pending entry).
     */
    int SendRetransmission(uint64_t pktNo, Ptr<Packet> payload,
                           uint32_t connId, uint8_t streamId, uint32_t streamSeqNo,
                           uint64_t captureTimeNs);

    uint64_t GetRetxPackets() const;
    uint32_t GetPendingCount() const;

    /// True if any pending entry on this path carries the given (connId, streamId, streamSeqNo).
    /// Used by CheckRetransmissions to suppress a permanent-drop callback when another path
    /// still has the same frame in flight (Redundant scheduler: PC5 drops, Uu still pending).
    bool HasStreamSeqPending(uint32_t connId, uint8_t streamId, uint32_t streamSeqNo) const;

    // ---- Phase 3: congestion control (AIMD / NewReno) ---------------------

    /**
     * True when the total unACKed packet count (including retransmissions) has reached
     * the CWND.  This rate-limits the total packet injection into the SL MAC so that
     * retransmissions + new sends never exceed the physical channel capacity.
     */
    bool IsBlocked() const;

    uint32_t GetCwndPackets() const;   ///< current CWND in packets
    uint32_t GetSsthresh() const;      ///< slow-start threshold in packets

    // Loss detection constants (public so socket base can read kMaxRetries)
    static constexpr uint8_t  kMaxRetries  = 3;
    static constexpr uint32_t kMaxPending  = 500; ///< per-path buffer cap

    // PTO constants (RFC 9002 §6.2)
    static constexpr double kGranularityMs        = 1.0;  ///< timer granularity (ms)
    static constexpr double kUuMaxAckDelayMs      = 25.0; ///< max_ack_delay for NR Uu (ms)
    /// For NR SL PC5 (Mode-2 SPS, RRI=50ms, slMaxTxTransNumPssch=1): PTO must exceed the
    /// worst-case SPS queuing delay (≤ RRI = 50ms) + Uu ACK (~9.5ms) = 59.5ms.
    /// With srtt ≈ 35ms and rttVar → 0, RFC 9002 gives PTO = 35+0+max_ack_delay.
    /// Setting max_ack_delay = 100ms ensures PTO ≥ 135ms >> 59.5ms (safe margin).
    /// Without this, a premature retransmission adds a duplicate to the SL MAC FIFO queue,
    /// inflating RTT measurements for all subsequent packets.
    static constexpr double kPc5MaxAckDelayMs     = 100.0; ///< max_ack_delay for NR SL PC5 (SPS-aware, ms)
    static constexpr double kRttVarBeta           = 0.25; ///< rttvar EWMA weight (RFC 6298)

    // Congestion control constants (RFC 9002 §7.2)
    static constexpr uint32_t kInitCwnd       = 10; ///< Initial CWND = 10 × max_datagram_size
    static constexpr uint32_t kInitSsthresh   = 64; ///< initial ssthresh (packets)
    static constexpr uint32_t kMinCwnd        = 2;  ///< minimum CWND for Uu (managed wireless)
    /// NR SL Mode 2 PC5: keep at most 1 packet in the SL MAC TX queue so the SPS
    /// queuing delay (avg 50 ms per 100 ms RRI) does not inflate the EWMA RTT beyond
    /// 100 ms.  When the single in-flight packet is lost (SPS collision), IsBlocked
    /// stays true for at most kMaxRetries × PTO ≈ 615 ms while Uu handles traffic.
    static constexpr uint32_t kMinCwndPc5     = 1;  ///< minimum CWND for PC5 (SPS-rate limited)
    /// Hard ceiling: SPS BDP ≈ 65ms×15pkt/s ≈ 1 pkt; prevents slow-start
    /// queue buildup from inflating EWMA RTT beyond 100 ms.
    static constexpr uint32_t kMaxCwndPc5     = 1;  ///< maximum CWND for PC5 (SPS BDP ceiling)

  private:
    /// Entry in the unACKed packet buffer.
    struct PendingEntry
    {
        Ptr<Packet> payload;            ///< raw payload (no MpQuicHeader)
        Time        sendTime;           ///< time of (last) transmission attempt
        uint8_t     retries{0};
        uint32_t    connId{0};          ///< connection identifier (preserved on retx)
        uint8_t     streamId{0};        ///< stream this packet belongs to (for retx)
        uint32_t    streamSeqNo{0};     ///< global per-stream seq (preserved on retx)
        bool        isReliable{true};   ///< false = Datagram (MPR-QUIC): drop on loss, no retx
        uint64_t    captureTimeNs{0};   ///< original frame-creation time; never updated on retx
    };
    uint8_t           m_pathId;
    MpQuicPathType    m_type;
    Ptr<Socket>       m_socket;
    InetSocketAddress m_remote;
    bool              m_active{true};

    // RTT estimate — RFC 9002 §5 (EWMA smoothed_rtt + rttvar)
    // Initial values are set in the constructor based on path type:
    //   Uu  : smoothed_rtt = 50 ms   (NR Uu scheduling pipeline latency)
    //   PC5 : smoothed_rtt = 333 ms  (RFC 9002 kInitialRtt; reset on first ACK to actual RTT)
    Time m_rttEstimate;            ///< smoothed_rtt (set in constructor)
    Time m_rttVar;                 ///< rttvar (set in constructor, init = smoothed_rtt / 2)
    bool m_rttInitialized{false};
    double m_maxAckDelayMs;        ///< path-specific max_ack_delay (set in constructor)

    // Counters
    uint64_t m_txPackets{0};
    uint64_t m_txBytes{0};
    uint64_t m_rxPackets{0};
    uint64_t m_rxBytes{0};

    // EWMA TX rate (bytes/ms) for Peekaboo feature extraction
    double   m_txRateEwmaBytesPerMs{0.0};
    Time     m_lastSendTime;

    // Packet numbering (Phase 1)
    uint64_t m_nextPktNumber{0};

    // Retransmission state (Phase 2)
    std::map<uint64_t, PendingEntry> m_pending;
    uint64_t m_retxPackets{0};

    // Congestion control state (Phase 3)
    uint32_t m_cwnd{kInitCwnd};
    uint32_t m_ssthresh{kInitSsthresh};
    double   m_cwndFrac{0.0};  ///< fractional CWND accumulator for congestion avoidance
    bool     m_lossInRound{false}; ///< prevent multiple halvings per loss round
    uint32_t m_minCwnd{kMinCwnd};  ///< per-path minimum CWND (set in constructor)
};

} // namespace ns3
#endif // MPQUIC_PATH_H
