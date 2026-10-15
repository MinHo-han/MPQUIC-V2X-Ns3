#ifndef MPQUIC_SOCKET_BASE_H
#define MPQUIC_SOCKET_BASE_H

#include "mpquic-path.h"
#include "mpquic-scheduler.h"

#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/inet-socket-address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/socket.h"

#include <array>
#include <deque>
#include <fstream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ns3
{

/**
 * \ingroup mpquic
 *
 * Core multipath QUIC transport object.
 *
 * Manages a set of MpQuicPath objects (each backed by a UDP socket) and
 * forwards outgoing packets through a pluggable MpQuicScheduler.
 * Incoming packets from all paths are unified through a single receive
 * callback that also reports which path the packet arrived on.
 *
 * Typical usage:
 * \code
 *   MpQuicHelper helper;
 *   auto mpquic = helper.InstallWithTwoPaths(vehicleNode,
 *                     InetSocketAddress(rrAddr, uuPort),
 *                     InetSocketAddress(rrPc5Addr, pc5Port));
 *   mpquic->SetRecvCallback(MakeCallback(&MyApp::OnReceive, this));
 *   mpquic->Send(packet);
 * \endcode
 */
class MpQuicSocketBase : public Object
{
  public:
    static TypeId GetTypeId();

    MpQuicSocketBase();
    ~MpQuicSocketBase() override;

    /**
     * Attach to a node and set the scheduler.  Must be called before AddPath().
     */
    void Setup(Ptr<Node> node, Ptr<MpQuicScheduler> scheduler);

    /// Set the connection identifier embedded in every outgoing packet header.
    /// Call once after Setup(), before the first Send().
    void SetConnId(uint32_t id);

    /**
     * Add a path backed by a new UDP socket bound on m_node.
     * \param remote  Remote server address and port for this path.
     * \param type    UU or PC5 (NR-V2X Rel. 16 sidelink).
     * \return        The assigned path ID (0-based, monotone).
     */
    uint8_t AddPath(InetSocketAddress remote,
                MpQuicPathType    type,
                Ipv4Address       localAddr = Ipv4Address::GetAny());

    /**
     * Send packet through the scheduler.
     * \param packet  payload (MpQuicHeader prepended by MpQuicPath::Send)
     * \param info    stream metadata: streamId, priority, deadline
     * \return bytes sent, or -1 on failure / drop.
     */
    int Send(Ptr<Packet> packet, const MpQuicStreamInfo& info = MpQuicStreamInfo{});

    /**
     * Unified receive callback: (pathId, packet, senderAddress).
     */
    void SetRecvCallback(Callback<void, uint8_t, Ptr<Packet>, Address> cb);

    /// Enable or disable a path by ID.
    void SetPathActive(uint8_t pathId, bool active);

    /**
     * Provide an RTT sample for path pathId.
     * Call this from the application whenever a round-trip is measured.
     */
    void UpdatePathRtt(uint8_t pathId, Time rtt);

    /// Print per-path TX/RX/RTT/retx statistics to stdout.
    void PrintStats() const;

    const std::vector<Ptr<MpQuicPath>>& GetPaths() const;

    /// Return the current scheduler (allows caller to cast and configure it).
    Ptr<MpQuicScheduler> GetScheduler() const { return m_scheduler; }

    /// Total retransmitted packets across all paths.
    uint64_t GetTotalRetxPackets() const;

    /**
     * Stage 1-C: Application-level queue monitor.
     * Returns the total number of unACKed (in-flight) packets summed across
     * all paths.  This is the sender-side occupancy observable by the TiL agent.
     */
    uint32_t GetTotalQueueDepth() const;

    /// Cumulative unique application frames submitted for transmission on stream sid.
    /// m_streamSeq[sid] is incremented once per Send() call before path dispatch,
    /// so this counter is capture-timestamp-ordered and scheduler-independent.
    uint32_t GetStreamSeqNo(uint8_t sid = 0) const { return m_streamSeq[sid]; }

    /**
     * Enable per-frame sender-side scheduler trace.
     * CSV columns: time_ms, conn_id, stream_seq, sched_label,
     *              uu_active, uu_blocked, uu_cwnd, uu_pending, uu_rtt_ms,
     *              pc5_active, pc5_blocked, pc5_cwnd, pc5_pending, pc5_rtt_ms,
     *              tx_path, deferred, dropped
     */
    void SetSchedLog(std::shared_ptr<std::ofstream> log) { m_schedLog = std::move(log); }
    void SetPktLog(std::shared_ptr<std::ofstream> log)  { m_pktLog  = std::move(log); }

    /// Register a callback invoked when a reliable stream frame is permanently dropped
    /// (kMaxRetries exhausted with no ACK received).  Callback signature: (connId, streamId, streamSeqNo).
    /// The receiver should use this to advance its expectedSeq past the lost frame,
    /// semantically equivalent to RFC 9000 RESET_STREAM for that frame.
    void SetPermanentDropCallback(Callback<void, uint32_t, uint8_t, uint32_t> cb)
    {
        m_permanentDropCb = cb;
    }

  private:
    void HandleRead(Ptr<Socket> socket);

    // Phase 2: periodic loss check & retransmission
    void ScheduleRetxCheck();
    void CheckRetransmissions();
    void Retransmit(Ptr<Packet> payload, uint8_t streamId, uint32_t streamSeqNo);

    /// Drain the deferred send queue, sending packets on any non-blocked path.
    /// Called after each ACK that frees a CWND slot.
    void FlushDeferredSends();

    Ptr<Node>                                     m_node;
    Ptr<MpQuicScheduler>                          m_scheduler;
    uint32_t                                      m_connId{0};
    std::vector<Ptr<MpQuicPath>>                  m_paths;
    Callback<void, uint8_t, Ptr<Packet>, Address> m_recvCb;
    Callback<void, uint32_t, uint8_t, uint32_t>   m_permanentDropCb; ///< invoked on kMaxRetries exhaustion

    // Stage 1-C: global per-stream sequence counters (32 streams max)
    static constexpr uint8_t kMaxStreams = 32;
    std::array<uint32_t, 32> m_streamSeq{};  ///< next streamSeqNo per stream

    // Retransmission timer
    Time    m_retxCheckInterval{MilliSeconds(5)};
    EventId m_retxEvent;

    // Deferred send queue: packets buffered when all paths are CWND-blocked.
    // streamSeq is pre-assigned at the original Send() call to preserve ordering.
    struct DeferredSend
    {
        Ptr<Packet>      packet;
        MpQuicStreamInfo info;
        uint32_t         streamSeq;
        uint64_t         captureTimeNs{0};
        // Per-path replication decision computed once at Send() time for Redundant
        // schedulers.  Empty for non-Redundant.  pathId → should_replicate.
        std::unordered_map<uint8_t, bool> replicateMap;
    };
    static constexpr uint32_t    kMaxDeferredSends = 500;
    std::deque<DeferredSend>     m_deferredSends;

    // Sender-side scheduler trace log (optional)
    std::shared_ptr<std::ofstream> m_schedLog;

    // Detailed per-packet trace log (optional): TX/DROP events including Redundant paths
    // CSV: t_ms,conn,stream,seq,event,path,uu_cwnd,uu_pend,pc5_cwnd,pc5_pend
    std::shared_ptr<std::ofstream> m_pktLog;
};

} // namespace ns3
#endif // MPQUIC_SOCKET_BASE_H
