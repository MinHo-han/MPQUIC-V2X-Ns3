#ifndef MPQUIC_HEADER_H
#define MPQUIC_HEADER_H

#include "ns3/header.h"

namespace ns3
{

/**
 * \ingroup mpquic
 *
 * Lightweight per-packet header prepended to every MpQuic data/ACK packet.
 *
 * Wire format (34 bytes):
 *   flags          : uint8_t   — bit 0: isAck, bit 1: pathId (0=Uu, 1=PC5)
 *   streamId       : uint8_t   — stream identifier within a connection (0=default)
 *   connId         : uint32_t  — connection identifier; uniquely identifies the sender
 *                                (e.g. vehicle index).  Receiver uses (connId, streamId)
 *                                as the key for per-stream reorder / continuity state.
 *   pktNumber      : uint64_t  — per-path sequence number (used for ACK/retx matching)
 *   sendTimeNs     : uint64_t  — Simulator::Now() at THIS transmission; updated on retx.
 *                                Echoed in ACK for per-attempt RTT measurement.
 *   captureTimeNs  : uint64_t  — Simulator::Now() at FIRST (original) send; never updated
 *                                on retransmissions.  Receiver uses this for deadline check.
 *   streamSeqNo    : uint32_t  — global per-stream sequence (used by reorder buffer)
 *
 * pktNumber is path-local and used for ACK → pending-map lookup.
 * streamSeqNo is assigned by MpQuicSocketBase and is monotone per stream across
 * all paths, so the receiver can reconstruct in-order delivery even under multipath.
 * Retransmissions preserve the original streamSeqNo.
 */
class MpQuicHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId        GetInstanceTypeId() const override;

    uint32_t GetSerializedSize() const override;
    void     Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void     Print(std::ostream& os) const override;

    void SetIsAck(bool ack);
    bool IsAck() const;

    void    SetPathId(uint8_t id);   ///< 0=Uu, 1=PC5 (stored in flags bit 1)
    uint8_t GetPathId() const;

    void    SetStreamId(uint8_t id);
    uint8_t GetStreamId() const;

    void     SetConnId(uint32_t id);
    uint32_t GetConnId() const;

    void     SetPktNumber(uint64_t n);
    uint64_t GetPktNumber() const;

    void     SetSendTimeNs(uint64_t ns);
    uint64_t GetSendTimeNs() const;

    void     SetCaptureTimeNs(uint64_t ns);
    uint64_t GetCaptureTimeNs() const;

    void     SetStreamSeqNo(uint32_t seq);
    uint32_t GetStreamSeqNo() const;

    static constexpr uint32_t kSize = 34; ///< serialised size: 1+1+4+8+8+8+4 bytes

  private:
    uint8_t  m_flags{0};
    uint8_t  m_streamId{0};
    uint32_t m_connId{0};
    uint64_t m_pktNumber{0};
    uint64_t m_sendTimeNs{0};
    uint64_t m_captureTimeNs{0};  ///< original frame-creation time; never updated on retx
    uint32_t m_streamSeqNo{0};
};

} // namespace ns3
#endif // MPQUIC_HEADER_H
