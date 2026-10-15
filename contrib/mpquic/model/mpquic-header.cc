#include "mpquic-header.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicHeader");
NS_OBJECT_ENSURE_REGISTERED(MpQuicHeader);

TypeId
MpQuicHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MpQuicHeader")
                            .SetParent<Header>()
                            .SetGroupName("MpQuic")
                            .AddConstructor<MpQuicHeader>();
    return tid;
}

TypeId
MpQuicHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
MpQuicHeader::GetSerializedSize() const
{
    return kSize; // 1+1+4+8+8+8+4 = 34 bytes
}

void
MpQuicHeader::Serialize(Buffer::Iterator start) const
{
    start.WriteU8(m_flags);
    start.WriteU8(m_streamId);
    start.WriteHtonU32(m_connId);
    start.WriteHtonU64(m_pktNumber);
    start.WriteHtonU64(m_sendTimeNs);
    start.WriteHtonU64(m_captureTimeNs);
    start.WriteHtonU32(m_streamSeqNo);
}

uint32_t
MpQuicHeader::Deserialize(Buffer::Iterator start)
{
    m_flags          = start.ReadU8();
    m_streamId       = start.ReadU8();
    m_connId         = start.ReadNtohU32();
    m_pktNumber      = start.ReadNtohU64();
    m_sendTimeNs     = start.ReadNtohU64();
    m_captureTimeNs  = start.ReadNtohU64();
    m_streamSeqNo    = start.ReadNtohU32();
    return kSize;
}

void
MpQuicHeader::Print(std::ostream& os) const
{
    os << (IsAck() ? "ACK" : "DATA")
       << " conn=" << m_connId
       << " stream=" << static_cast<int>(m_streamId)
       << " pktNo=" << m_pktNumber
       << " streamSeq=" << m_streamSeqNo
       << " sendNs=" << m_sendTimeNs
       << " capNs=" << m_captureTimeNs;
}

void MpQuicHeader::SetIsAck(bool ack)
{
    if (ack) m_flags |= 0x01u;
    else     m_flags &= ~0x01u;
}
bool MpQuicHeader::IsAck() const { return (m_flags & 0x01u) != 0; }

void MpQuicHeader::SetPathId(uint8_t id)
{
    if (id) m_flags |= 0x02u;
    else    m_flags &= ~0x02u;
}
uint8_t MpQuicHeader::GetPathId() const { return (m_flags & 0x02u) ? 1 : 0; }

void    MpQuicHeader::SetStreamId(uint8_t id) { m_streamId = id; }
uint8_t MpQuicHeader::GetStreamId()     const { return m_streamId; }

void     MpQuicHeader::SetConnId(uint32_t id) { m_connId = id; }
uint32_t MpQuicHeader::GetConnId()      const { return m_connId; }

void     MpQuicHeader::SetPktNumber(uint64_t n) { m_pktNumber = n; }
uint64_t MpQuicHeader::GetPktNumber()    const  { return m_pktNumber; }

void     MpQuicHeader::SetSendTimeNs(uint64_t ns) { m_sendTimeNs = ns; }
uint64_t MpQuicHeader::GetSendTimeNs()     const  { return m_sendTimeNs; }

void     MpQuicHeader::SetCaptureTimeNs(uint64_t ns) { m_captureTimeNs = ns; }
uint64_t MpQuicHeader::GetCaptureTimeNs()      const { return m_captureTimeNs; }

void     MpQuicHeader::SetStreamSeqNo(uint32_t seq) { m_streamSeqNo = seq; }
uint32_t MpQuicHeader::GetStreamSeqNo()       const { return m_streamSeqNo; }

} // namespace ns3
