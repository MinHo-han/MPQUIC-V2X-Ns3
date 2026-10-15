#include "mpquic-sender-app.h"

#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicSenderApp");
NS_OBJECT_ENSURE_REGISTERED(MpQuicSenderApp);

TypeId
MpQuicSenderApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MpQuicSenderApp")
                            .SetParent<Application>()
                            .SetGroupName("MpQuic")
                            .AddConstructor<MpQuicSenderApp>();
    return tid;
}

MpQuicSenderApp::MpQuicSenderApp()
{
}

MpQuicSenderApp::~MpQuicSenderApp()
{
}

void
MpQuicSenderApp::SetMpQuic(Ptr<MpQuicSocketBase> mpquic)
{
    m_mpquic = mpquic;
}

void
MpQuicSenderApp::SetPacketSize(uint32_t bytes)
{
    m_pktSize = bytes;
}

void
MpQuicSenderApp::SetInterval(Time interval)
{
    m_interval = interval;
}

void
MpQuicSenderApp::SetStreamId(uint8_t id)
{
    m_streamId = id;
}

void
MpQuicSenderApp::SetConnId(uint32_t id)
{
    NS_ASSERT_MSG(m_mpquic, "SetConnId: call SetMpQuic() first");
    m_mpquic->SetConnId(id);
}

uint64_t
MpQuicSenderApp::GetTotalPacketsSent() const
{
    return m_pktCount;
}

void
MpQuicSenderApp::StartApplication()
{
    NS_ASSERT_MSG(m_mpquic, "MpQuicSenderApp: SetMpQuic() must be called before start");
    m_running = true;
    m_pktCount = 0;
    SendPacket();
}

void
MpQuicSenderApp::StopApplication()
{
    m_running = false;
    if (m_sendEvent.IsPending())
    {
        Simulator::Cancel(m_sendEvent);
    }
    NS_LOG_INFO("MpQuicSenderApp stopped after " << m_pktCount << " packets");
}

void
MpQuicSenderApp::SendPacket()
{
    if (!m_running)
    {
        return;
    }

    Ptr<Packet> pkt = Create<Packet>(m_pktSize);
    MpQuicStreamInfo info;
    info.streamId = m_streamId;
    int sent = m_mpquic->Send(pkt, info);

    if (sent > 0)
    {
        ++m_pktCount;
        NS_LOG_DEBUG("MpQuicSenderApp: sent pkt #" << m_pktCount
                                                   << " (" << m_pktSize << " B)"
                                                   << " at " << Simulator::Now().GetSeconds()
                                                   << " s");
    }

    m_sendEvent = Simulator::Schedule(m_interval, &MpQuicSenderApp::SendPacket, this);
}

} // namespace ns3
