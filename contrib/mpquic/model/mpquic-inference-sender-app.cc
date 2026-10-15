#include "mpquic-inference-sender-app.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicInferenceSenderApp");
NS_OBJECT_ENSURE_REGISTERED(MpQuicInferenceSenderApp);

TypeId
MpQuicInferenceSenderApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MpQuicInferenceSenderApp")
            .SetParent<Application>()
            .SetGroupName("MpQuic")
            .AddConstructor<MpQuicInferenceSenderApp>()
            .AddAttribute("MinResultBytes",
                          "Minimum inference result payload size (bytes)",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&MpQuicInferenceSenderApp::m_minResultBytes),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MaxResultBytes",
                          "Maximum inference result payload size (bytes)",
                          UintegerValue(8000),
                          MakeUintegerAccessor(&MpQuicInferenceSenderApp::m_maxResultBytes),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PktSize",
                          "MPQUIC fragment size (bytes)",
                          UintegerValue(1200),
                          MakeUintegerAccessor(&MpQuicInferenceSenderApp::m_pktSize),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("Fps",
                          "Inference rate (results per second)",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&MpQuicInferenceSenderApp::m_fps),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("Deadline",
                          "Per-result delivery deadline",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&MpQuicInferenceSenderApp::m_deadline),
                          MakeTimeChecker())
            .AddAttribute("StreamId",
                          "MPQUIC stream identifier for inference results",
                          UintegerValue(1),
                          MakeUintegerAccessor(&MpQuicInferenceSenderApp::m_streamId),
                          MakeUintegerChecker<uint8_t>());
    return tid;
}

MpQuicInferenceSenderApp::MpQuicInferenceSenderApp()
    : m_rng(CreateObject<UniformRandomVariable>())
{
}

MpQuicInferenceSenderApp::~MpQuicInferenceSenderApp()
{
}

void
MpQuicInferenceSenderApp::SetMpQuic(Ptr<MpQuicSocketBase> mpquic)
{
    m_mpquic = mpquic;
}

void
MpQuicInferenceSenderApp::StartApplication()
{
    NS_ASSERT_MSG(m_mpquic, "Call SetMpQuic() before starting the application");
    m_running  = true;
    m_pktCount = 0;
    SendResult();
}

void
MpQuicInferenceSenderApp::StopApplication()
{
    m_running = false;
    m_sendEvent.Cancel();
}

void
MpQuicInferenceSenderApp::SendResult()
{
    if (!m_running)
        return;

    // Variable result size: uniform between min and max
    uint32_t resultBytes = static_cast<uint32_t>(
        m_rng->GetValue(m_minResultBytes, m_maxResultBytes));

    // Stream metadata: high priority + explicit deadline
    MpQuicStreamInfo info;
    info.streamId = m_streamId;
    info.priority = 1;                    // latency-sensitive
    info.deadline = m_deadline;

    NS_LOG_DEBUG("InferenceSender: result=" << resultBytes << " B"
                 << " deadline=" << m_deadline.GetMilliSeconds() << " ms");

    // Fragment and send
    uint32_t remaining = resultBytes;
    while (remaining > 0)
    {
        uint32_t pktBytes = std::min(remaining, m_pktSize);
        auto     pkt      = Create<Packet>(pktBytes);
        m_mpquic->Send(pkt, info);
        remaining -= pktBytes;
        ++m_pktCount;
    }

    // Schedule next inference result
    m_sendEvent = Simulator::Schedule(Seconds(1.0 / m_fps),
                                      &MpQuicInferenceSenderApp::SendResult, this);
}

uint64_t
MpQuicInferenceSenderApp::GetTotalPacketsSent() const
{
    return m_pktCount;
}

} // namespace ns3
