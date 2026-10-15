#include "mpquic-video-sender-app.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicVideoSenderApp");
NS_OBJECT_ENSURE_REGISTERED(MpQuicVideoSenderApp);

TypeId
MpQuicVideoSenderApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MpQuicVideoSenderApp")
            .SetParent<Application>()
            .SetGroupName("MpQuic")
            .AddConstructor<MpQuicVideoSenderApp>()
            .AddAttribute("GopSize",
                          "Number of frames per GoP (I-frame interval)",
                          UintegerValue(30),
                          MakeUintegerAccessor(&MpQuicVideoSenderApp::m_gopSize),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("IFrameBytes",
                          "I-frame payload size in bytes",
                          UintegerValue(40000),
                          MakeUintegerAccessor(&MpQuicVideoSenderApp::m_iFrameBytes),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PFrameBytes",
                          "P-frame payload size in bytes",
                          UintegerValue(4000),
                          MakeUintegerAccessor(&MpQuicVideoSenderApp::m_pFrameBytes),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("BFrameBytes",
                          "B-frame payload size in bytes",
                          UintegerValue(1500),
                          MakeUintegerAccessor(&MpQuicVideoSenderApp::m_bFrameBytes),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PktSize",
                          "MPQUIC fragment size (bytes)",
                          UintegerValue(1200),
                          MakeUintegerAccessor(&MpQuicVideoSenderApp::m_pktSize),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("Fps",
                          "Video frame rate (frames per second)",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&MpQuicVideoSenderApp::m_fps),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("StreamId",
                          "MPQUIC stream identifier for video",
                          UintegerValue(0),
                          MakeUintegerAccessor(&MpQuicVideoSenderApp::m_streamId),
                          MakeUintegerChecker<uint8_t>());
    return tid;
}

MpQuicVideoSenderApp::MpQuicVideoSenderApp()
{
}

MpQuicVideoSenderApp::~MpQuicVideoSenderApp()
{
}

void
MpQuicVideoSenderApp::SetMpQuic(Ptr<MpQuicSocketBase> mpquic)
{
    m_mpquic = mpquic;
}

void
MpQuicVideoSenderApp::StartApplication()
{
    NS_ASSERT_MSG(m_mpquic, "Call SetMpQuic() before starting the application");
    m_running    = true;
    m_frameCount = 0;
    m_pktCount   = 0;
    SendFrame();
}

void
MpQuicVideoSenderApp::StopApplication()
{
    m_running = false;
    m_sendEvent.Cancel();
}

void
MpQuicVideoSenderApp::SendFrame()
{
    if (!m_running)
        return;

    // Determine frame type within the GoP
    uint32_t posInGop   = m_frameCount % m_gopSize;
    bool     isIFrame   = (posInGop == 0);
    bool     isPFrame   = (!isIFrame && posInGop % 3 == 1);

    uint32_t frameBytes = isIFrame ? m_iFrameBytes
                        : isPFrame ? m_pFrameBytes
                                   : m_bFrameBytes;

    // Stream metadata: I-frames are high priority and reliable (Stream);
    // P/B-frames are best-effort unreliable (Datagram) — MPR-QUIC §5.1
    MpQuicStreamInfo info;
    info.streamId  = m_streamId;
    info.priority  = isIFrame ? 1 : 0;
    info.isReliable = isIFrame;

    const char* ftype = isIFrame ? "I" : isPFrame ? "P" : "B";
    NS_LOG_DEBUG("VideoSender: frame=" << m_frameCount
                 << " type=" << ftype
                 << " bytes=" << frameBytes
                 << " priority=" << static_cast<int>(info.priority));

    // Fragment and send
    uint32_t remaining = frameBytes;
    while (remaining > 0)
    {
        uint32_t pktBytes = std::min(remaining, m_pktSize);
        auto     pkt      = Create<Packet>(pktBytes);
        m_mpquic->Send(pkt, info);
        remaining -= pktBytes;
        ++m_pktCount;
    }

    ++m_frameCount;

    // Schedule next frame
    m_sendEvent = Simulator::Schedule(Seconds(1.0 / m_fps),
                                      &MpQuicVideoSenderApp::SendFrame, this);
}

uint64_t MpQuicVideoSenderApp::GetTotalPacketsSent() const { return m_pktCount;   }
uint64_t MpQuicVideoSenderApp::GetTotalFramesSent()  const { return m_frameCount; }

} // namespace ns3
