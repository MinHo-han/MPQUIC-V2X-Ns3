// SPDX-License-Identifier: GPL-2.0-only
#include "edge-video-metadata-app.h"

#include "v2x-headers.h"

#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EdgeVideoMetadataApp");
NS_OBJECT_ENSURE_REGISTERED(EdgeVideoMetadataApp);

TypeId
EdgeVideoMetadataApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::EdgeVideoMetadataApp")
            .SetParent<Application>()
            .SetGroupName("V2xRemoteSupport")
            .AddConstructor<EdgeVideoMetadataApp>()
            // ── Uu path ──────────────────────────────────────────────────────
            .AddAttribute("ServerAddressUu",
                          "Edge server IPv4 address reachable via NR Uu",
                          Ipv4AddressValue(Ipv4Address("1.0.0.2")),
                          MakeIpv4AddressAccessor(&EdgeVideoMetadataApp::m_serverAddrUu),
                          MakeIpv4AddressChecker())
            .AddAttribute("VideoPortUu",
                          "UDP port for video chunks on Uu path",
                          UintegerValue(5000),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_videoPortUu),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("MetaPortUu",
                          "UDP port for metadata on Uu path",
                          UintegerValue(5001),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_metaPortUu),
                          MakeUintegerChecker<uint16_t>())
            // ── PC5 path ─────────────────────────────────────────────────────
            .AddAttribute("ServerAddressPc5",
                          "Edge server IPv4 address reachable via NR SL PC5 + RSU backhaul. "
                          "Leave as 0.0.0.0 to disable PC5 video path.",
                          Ipv4AddressValue(Ipv4Address("0.0.0.0")),
                          MakeIpv4AddressAccessor(&EdgeVideoMetadataApp::m_serverAddrPc5),
                          MakeIpv4AddressChecker())
            .AddAttribute("VideoPortPc5",
                          "UDP port for video chunks on PC5 path",
                          UintegerValue(5000),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_videoPortPc5),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("MetaPortPc5",
                          "UDP port for metadata on PC5 path",
                          UintegerValue(5001),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_metaPortPc5),
                          MakeUintegerChecker<uint16_t>())
            // ── Path control ─────────────────────────────────────────────────
            .AddAttribute("PathWeightUu",
                          "Probability [0,1] of sending each frame via Uu. "
                          "1.0=Uu only, 0.0=PC5 only, (0,1)=stochastic split. "
                          "Set at runtime by TiL advisory steering.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&EdgeVideoMetadataApp::m_pathWeightUu),
                          MakeDoubleChecker<double>(0.0, 1.0))
            // ── Common ───────────────────────────────────────────────────────
            .AddAttribute("VehicleId",
                          "Logical vehicle identifier (embedded in every header)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_vehicleId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("FPS",
                          "Frames per second",
                          UintegerValue(15),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_fps),
                          MakeUintegerChecker<uint32_t>(1, 120))
            .AddAttribute("AvgFrameSize",
                          "Average P-frame payload size in bytes (25000 = 720p H.264 ~3 Mbps)",
                          UintegerValue(25000),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_avgFrameSize),
                          MakeUintegerChecker<uint32_t>(100, 10 * 1024 * 1024))
            .AddAttribute("KeyframeInterval",
                          "Number of frames between consecutive I-frames",
                          UintegerValue(30),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_keyframeInterval),
                          MakeUintegerChecker<uint32_t>(1, 600))
            .AddAttribute("KeyframeMultiplier",
                          "I-frame size = AvgFrameSize * KeyframeMultiplier",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&EdgeVideoMetadataApp::m_keyframeMultiplier),
                          MakeDoubleChecker<double>(1.0, 20.0))
            .AddAttribute("MetaPayloadSize",
                          "Bytes of dummy padding appended after MetaPacketHeader",
                          UintegerValue(512),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_metaPayloadSize),
                          MakeUintegerChecker<uint32_t>(0, 65000))
            .AddAttribute("ChunkSize",
                          "Maximum UDP payload bytes per video chunk datagram",
                          UintegerValue(1200),
                          MakeUintegerAccessor(&EdgeVideoMetadataApp::m_chunkSize),
                          MakeUintegerChecker<uint32_t>(100, 65000));
    return tid;
}

EdgeVideoMetadataApp::EdgeVideoMetadataApp()
    : m_jitterRng(CreateObject<NormalRandomVariable>()),
      m_pathRng(CreateObject<UniformRandomVariable>())
{
    // σ = 0.1 → ~10% standard deviation in frame size; clamp at ±30%
    m_jitterRng->SetAttribute("Mean",     DoubleValue(0.0));
    m_jitterRng->SetAttribute("Variance", DoubleValue(0.01));
}

EdgeVideoMetadataApp::~EdgeVideoMetadataApp()
{
}

// ---------------------------------------------------------------------------
// Helper: open a connected UDP socket, return nullptr if addr is 0.0.0.0
// ---------------------------------------------------------------------------
static Ptr<Socket>
MakeUdpTx(Ptr<Node> node, Ipv4Address addr, uint16_t port)
{
    if (addr == Ipv4Address("0.0.0.0"))
        return nullptr;
    Ptr<Socket> sock = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
    sock->Connect(InetSocketAddress(addr, port));
    return sock;
}

void
EdgeVideoMetadataApp::StartApplication()
{
    m_videoSocketUu  = MakeUdpTx(GetNode(), m_serverAddrUu,  m_videoPortUu);
    m_metaSocketUu   = MakeUdpTx(GetNode(), m_serverAddrUu,  m_metaPortUu);
    m_videoSocketPc5 = MakeUdpTx(GetNode(), m_serverAddrPc5, m_videoPortPc5);
    m_metaSocketPc5  = MakeUdpTx(GetNode(), m_serverAddrPc5, m_metaPortPc5);

    NS_LOG_INFO("Veh[" << m_vehicleId << "] EdgeVideoMetadataApp start"
                << " Uu=" << m_serverAddrUu
                << " PC5=" << m_serverAddrPc5
                << " weight_uu=" << m_pathWeightUu);

    m_frameEvent = Simulator::ScheduleNow(&EdgeVideoMetadataApp::SendFrame, this);
}

void
EdgeVideoMetadataApp::StopApplication()
{
    m_frameEvent.Cancel();
    for (auto* s : {&m_videoSocketUu, &m_metaSocketUu,
                    &m_videoSocketPc5, &m_metaSocketPc5})
    {
        if (*s) { (*s)->Close(); *s = nullptr; }
    }
}

// ---------------------------------------------------------------------------
// SendFrame — path selected per-frame via PathWeightUu
// ---------------------------------------------------------------------------
void
EdgeVideoMetadataApp::SendFrame()
{
    ++m_frameId;

    const bool   isKeyframe = ((m_frameId % m_keyframeInterval) == 0);
    const double jitter     = 1.0 + std::clamp(m_jitterRng->GetValue(), -0.3, 0.3);
    const double baseSize   = isKeyframe
                                  ? static_cast<double>(m_avgFrameSize) * m_keyframeMultiplier
                                  : static_cast<double>(m_avgFrameSize);
    const uint32_t frameSize   = static_cast<uint32_t>(baseSize * jitter);
    const uint32_t totalChunks = (frameSize + m_chunkSize - 1) / m_chunkSize;
    const double   sendTime    = Simulator::Now().GetSeconds();

    // ── Path selection ────────────────────────────────────────────────────────
    // Stochastic per-frame: use Uu with probability PathWeightUu.
    // Falls back to the available path if one socket is null.
    bool useUu;
    if (!m_videoSocketUu)
        useUu = false;
    else if (!m_videoSocketPc5)
        useUu = true;
    else
        useUu = (m_pathRng->GetValue() < m_pathWeightUu);

    Ptr<Socket> videoSock = useUu ? m_videoSocketUu  : m_videoSocketPc5;
    Ptr<Socket> metaSock  = useUu ? m_metaSocketUu   : m_metaSocketPc5;

    NS_LOG_DEBUG("Veh[" << m_vehicleId << "] Frame " << m_frameId
                        << (isKeyframe ? "(I)" : "(P)")
                        << " size=" << frameSize << "B chunks=" << totalChunks
                        << " path=" << (useUu ? "Uu" : "PC5"));

    // ── Video chunks ──────────────────────────────────────────────────────────
    for (uint32_t c = 0; c < totalChunks; ++c)
    {
        const uint32_t bytesSoFar = c * m_chunkSize;
        const uint16_t payload =
            static_cast<uint16_t>(std::min(m_chunkSize, frameSize - bytesSoFar));

        VideoPacketHeader hdr;
        hdr.common.vehicleId   = m_vehicleId;
        hdr.common.sendTimeSec = sendTime;
        hdr.common.appType     = V2X_APP_VIDEO;
        hdr.frameId            = m_frameId;
        hdr.chunkId            = static_cast<uint16_t>(c);
        hdr.totalChunks        = static_cast<uint16_t>(totalChunks);
        hdr.frameType          = isKeyframe ? 1 : 0;
        hdr.payloadSize        = payload;
        hdr.pathFlag           = useUu ? 1 : 0;

        Ptr<Packet> pkt = Create<Packet>(payload);
        pkt->AddHeader(hdr);
        videoSock->Send(pkt);
    }

    // ── Metadata packet (same path as video) ─────────────────────────────────
    {
        Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
        Vector             pos = mob->GetPosition();
        Vector             vel = mob->GetVelocity();
        const float spd = static_cast<float>(std::sqrt(vel.x * vel.x + vel.y * vel.y));
        const float hdg = static_cast<float>(std::atan2(vel.y, vel.x) * 180.0 / M_PI);

        MetaPacketHeader mhdr;
        mhdr.common.vehicleId   = m_vehicleId;
        mhdr.common.sendTimeSec = sendTime;
        mhdr.common.appType     = V2X_APP_META;
        mhdr.frameId            = m_frameId;
        mhdr.latitude           = pos.x;
        mhdr.longitude          = pos.y;
        mhdr.heading            = hdg;
        mhdr.speed              = spd;
        mhdr.cpuLoad            = 0.5f;
        mhdr.memUsage           = 512.0f;

        Ptr<Packet> mpkt = Create<Packet>(m_metaPayloadSize);
        mpkt->AddHeader(mhdr);
        metaSock->Send(mpkt);
    }

    m_frameEvent =
        Simulator::Schedule(Seconds(1.0 / static_cast<double>(m_fps)),
                            &EdgeVideoMetadataApp::SendFrame, this);
}

} // namespace ns3
