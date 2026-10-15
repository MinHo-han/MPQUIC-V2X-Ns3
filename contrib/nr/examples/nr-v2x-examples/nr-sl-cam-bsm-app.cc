// SPDX-License-Identifier: GPL-2.0-only
#include "nr-sl-cam-bsm-app.h"

#include "ns3/inet-socket-address.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"

NS_LOG_COMPONENT_DEFINE("NrSlCamBsmApp");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(NrSlCamBsmApp);

TypeId
NrSlCamBsmApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrSlCamBsmApp")
            .SetParent<Application>()
            .SetGroupName("Nr")
            .AddConstructor<NrSlCamBsmApp>()
            .AddAttribute("CamInterval", "CAM broadcast interval",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrSlCamBsmApp::m_camInterval),
                          MakeTimeChecker())
            .AddAttribute("BsmInterval", "BSM broadcast interval",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NrSlCamBsmApp::m_bsmInterval),
                          MakeTimeChecker())
            .AddAttribute("CamSize", "CAM payload bytes (ETSI EN 302 637-2)",
                          UintegerValue(300),
                          MakeUintegerAccessor(&NrSlCamBsmApp::m_camSize),
                          MakeUintegerChecker<uint32_t>(1, 1500))
            .AddAttribute("BsmSize", "BSM payload bytes (SAE J2735)",
                          UintegerValue(200),
                          MakeUintegerAccessor(&NrSlCamBsmApp::m_bsmSize),
                          MakeUintegerChecker<uint32_t>(1, 1500))
            .AddAttribute("Port", "UDP port for V2X broadcast",
                          UintegerValue(9999),
                          MakeUintegerAccessor(&NrSlCamBsmApp::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("BsmEnabled", "Enable BSM broadcasts (false = CAM-only)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrSlCamBsmApp::m_bsmEnabled),
                          MakeBooleanChecker());
    return tid;
}

NrSlCamBsmApp::NrSlCamBsmApp() = default;

void
NrSlCamBsmApp::StartApplication()
{
    NS_ABORT_MSG_IF(m_slAddr == Ipv4Address(),
                    "NrSlCamBsmApp: SetSlAddress() must be called before Start");

    // CAM socket — broadcast on SL interface
    m_camSocket = Socket::CreateSocket(GetNode(),
                                       UdpSocketFactory::GetTypeId());
    m_camSocket->SetAllowBroadcast(true);
    m_camSocket->Bind(InetSocketAddress(m_slAddr, m_port));
    m_camSocket->Connect(InetSocketAddress(Ipv4Address("255.255.255.255"), m_port));

    // BSM socket — same interface, different port to distinguish traffic
    m_bsmSocket = Socket::CreateSocket(GetNode(),
                                       UdpSocketFactory::GetTypeId());
    m_bsmSocket->SetAllowBroadcast(true);
    m_bsmSocket->Bind(InetSocketAddress(m_slAddr, m_port + 1));
    m_bsmSocket->Connect(InetSocketAddress(Ipv4Address("255.255.255.255"), m_port + 1));

    m_camEvent = Simulator::Schedule(m_camInterval, &NrSlCamBsmApp::SendCam, this);
    if (m_bsmEnabled)
        m_bsmEvent = Simulator::Schedule(m_bsmInterval + MilliSeconds(50), &NrSlCamBsmApp::SendBsm, this);
}

void
NrSlCamBsmApp::StopApplication()
{
    m_camEvent.Cancel();
    m_bsmEvent.Cancel();
    if (m_camSocket) { m_camSocket->Close(); m_camSocket = nullptr; }
    if (m_bsmSocket) { m_bsmSocket->Close(); m_bsmSocket = nullptr; }
}

void
NrSlCamBsmApp::SendCam()
{
    Ptr<Packet> pkt = Create<Packet>(m_camSize);
    int sent = m_camSocket->Send(pkt);
    if (sent > 0)
        ++m_camTxCount;
    NS_LOG_DEBUG("[CAM] node=" << GetNode()->GetId()
                << " t=" << Simulator::Now().GetSeconds()
                << " size=" << m_camSize << " total=" << m_camTxCount);
    m_camEvent = Simulator::Schedule(m_camInterval, &NrSlCamBsmApp::SendCam, this);
}

void
NrSlCamBsmApp::SendBsm()
{
    Ptr<Packet> pkt = Create<Packet>(m_bsmSize);
    int sent = m_bsmSocket->Send(pkt);
    if (sent > 0)
        ++m_bsmTxCount;
    NS_LOG_DEBUG("[BSM] node=" << GetNode()->GetId()
                << " t=" << Simulator::Now().GetSeconds()
                << " size=" << m_bsmSize << " total=" << m_bsmTxCount);
    m_bsmEvent = Simulator::Schedule(m_bsmInterval, &NrSlCamBsmApp::SendBsm, this);
}

} // namespace ns3
