// SPDX-License-Identifier: GPL-2.0-only
#include "cooperative-vehicle-app.h"

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

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("CooperativeVehicleApp");
NS_OBJECT_ENSURE_REGISTERED(CooperativeVehicleApp);

TypeId
CooperativeVehicleApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::CooperativeVehicleApp")
            .SetParent<Application>()
            .SetGroupName("V2xRemoteSupport")
            .AddConstructor<CooperativeVehicleApp>()
            .AddAttribute("ServerAddress",
                          "Remote server IPv4 address",
                          Ipv4AddressValue(Ipv4Address("1.0.0.2")),
                          MakeIpv4AddressAccessor(&CooperativeVehicleApp::m_serverAddr),
                          MakeIpv4AddressChecker())
            .AddAttribute("ServerPort",
                          "UDP port for cooperative traffic (awareness + hazard)",
                          UintegerValue(5002),
                          MakeUintegerAccessor(&CooperativeVehicleApp::m_serverPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("VehicleId",
                          "Logical vehicle identifier (embedded in every header)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&CooperativeVehicleApp::m_vehicleId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("AwarenessInterval",
                          "Period between awareness messages",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&CooperativeVehicleApp::m_awarenessInterval),
                          MakeTimeChecker())
            .AddAttribute("HazardProbPerSec",
                          "Probability of generating a hazard event each second",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&CooperativeVehicleApp::m_hazardProbPerSec),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

CooperativeVehicleApp::CooperativeVehicleApp()
    : m_rng(CreateObject<UniformRandomVariable>())
{
}

CooperativeVehicleApp::~CooperativeVehicleApp()
{
}

void
CooperativeVehicleApp::StartApplication()
{
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_socket->Connect(InetSocketAddress(m_serverAddr, m_serverPort));

    // First awareness immediately; hazard check every second
    m_awarenessEvent =
        Simulator::ScheduleNow(&CooperativeVehicleApp::SendAwareness, this);
    m_hazardEvent =
        Simulator::Schedule(Seconds(1.0), &CooperativeVehicleApp::CheckHazard, this);
}

void
CooperativeVehicleApp::StopApplication()
{
    m_awarenessEvent.Cancel();
    m_hazardEvent.Cancel();
    if (m_socket)
    {
        m_socket->Close();
        m_socket = nullptr;
    }
}

// ── Periodic awareness ────────────────────────────────────────────────────────
void
CooperativeVehicleApp::SendAwareness()
{
    Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
    Vector             pos = mob->GetPosition();
    Vector             vel = mob->GetVelocity();
    const float spd = static_cast<float>(std::sqrt(vel.x * vel.x + vel.y * vel.y));
    const float hdg = static_cast<float>(std::atan2(vel.y, vel.x) * 180.0 / M_PI);

    AwarenessMessageHeader hdr;
    hdr.common.vehicleId   = m_vehicleId;
    hdr.common.sendTimeSec = Simulator::Now().GetSeconds();
    hdr.common.appType     = V2X_APP_AWARENESS;
    hdr.latitude           = pos.x;
    hdr.longitude          = pos.y;
    hdr.speedMs            = spd;
    hdr.headingDeg         = hdg;
    hdr.accelMs2           = 0.0f; // not tracked in waypoint model

    Ptr<Packet> pkt = Create<Packet>(0);
    pkt->AddHeader(hdr);
    m_socket->Send(pkt);

    NS_LOG_DEBUG("Veh[" << m_vehicleId << "] Awareness pos=(" << pos.x << "," << pos.y
                        << ") v=" << spd << " t=" << hdr.common.sendTimeSec);

    m_awarenessEvent =
        Simulator::Schedule(m_awarenessInterval, &CooperativeVehicleApp::SendAwareness, this);
}

// ── Event-driven hazard ───────────────────────────────────────────────────────
void
CooperativeVehicleApp::CheckHazard()
{
    if (m_rng->GetValue(0.0, 1.0) < m_hazardProbPerSec)
    {
        Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
        Vector             pos = mob->GetPosition();

        HazardEventHeader hdr;
        hdr.common.vehicleId   = m_vehicleId;
        hdr.common.sendTimeSec = Simulator::Now().GetSeconds();
        hdr.common.appType     = V2X_APP_HAZARD;
        hdr.hazardType         = static_cast<uint8_t>(m_rng->GetInteger(0, 3));
        hdr.latitude           = pos.x;
        hdr.longitude          = pos.y;
        hdr.severity           = static_cast<uint8_t>(m_rng->GetInteger(0, 3));

        Ptr<Packet> pkt = Create<Packet>(0);
        pkt->AddHeader(hdr);
        m_socket->Send(pkt);

        NS_LOG_INFO("Veh[" << m_vehicleId << "] HazardEvent type=" << (int)hdr.hazardType
                           << " sev=" << (int)hdr.severity << " t=" << hdr.common.sendTimeSec);
    }

    m_hazardEvent =
        Simulator::Schedule(Seconds(1.0), &CooperativeVehicleApp::CheckHazard, this);
}

} // namespace ns3
