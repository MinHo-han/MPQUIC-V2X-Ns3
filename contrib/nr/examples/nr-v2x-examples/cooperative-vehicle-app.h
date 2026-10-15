// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file cooperative-vehicle-app.h
 * @brief Cooperative normal-vehicle application for V2X remote-support simulation.
 *
 * Sends two message types to the edge server via UDP (port 5002):
 *   - AwarenessMessageHeader : periodic (default 100 ms, i.e. 10 Hz CAM-like)
 *   - HazardEventHeader      : event-driven (Poisson, default ~0.01 events/s)
 *
 * Position and velocity are read from the node's attached MobilityModel.
 */
#pragma once

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"

namespace ns3
{

class CooperativeVehicleApp : public Application
{
  public:
    static TypeId GetTypeId();
    CooperativeVehicleApp();
    ~CooperativeVehicleApp() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    void SendAwareness();
    void CheckHazard();

    // ── Attributes ──────────────────────────────────────────────────────────
    Ipv4Address m_serverAddr;
    uint16_t    m_serverPort{5002};
    uint32_t    m_vehicleId{0};
    Time        m_awarenessInterval{MilliSeconds(100)};
    double      m_hazardProbPerSec{0.01};

    // ── State ────────────────────────────────────────────────────────────────
    Ptr<Socket>                m_socket;
    EventId                    m_awarenessEvent;
    EventId                    m_hazardEvent;
    Ptr<UniformRandomVariable> m_rng;
};

} // namespace ns3
