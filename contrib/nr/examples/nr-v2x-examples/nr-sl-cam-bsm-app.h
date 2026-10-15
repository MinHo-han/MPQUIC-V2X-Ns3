#pragma once
// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file nr-sl-cam-bsm-app.h
 * @brief V2X background traffic app: CAM + BSM broadcast over NR SL (PC5).
 *
 * Installed on ALL vehicles (not just senders) to model V2X broadcast load.
 * Both messages are sent as UDP broadcast on the SL interface, competing with
 * video PC5 traffic for sidelink radio resources.
 *
 *  CAM  (ETSI EN 302 637-2): 10 Hz,  300 bytes
 *  BSM  (SAE J2735):         10 Hz,  200 bytes
 */

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/socket.h"

namespace ns3
{

class NrSlCamBsmApp : public Application
{
  public:
    static TypeId GetTypeId();
    NrSlCamBsmApp();
    ~NrSlCamBsmApp() override = default;

    // Bind to this vehicle's SL interface IP (assigned by EPC)
    void SetSlAddress(Ipv4Address addr) { m_slAddr = addr; }

    uint64_t GetCamTxCount() const { return m_camTxCount; }
    uint64_t GetBsmTxCount() const { return m_bsmTxCount; }

  private:
    void StartApplication() override;
    void StopApplication() override;

    void SendCam();
    void SendBsm();

    // ── Attributes ─────────────────────────────────────────────────────────
    Time     m_camInterval{MilliSeconds(100)};  ///< CAM period (10 Hz)
    Time     m_bsmInterval{MilliSeconds(100)};  ///< BSM period (10 Hz)
    uint32_t m_camSize{300};                    ///< CAM packet bytes (ETSI)
    uint32_t m_bsmSize{200};                    ///< BSM packet bytes (SAE J2735)
    uint16_t m_port{9999};                      ///< UDP port for V2X broadcast
    bool     m_bsmEnabled{false};               ///< false = CAM-only mode (BSM disabled)

    // ── State ──────────────────────────────────────────────────────────────
    Ipv4Address m_slAddr;           ///< this vehicle's SL interface IP
    Ptr<Socket> m_camSocket;
    Ptr<Socket> m_bsmSocket;
    EventId     m_camEvent;
    EventId     m_bsmEvent;

    uint64_t m_camTxCount{0};
    uint64_t m_bsmTxCount{0};
};

} // namespace ns3
