// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file edge-video-metadata-app.h
 * @brief Remote-support vehicle application: video uplink + metadata stream.
 *
 * At each frame interval (1/FPS seconds):
 *   1. Determines frame type: I-frame every KeyframeInterval frames, else P-frame.
 *   2. Computes frame size with ±10% Gaussian jitter:
 *        P-frame: AvgFrameSize * jitter
 *        I-frame: AvgFrameSize * KeyframeMultiplier * jitter
 *   3. Fragments frame into ceil(frameSize / ChunkSize) UDP datagrams, each
 *      carrying a VideoPacketHeader + dummy payload → sent to ServerAddress:VideoPort.
 *   4. Sends one MetaPacketHeader (vehicle position/speed/cpuLoad) as a separate
 *      UDP datagram → sent to ServerAddress:MetaPort.
 *
 * Ports (defaults): VideoPort=5000, MetaPort=5001.
 */
#pragma once

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"

namespace ns3
{

class EdgeVideoMetadataApp : public Application
{
  public:
    static TypeId GetTypeId();
    EdgeVideoMetadataApp();
    ~EdgeVideoMetadataApp() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    void SendFrame();

    // ── Attributes ──────────────────────────────────────────────────────────
    // Uu path
    Ipv4Address m_serverAddrUu;
    uint16_t    m_videoPortUu{5000};
    uint16_t    m_metaPortUu{5001};
    // PC5 path
    Ipv4Address m_serverAddrPc5;
    uint16_t    m_videoPortPc5{5000};
    uint16_t    m_metaPortPc5{5001};
    // Path control
    // PathWeightUu ∈ [0,1]: probability of sending a frame via Uu
    //   1.0 = Uu only, 0.0 = PC5 only, 0.5 = equal split
    double      m_pathWeightUu{1.0};
    // Common
    uint32_t    m_vehicleId{0};
    uint32_t    m_fps{15};
    uint32_t    m_avgFrameSize{25000};      // bytes  (25 KB for P-frames, 720p H.264 ~3 Mbps)
    uint32_t    m_keyframeInterval{30};     // frames between I-frames
    double      m_keyframeMultiplier{3.0};  // I-frame = avgFrameSize * multiplier
    uint32_t    m_metaPayloadSize{512};     // bytes of dummy padding after MetaPacketHeader
    uint32_t    m_chunkSize{1200};          // max UDP payload per video chunk (bytes)

    // ── State ────────────────────────────────────────────────────────────────
    Ptr<Socket>                m_videoSocketUu;
    Ptr<Socket>                m_metaSocketUu;
    Ptr<Socket>                m_videoSocketPc5;
    Ptr<Socket>                m_metaSocketPc5;
    EventId                    m_frameEvent;
    Ptr<NormalRandomVariable>  m_jitterRng;  // mean=0, stddev=0.1 → ±10% frame size
    Ptr<UniformRandomVariable> m_pathRng;    // for stochastic path selection
    uint32_t                   m_frameId{0};
};

} // namespace ns3
