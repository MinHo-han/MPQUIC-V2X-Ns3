#ifndef MPQUIC_VIDEO_SENDER_APP_H
#define MPQUIC_VIDEO_SENDER_APP_H

#include "mpquic-socket-base.h"

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"

namespace ns3
{

/**
 * \ingroup mpquic
 *
 * Video stream sender application for MPQUIC.
 *
 * Generates H.264-like GoP (Group of Pictures) traffic with variable frame sizes:
 *   - I-frame  : large keyframe, sent every gopSize frames (high priority)
 *   - P-frame  : predicted frame, every 3rd non-I frame (normal priority)
 *   - B-frame  : bidirectional frame, all others (normal priority)
 *
 * Each frame is fragmented into 1200-byte MPQUIC packets.
 * I-frame packets carry priority=1 in MpQuicStreamInfo so that
 * MprPriorityScheduler routes them to the best path.
 *
 * Default parameters model 1080p H.264 at ~4 Mbps, 30 fps.
 *
 * Usage:
 * \code
 *   auto app = CreateObject<MpQuicVideoSenderApp>();
 *   app->SetMpQuic(mpquic);
 *   node->AddApplication(app);
 *   app->SetStartTime(start);
 *   app->SetStopTime(stop);
 * \endcode
 */
class MpQuicVideoSenderApp : public Application
{
  public:
    static TypeId GetTypeId();

    MpQuicVideoSenderApp();
    ~MpQuicVideoSenderApp() override;

    void SetMpQuic(Ptr<MpQuicSocketBase> mpquic);

    uint64_t GetTotalPacketsSent()  const;
    uint64_t GetTotalFramesSent()   const;

  private:
    void StartApplication() override;
    void StopApplication()  override;
    void SendFrame();

    Ptr<MpQuicSocketBase> m_mpquic;

    // GoP parameters
    uint32_t m_gopSize{30};       ///< I-frame every gopSize frames
    uint32_t m_iFrameBytes{40000};///< I-frame size (bytes) — ~40 KB
    uint32_t m_pFrameBytes{4000}; ///< P-frame size (bytes) — ~4 KB
    uint32_t m_bFrameBytes{1500}; ///< B-frame size (bytes) — ~1.5 KB
    uint32_t m_pktSize{1200};     ///< MTU fragment size (bytes)
    double   m_fps{30.0};         ///< frames per second

    // Stream identity
    uint8_t m_streamId{0};        ///< MPQUIC stream 0 = video

    // State
    uint64_t m_frameCount{0};
    uint64_t m_pktCount{0};
    bool     m_running{false};
    EventId  m_sendEvent;
};

} // namespace ns3
#endif // MPQUIC_VIDEO_SENDER_APP_H
