#ifndef MPQUIC_INFERENCE_SENDER_APP_H
#define MPQUIC_INFERENCE_SENDER_APP_H

#include "mpquic-socket-base.h"

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

namespace ns3
{

/**
 * \ingroup mpquic
 *
 * AI inference result sender application for MPQUIC.
 *
 * Simulates the output of an object detection model (e.g. YOLO) running at
 * a fixed frame rate.  Each inference result is one UDP payload whose size
 * varies uniformly between MinResultBytes and MaxResultBytes, reflecting the
 * variable number of detected objects (bounding boxes, confidence scores).
 *
 * All packets carry:
 *   streamId = 1  (inference stream, distinct from video stream 0)
 *   priority = 1  (high — latency-sensitive control data)
 *   deadline      = configured deadline (default 100 ms)
 *
 * The MprDeadlineScheduler uses the deadline to decide feasibility; if the
 * inferred latency exceeds the deadline the packet is dropped.
 *
 * Default parameters: 30 fps, result payload 2–8 KB, deadline 100 ms.
 *
 * Usage:
 * \code
 *   auto app = CreateObject<MpQuicInferenceSenderApp>();
 *   app->SetMpQuic(mpquic);
 *   node->AddApplication(app);
 *   app->SetStartTime(start);
 *   app->SetStopTime(stop);
 * \endcode
 */
class MpQuicInferenceSenderApp : public Application
{
  public:
    static TypeId GetTypeId();

    MpQuicInferenceSenderApp();
    ~MpQuicInferenceSenderApp() override;

    void SetMpQuic(Ptr<MpQuicSocketBase> mpquic);

    uint64_t GetTotalPacketsSent() const;

  private:
    void StartApplication() override;
    void StopApplication()  override;
    void SendResult();

    Ptr<MpQuicSocketBase> m_mpquic;

    uint32_t m_minResultBytes{2000};  ///< min inference result payload (bytes)
    uint32_t m_maxResultBytes{8000};  ///< max inference result payload (bytes)
    uint32_t m_pktSize{1200};         ///< fragment MTU (bytes)
    double   m_fps{30.0};             ///< inference rate (results/sec)
    Time     m_deadline{MilliSeconds(100)}; ///< per-result delivery deadline
    uint8_t  m_streamId{1};           ///< MPQUIC stream 1 = inference

    Ptr<UniformRandomVariable> m_rng;

    uint64_t m_pktCount{0};
    bool     m_running{false};
    EventId  m_sendEvent;
};

} // namespace ns3
#endif // MPQUIC_INFERENCE_SENDER_APP_H
