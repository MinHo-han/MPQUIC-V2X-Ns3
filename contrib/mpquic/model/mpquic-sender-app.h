#ifndef MPQUIC_SENDER_APP_H
#define MPQUIC_SENDER_APP_H

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
 * Application that generates a CBR UDP stream and forwards each packet
 * through a MpQuicSocketBase.  The configured scheduler decides which
 * physical path (Uu / C-V2X PC5) carries each packet.
 *
 * Usage:
 * \code
 *   auto app = CreateObject<MpQuicSenderApp>();
 *   app->SetMpQuic(mpquic);          // pre-configured MpQuicSocketBase
 *   app->SetPacketSize(1200);
 *   app->SetInterval(Seconds(1.0 / 5000.0));
 *   node->AddApplication(app);
 *   app->SetStartTime(appStart);
 *   app->SetStopTime(simTime);
 * \endcode
 */
class MpQuicSenderApp : public Application
{
  public:
    static TypeId GetTypeId();

    MpQuicSenderApp();
    ~MpQuicSenderApp() override;

    void SetMpQuic(Ptr<MpQuicSocketBase> mpquic);
    void SetPacketSize(uint32_t bytes);
    void SetInterval(Time interval);
    /// Assign a stream ID within this connection (default 0 = single data stream).
    void SetStreamId(uint8_t id);

    /// Convenience: set the connection ID on the underlying MpQuicSocketBase.
    /// Equivalent to calling mpquic->SetConnId(id) directly.
    void SetConnId(uint32_t id);

    uint64_t GetTotalPacketsSent() const;

  private:
    void StartApplication() override;
    void StopApplication() override;
    void SendPacket();

    Ptr<MpQuicSocketBase> m_mpquic;
    uint32_t              m_pktSize{1200};
    Time                  m_interval{MilliSeconds(1)};
    EventId               m_sendEvent;
    bool                  m_running{false};
    uint64_t              m_pktCount{0};
    uint8_t               m_streamId{0};   ///< stream ID passed to MpQuicStreamInfo
};

} // namespace ns3
#endif // MPQUIC_SENDER_APP_H
