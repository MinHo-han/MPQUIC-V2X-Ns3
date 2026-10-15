#ifndef MPQUIC_HELPER_H
#define MPQUIC_HELPER_H

#include "../model/mpquic-scheduler.h"
#include "../model/mpquic-socket-base.h"

#include "ns3/inet-socket-address.h"
#include "ns3/node.h"
#include "ns3/ptr.h"

namespace ns3
{

/**
 * \ingroup mpquic
 *
 * Convenience helper to build a MpQuicSocketBase with a chosen scheduler
 * and pre-configured paths.
 *
 * Example (two-path vehicle scenario):
 * \code
 *   MpQuicHelper helper;
 *   auto mpquic = helper.InstallWithTwoPaths(
 *       vehicleNode,
 *       InetSocketAddress(remoteHostAddr,  uuPort),
 *       InetSocketAddress(remotePc5Addr,   pc5Port),
 *       MpQuicHelper::SchedulerType::MinRtt);
 *   mpquic->Send(myPacket);
 * \endcode
 */
class MpQuicHelper
{
  public:
    enum class SchedulerType
    {
        RoundRobin,   ///< Equal split, cycles through active paths
        MinRtt,       ///< Always use the path with the lowest RTT
        Redundant,    ///< Copy every packet to ALL active paths
    };

    MpQuicHelper() = default;

    /**
     * Install a bare MpQuicSocketBase on node with the given scheduler.
     * Call AddPath() on the returned object to add paths manually.
     */
    Ptr<MpQuicSocketBase> Install(Ptr<Node>     node,
                                   SchedulerType sched = SchedulerType::RoundRobin) const;

    /**
     * Install and configure two paths in one call (path 0 = Uu, path 1 = PC5).
     */
    Ptr<MpQuicSocketBase> InstallWithTwoPaths(
        Ptr<Node>         node,
        InetSocketAddress uuRemote,
        InetSocketAddress pc5Remote,
        SchedulerType     sched        = SchedulerType::RoundRobin,
        Ipv4Address       pc5LocalAddr = Ipv4Address::GetAny()) const;

  private:
    Ptr<MpQuicScheduler> CreateScheduler(SchedulerType type) const;
};

} // namespace ns3
#endif // MPQUIC_HELPER_H
