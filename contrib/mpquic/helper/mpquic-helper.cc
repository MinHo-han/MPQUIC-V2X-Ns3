#include "mpquic-helper.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicHelper");

Ptr<MpQuicSocketBase>
MpQuicHelper::Install(Ptr<Node> node, SchedulerType sched) const
{
    auto mpquic = CreateObject<MpQuicSocketBase>();
    mpquic->Setup(node, CreateScheduler(sched));
    return mpquic;
}

Ptr<MpQuicSocketBase>
MpQuicHelper::InstallWithTwoPaths(Ptr<Node>         node,
                                   InetSocketAddress uuRemote,
                                   InetSocketAddress pc5Remote,
                                   SchedulerType     sched,
                                   Ipv4Address       pc5LocalAddr) const
{
    auto mpquic = Install(node, sched);
    mpquic->AddPath(uuRemote,  MpQuicPathType::UU);
    mpquic->AddPath(pc5Remote, MpQuicPathType::PC5, pc5LocalAddr);
    return mpquic;
}

Ptr<MpQuicScheduler>
MpQuicHelper::CreateScheduler(SchedulerType type) const
{
    switch (type)
    {
    case SchedulerType::MinRtt:
        return CreateObject<MpQuicMinRttScheduler>();
    case SchedulerType::Redundant:
        return CreateObject<MpQuicRedundantScheduler>();
    case SchedulerType::RoundRobin:
    default:
        return CreateObject<MpQuicRoundRobinScheduler>();
    }
}

} // namespace ns3
