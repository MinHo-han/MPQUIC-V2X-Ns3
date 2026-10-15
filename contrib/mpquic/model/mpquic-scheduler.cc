#include "mpquic-scheduler.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MpQuicScheduler");

// ---------------------------------------------------------------------------
// MpQuicScheduler (abstract)
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(MpQuicScheduler);

TypeId
MpQuicScheduler::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MpQuicScheduler").SetParent<Object>().SetGroupName("MpQuic");
    return tid;
}

// ---------------------------------------------------------------------------
// RoundRobin
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(MpQuicRoundRobinScheduler);

TypeId
MpQuicRoundRobinScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MpQuicRoundRobinScheduler")
                            .SetParent<MpQuicScheduler>()
                            .SetGroupName("MpQuic")
                            .AddConstructor<MpQuicRoundRobinScheduler>();
    return tid;
}

Ptr<MpQuicPath>
MpQuicRoundRobinScheduler::SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                       const MpQuicStreamInfo& /*info*/)
{
    // Advisory active AND d≠0: credit-based weighted split (Bresenham).
    // d=0 or advisory inactive: original m_lastIndex cycling (baseline-equivalent).
    if (m_advActive && std::abs(m_advD) > 1e-6)
    {
        // Identify Uu and PC5 paths (pathId 0=Uu by convention).
        Ptr<MpQuicPath> uuPath  = nullptr;
        Ptr<MpQuicPath> pc5Path = nullptr;
        for (const auto& p : paths)
        {
            if (!p->IsActive()) continue;
            if (p->GetPathId() == 0) uuPath  = p;
            else                     pc5Path = p;
        }
        if (!uuPath && !pc5Path) return nullptr;
        if (!uuPath)  return pc5Path;
        if (!pc5Path) return uuPath;

        m_advCredit += m_advWeight;
        if (m_advCredit >= 1.0)
        {
            m_advCredit -= 1.0;
            NS_LOG_DEBUG("RoundRobin(adv) -> Uu  credit=" << m_advCredit
                         << " weight=" << m_advWeight);
            return uuPath;
        }
        NS_LOG_DEBUG("RoundRobin(adv) -> PC5 credit=" << m_advCredit
                     << " weight=" << m_advWeight);
        return pc5Path;
    }

    // Original RR: m_lastIndex cycling.
    const uint32_t n = paths.size();
    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t idx = (m_lastIndex + i) % n;
        if (paths[idx]->IsActive())
        {
            m_lastIndex = (idx + 1) % n;
            NS_LOG_DEBUG("RoundRobin -> path " << static_cast<int>(paths[idx]->GetPathId())
                                               << " [" << paths[idx]->GetPathLabel() << "]");
            return paths[idx];
        }
    }
    return nullptr;
}

void
MpQuicRoundRobinScheduler::ApplyAdvisory(double wAdv, double wBase)
{
    m_advWeight = std::max(0.0, std::min(1.0, wAdv));
    m_advD      = wAdv - wBase;
    if (!m_advActive && std::abs(m_advD) > 1e-6)
    {
        // First real activation: reset credit so first advisory packet is deterministic.
        m_advCredit = 0.0;
        m_advActive = true;
    }
    else if (std::abs(m_advD) <= 1e-6)
    {
        // d≈0: deactivate advisory so SelectPath uses original index-RR (baseline equivalent).
        m_advActive = false;
    }
}

void
MpQuicRoundRobinScheduler::ClearAdvisory()
{
    m_advActive = false;
    // m_lastIndex is preserved; RR resumes from where it was before advisory.
}

// ---------------------------------------------------------------------------
// MinRTT
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(MpQuicMinRttScheduler);

TypeId
MpQuicMinRttScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MpQuicMinRttScheduler")
                            .SetParent<MpQuicScheduler>()
                            .SetGroupName("MpQuic")
                            .AddConstructor<MpQuicMinRttScheduler>();
    return tid;
}

Ptr<MpQuicPath>
MpQuicMinRttScheduler::SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                   const MpQuicStreamInfo& /*info*/)
{
    Ptr<MpQuicPath> best        = nullptr;
    double          bestEffRtt  = std::numeric_limits<double>::max();

    for (const auto& p : paths)
    {
        if (!p->IsActive()) continue;
        double rtt = p->GetRtt().GetMilliSeconds();
        if (rtt < 0.01) rtt = 0.01;

        // Advisory: apply effective-RTT bias while preserving actual RTT.
        // prefer ∈ [-1,+1]: +1 = prefer Uu (pathId 0), -1 = prefer PC5.
        // effective_rtt = rtt * (1 - kAdvGain * prefer * pathSign)
        //   pathSign = +1 for Uu, -1 for PC5
        // When m_advPrefer=0 (OFF or d=0): no change.
        double effRtt = rtt;
        if (m_advActive && m_advPrefer != 0.0)
        {
            double pathSign = (p->GetPathId() == 0) ? 1.0 : -1.0;
            double factor   = 1.0 - kAdvGain * m_advPrefer * pathSign;
            factor  = std::max(0.01, factor);  // effective RTT stays positive
            effRtt  = rtt * factor;
        }

        if (effRtt < bestEffRtt)
        {
            bestEffRtt = effRtt;
            best       = p;
        }
    }
    if (best)
    {
        NS_LOG_DEBUG("MinRTT" << (m_advActive ? "(adv)" : "")
                              << " -> path " << static_cast<int>(best->GetPathId())
                              << " [" << best->GetPathLabel()
                              << "] RTT=" << best->GetRtt().GetMilliSeconds() << " ms"
                              << " effRTT=" << bestEffRtt << " prefer=" << m_advPrefer);
    }
    return best;
}

void
MpQuicMinRttScheduler::ApplyAdvisory(double wAdv, double wBase)
{
    double d    = wAdv - wBase;
    // Normalize to [-1,+1]; kMaxD = max expected |d| (= ε, default 0.30).
    m_advPrefer = std::max(-1.0, std::min(1.0, d / kMaxD));
    m_advActive = true;
}

void
MpQuicMinRttScheduler::ClearAdvisory()
{
    m_advActive = false;
    m_advPrefer = 0.0;
}

void
MpQuicMinRttScheduler::ApplyBinaryAdvisory(bool suppressUu, bool suppressPc5, double /*wBase*/)
{
    if (suppressUu && !suppressPc5)       { m_advPrefer = -1.0; m_advActive = true; }
    else if (!suppressUu && suppressPc5)  { m_advPrefer = +1.0; m_advActive = true; }
    else                                  { ClearAdvisory(); }
}

// ---------------------------------------------------------------------------
// Redundant
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(MpQuicRedundantScheduler);

TypeId
MpQuicRedundantScheduler::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MpQuicRedundantScheduler")
                            .SetParent<MpQuicScheduler>()
                            .SetGroupName("MpQuic")
                            .AddConstructor<MpQuicRedundantScheduler>();
    return tid;
}

Ptr<MpQuicPath>
MpQuicRedundantScheduler::SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                      const MpQuicStreamInfo& /*info*/)
{
    // MpQuicSocketBase checks IsRedundant() and iterates all paths itself.
    // Return the first active path as a fallback reference.
    for (const auto& p : paths)
    {
        if (p->IsActive())
        {
            return p;
        }
    }
    return nullptr;
}

void
MpQuicRedundantScheduler::ApplyAdvisory(double wAdv, double wBase)
{
    m_advD      = wAdv - wBase;
    m_advActive = true;
}

void
MpQuicRedundantScheduler::ClearAdvisory()
{
    m_advActive = false;
    m_advD      = 0.0;
    m_redCredit = 0.0;  // reset credit so next advisory starts fresh
}

void
MpQuicRedundantScheduler::ApplyBinaryAdvisory(bool suppressUu, bool suppressPc5, double /*wBase*/)
{
    m_redCredit = 0.0;
    if (suppressUu && !suppressPc5)
    {
        // Force PC5 only: d = -2.0 → q = max(0, 1 - kAlpha*2.0) = 0 → no Uu replication
        m_advD      = -2.0;
        m_advActive = true;
    }
    else if (!suppressUu && suppressPc5)
    {
        // Force Uu only: d = +2.0 → q = max(0, 1 - kAlpha*2.0) = 0 → no PC5 replication
        m_advD      = +2.0;
        m_advActive = true;
    }
    else
    {
        // Both risky or neither: full Redundant (best effort)
        ClearAdvisory();
    }
}

std::unordered_map<uint8_t, bool>
MpQuicRedundantScheduler::ComputeReplicationMap(const std::vector<Ptr<MpQuicPath>>& paths)
{
    std::unordered_map<uint8_t, bool> result;
    if (!m_advActive || m_advD == 0.0)
    {
        // Advisory OFF or zero adjustment: replicate to all paths (full Redundant).
        for (const auto& p : paths) result[p->GetPathId()] = true;
        return result;
    }

    // Preferred path: pathId 0 (Uu) when d>0, pathId 1 (PC5) when d<0.
    uint8_t preferredId    = (m_advD > 0.0) ? 0 : 1;
    uint8_t lessPreferredId = (m_advD > 0.0) ? 1 : 0;

    // Replication ratio for less-preferred path: q = clip(1 - kAlpha*|d|, 0, 1).
    double q = std::max(0.0, std::min(1.0, 1.0 - kAlpha * std::abs(m_advD)));

    // Preferred path: always replicate.
    result[preferredId] = true;

    // Less-preferred path: Bresenham credit accumulation.
    m_redCredit += q;
    if (m_redCredit >= 1.0)
    {
        m_redCredit -= 1.0;
        result[lessPreferredId] = true;
    }
    else
    {
        result[lessPreferredId] = false;
    }

    // Safety: if only one path exists in paths[], override to true.
    // (Actual single-path fallback is enforced in SocketBase, not here.)
    return result;
}


} // namespace ns3
