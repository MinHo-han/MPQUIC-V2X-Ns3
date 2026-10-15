#ifndef MPQUIC_SCHEDULER_H
#define MPQUIC_SCHEDULER_H

#include "mpquic-path.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <unordered_map>
#include <vector>

namespace ns3
{

// ---------------------------------------------------------------------------
/**
 * Per-packet stream metadata passed from application to scheduler.
 *
 * Allows stream-aware path selection:
 *   streamId  — 0 = video, 1 = AI inference, …
 *   priority  — 0 = normal (P/B-frame), 1 = high (I-frame / urgent)
 *   deadline  — absolute per-packet deadline; Time::Max() = no constraint
 */
struct MpQuicStreamInfo
{
    uint8_t streamId{0};
    uint8_t priority{0};
    Time    deadline{Time::Max()};
    bool    isReliable{true};  ///< false = unreliable (Datagram): lost packets are dropped, not retransmitted (MPR-QUIC §5.1)
};

// ---------------------------------------------------------------------------

/**
 * \ingroup mpquic
 * Abstract base for all MPQUIC path schedulers.
 * Subclass and override SelectPath() to implement a custom policy.
 */
class MpQuicScheduler : public Object
{
  public:
    static TypeId GetTypeId();

    /**
     * Pick one active path for the next outgoing packet.
     * \param paths   all registered paths
     * \param info    stream metadata (streamId, priority, deadline)
     * \return selected path, or nullptr if none are feasible.
     */
    virtual Ptr<MpQuicPath> SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                        const MpQuicStreamInfo& info) = 0;

    /// Returns true for schedulers that send on ALL paths (e.g. Redundant).
    virtual bool IsRedundant() const { return false; }

    /**
     * Apply TiL advisory for next epoch.
     * d = wAdv - wBase is the signed adjustment relative to the baseline used
     * to compute wAdv.  Default: no-op (neutral — preserves existing state).
     */
    virtual void ApplyAdvisory(double /*wAdv*/, double /*wBase*/) {}

    /// Revoke advisory; scheduler reverts to its own selection rules.
    virtual void ClearAdvisory() {}

    /**
     * Return true if this scheduler may return nullptr to request a packet defer
     * (as opposed to a genuine "no feasible path" drop).
     * Socket defers the packet to m_deferredSends instead of dropping permanently.
     */
    virtual bool DeferOnNullPath() const { return false; }

    /**
     * TwinSteer hard binary override: suppress one path entirely for the next epoch.
     * suppressUu=true  → route all frames to PC5 (Uu predicted degraded, PC5 ok).
     * suppressPc5=true → route all frames to Uu  (PC5 predicted degraded, Uu ok).
     * Both true or both false → full scheduler freedom (ClearAdvisory).
     * Default implementation falls back to ApplyAdvisory with saturated weights.
     */
    virtual void ApplyBinaryAdvisory(bool suppressUu, bool suppressPc5, double wBase)
    {
        if      (suppressUu && !suppressPc5)  ApplyAdvisory(0.0,   wBase);
        else if (!suppressUu && suppressPc5)  ApplyAdvisory(1.0,   wBase);
        else                                  ClearAdvisory();
    }

    /**
     * Compute per-path replication map for Redundant-mode senders.
     * Called ONCE per frame in MpQuicSocketBase::Send(); result is stored with
     * any deferred entry so FlushDeferredSends() never re-consumes credit.
     * Returns empty map by default (non-Redundant schedulers ignore this).
     */
    virtual std::unordered_map<uint8_t, bool>
    ComputeReplicationMap(const std::vector<Ptr<MpQuicPath>>& /*paths*/) { return {}; }
};

// ---------------------------------------------------------------------------

/**
 * Round-robin: cycles through active paths in order (stream-agnostic).
 */
class MpQuicRoundRobinScheduler : public MpQuicScheduler
{
  public:
    static TypeId GetTypeId();
    Ptr<MpQuicPath> SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                const MpQuicStreamInfo& info) override;
    void ApplyAdvisory(double wAdv, double wBase) override;
    void ClearAdvisory() override;

  private:
    uint32_t m_lastIndex{0};
    // Advisory state: credit-based weighted split replaces RR only when |d| > 0.
    // Neutral (m_advActive=false OR m_advD==0): original m_lastIndex cycling.
    bool     m_advActive{false};
    double   m_advWeight{0.5};   ///< Uu fraction [0,1]
    double   m_advD{0.0};        ///< d = wAdv - wBase; 0 → fall through to index-RR
    double   m_advCredit{0.0};   ///< Bresenham credit; reset to 0 on first activation
};

// ---------------------------------------------------------------------------

/**
 * Min-RTT: always selects the active path with the lowest RTT estimate.
 */
class MpQuicMinRttScheduler : public MpQuicScheduler
{
  public:
    static TypeId GetTypeId();
    Ptr<MpQuicPath> SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                const MpQuicStreamInfo& info) override;
    void ApplyAdvisory(double wAdv, double wBase) override;
    void ApplyBinaryAdvisory(bool suppressUu, bool suppressPc5, double wBase) override;
    void ClearAdvisory() override;

  private:
    bool   m_advActive{false};
    // prefer ∈ [-1,+1]: +1=prefer Uu (reduce Uu effective RTT), -1=prefer PC5.
    // Computed as clip(d / kMaxD, -1, 1) where d = wAdv - wBase.
    double m_advPrefer{0.0};

    // Max expected |d|; equals cfg.epsilonP (default 0.3).  Must be > 0.
    static constexpr double kMaxD    = 0.30;
    // Effective-RTT bias gain k: effective_rtt[i] *= (1 - k * prefer * sign[i]).
    // k=0.30 → max 30% RTT reduction on preferred path.  To be validated.
    static constexpr double kAdvGain = 0.30;
};

// ---------------------------------------------------------------------------

/**
 * Redundant: copies every packet to ALL active paths.
 */
class MpQuicRedundantScheduler : public MpQuicScheduler
{
  public:
    static TypeId   GetTypeId();
    bool            IsRedundant() const override { return true; }
    Ptr<MpQuicPath> SelectPath(const std::vector<Ptr<MpQuicPath>>& paths,
                                const MpQuicStreamInfo& info) override;
    void ApplyAdvisory(double wAdv, double wBase) override;
    void ClearAdvisory() override;
    void ApplyBinaryAdvisory(bool suppressUu, bool suppressPc5, double wBase) override;

    /**
     * Compute per-path replication decisions for one frame.
     * Called ONCE per frame in SocketBase::Send(); result stored with deferred entry.
     *
     * Preferred path (pathId 0=Uu when d>0, pathId 1=PC5 when d<0): always true.
     * Less-preferred path: true with ratio q = clip(1 - kAlpha*|d|, 0, 1),
     *   tracked via Bresenham credit m_redCredit.
     * Advisory OFF (d=0): all paths true (full Redundant).
     * pathId→true means "replicate to this path if it is active and not blocked".
     */
    std::unordered_map<uint8_t, bool>
    ComputeReplicationMap(const std::vector<Ptr<MpQuicPath>>& paths) override;

    // Replication-reduction gain α.  q = clip(1 - kAlpha*|d|, 0, 1).
    // kAlpha=0 → no reduction (full Redundant always).  To be validated.
    static constexpr double kAlpha = 0.80;

  private:
    bool    m_advActive{false};
    double  m_advD{0.0};         ///< d = wAdv - wBase; +→prefer Uu, -→prefer PC5
    double  m_redCredit{0.0};    ///< Bresenham credit for less-preferred path replication
};

} // namespace ns3
#endif // MPQUIC_SCHEDULER_H
