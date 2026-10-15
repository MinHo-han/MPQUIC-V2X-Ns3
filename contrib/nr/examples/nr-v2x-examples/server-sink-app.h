// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file server-sink-app.h
 * @brief Server-side sink for all V2X uplink streams.
 *
 * Per-vehicle CSV output in OutputDir/vehicles/:
 *   veh_{id}_video_raw.csv  — every received video chunk
 *   veh_{id}_frames.csv     — per-frame delivery summary
 *   veh_{id}_meta.csv       — per-frame metadata telemetry
 *   veh_{id}_coop.csv       — awareness + hazard messages
 *   veh_{id}_epoch.csv      — 100 ms epoch log (TiL predictor input)
 *
 * Epoch log columns (30 total):
 *   [Index]
 *   1.  epoch_idx                        monotone epoch counter
 *   2.  epoch_start_ms                   epoch start (simulation ms)
 *   3.  epoch_end_ms                     epoch end   (simulation ms)
 *
 *   [y_recv^Uu — Uu video receive evidence]
 *   4.  uu_completed_frames              video frames fully assembled via Uu
 *   5.  uu_timeout_frames                Uu frames that exceeded deadline
 *   6.  uu_deadline_met_frames           Uu completed frames within deadline
 *   7.  uu_avg_delay_ms                  mean Uu E2E delay of completed frames
 *   8.  uu_max_delay_ms                  max  Uu E2E delay of completed frames
 *
 *   [y_recv^PC5 — PC5 video receive evidence]
 *   9.  pc5_video_completed_frames       video frames fully assembled via PC5
 *   10. pc5_video_timeout_frames         PC5 frames that exceeded deadline
 *   11. pc5_video_deadline_met_frames    PC5 completed frames within deadline
 *   12. pc5_video_avg_delay_ms           mean PC5 video E2E delay
 *   13. pc5_video_max_delay_ms           max  PC5 video E2E delay
 *
 *   [y_recv^coop — cooperative / BSM message evidence]
 *   14. coop_rx_packets                  coop (BSM/CAM) packets received
 *   15. coop_avg_delay_ms                mean coop E2E delay
 *   16. coop_max_delay_ms                max  coop E2E delay
 *
 *   [y_svc — service continuity evidence]
 *   17. freeze_flag_e2e                  1 if (uu+pc5) deadline_met_frames == 0
 *   18. consecutive_freeze_epochs        how many epochs in a row were frozen
 *
 *   [r_veh — last received vehicle-state telemetry]
 *   19. last_rx_x                        last received x coordinate (m)
 *   20. last_rx_y                        last received y coordinate (m)
 *   21. last_rx_speed                    last received speed (m/s)
 *   22. last_rx_heading                  last received heading (deg)
 *   23. last_rx_acceleration             last received acceleration (m/s²)
 *   24. age_of_last_vehicle_state_ms     ms since last r_veh telemetry arrived
 *
 *   [m_prior — map / infrastructure priors, computed server-side]
 *   25. dist_to_nearest_gnb              dist from last_rx pos to nearest gNB (m)
 *   26. dist_to_nearest_rsu              dist from last_rx pos to nearest RSU (m)
 *   27. recent_handover_flag             1 if HO occurred within kRecentHoWindowSec
 *   28. time_since_last_handover_ms      ms since last Uu handover (-1 = never)
 *
 *   [v_scene — traffic-load / scene proxy]
 *   29. num_i_frames                     I-frames completed this epoch (Uu+PC5)
 *   30. num_p_frames                     P-frames completed this epoch (Uu+PC5)
 */
#pragma once

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/socket.h"

#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

class ServerSinkApp : public Application
{
  public:
    static TypeId GetTypeId();
    ServerSinkApp();
    ~ServerSinkApp() override;

    /**
     * Pass fixed infrastructure positions so the server can compute
     * dist_to_serving_gnb and dist_to_nearest_rsu each epoch.
     * Must be called before Simulator::Run().
     */
    void SetInfrastructurePositions(const std::vector<std::pair<double, double>>& gnbXY,
                                    const std::vector<std::pair<double, double>>& rsuXY);

    /**
     * Called by UuHandoverHelper whenever a position-based Uu handover fires.
     * Updates recent_handover_flag and time_since_last_handover_ms for vehicleId.
     */
    void NotifyHandover(uint32_t vehicleId);

  private:
    void StartApplication() override;
    void StopApplication() override;

    Ptr<Socket> MakeUdpSink(uint16_t port, Callback<void, Ptr<Socket>> recvCb);

    void RecvVideo(Ptr<Socket> socket);
    void RecvMeta(Ptr<Socket> socket);
    void RecvCoop(Ptr<Socket> socket);
    void CheckFrameDeadlines();
    void WriteEpochSummaries();

    // Per-vehicle file accessors (open on first use)
    std::ofstream& VideoRawFile(uint32_t vehicleId);
    std::ofstream& FramesFile(uint32_t vehicleId);
    std::ofstream& MetaFile(uint32_t vehicleId);
    std::ofstream& CoopFile(uint32_t vehicleId);
    std::ofstream& EpochFile(uint32_t vehicleId);

    // Infrastructure distance helpers
    double CalcDistToNearestGnb(double x, double y) const;
    double CalcDistToNearestRsu(double x, double y) const;

    // ── Attributes ──────────────────────────────────────────────────────────
    uint16_t    m_videoPort{5000};
    uint16_t    m_metaPort{5001};
    uint16_t    m_coopPort{5002};
    std::string m_outputDir{"."};
    double      m_frameDeadlineSec{0.1};
    double      m_iFrameThreshold{0.90};  ///< min useful-chunk ratio for I-frame acceptance
    double      m_pFrameThreshold{0.70};  ///< min useful-chunk ratio for P-frame acceptance  ///< video delivery deadline (s)
    double      m_epochIntervalSec{0.1};  ///< TiL epoch width T_d (s)

    // ── Sockets & timers ─────────────────────────────────────────────────────
    Ptr<Socket> m_videoSocket;
    Ptr<Socket> m_metaSocket;
    Ptr<Socket> m_coopSocket;
    EventId     m_deadlineCheckEvent;
    EventId     m_epochSummaryEvent;

    // ── Per-vehicle file streams ──────────────────────────────────────────────
    std::string m_vehiclesDir;
    std::map<uint32_t, std::ofstream> m_videoRawFiles;
    std::map<uint32_t, std::ofstream> m_framesFiles;
    std::map<uint32_t, std::ofstream> m_metaFiles;
    std::map<uint32_t, std::ofstream> m_coopFiles;
    std::map<uint32_t, std::ofstream> m_epochFiles;

    // ── Infrastructure positions ──────────────────────────────────────────────
    std::vector<std::pair<double, double>> m_gnbXY; ///< gNB (x,y) in metres
    std::vector<std::pair<double, double>> m_rsuXY; ///< RSU (x,y) in metres

    // ── Handover state (per vehicle) ─────────────────────────────────────────
    /// HO within this window is flagged as "recent"
    static constexpr double kRecentHoWindowSec = 5.0;

    struct HandoverState
    {
        double lastHandoverTimeSec{-1.0}; ///< sim time of last HO; -1 = never
    };
    std::map<uint32_t, HandoverState> m_hoStates;

    // ── Per-epoch accumulator (per vehicle) ───────────────────────────────────
    // freeze_flag = 1  iff  uu_deadline_met_frames == 0 for this epoch.
    // Epoch file is written only for isVideoVehicle == true.

    struct EpochState
    {
        // ── epoch index & timing ─────────────────────────────────────────
        uint32_t epochIdx{0};        ///< col 1
        double   epochStartSec{0.0}; ///< col 2 source

        // ── video vehicle marker ─────────────────────────────────────────
        bool isVideoVehicle{false};  ///< set when first video chunk arrives

        // ── y_recv^Uu (reset each epoch) ─────────────────────────────────
        uint32_t uuCompletedFrames{0};   ///< col 4
        uint32_t uuTimeoutFrames{0};     ///< col 5
        uint32_t uuDeadlineMetFrames{0}; ///< col 6
        double   uuLatencySum{0.0};      ///< accumulator for col 7
        double   uuLatencyMax{-1.0};     ///< col 8; -1 = no data

        // ── y_recv^PC5 video (reset each epoch) ──────────────────────────
        uint32_t pc5VideoCompletedFrames{0};   ///< col 9
        uint32_t pc5VideoTimeoutFrames{0};     ///< col 10
        uint32_t pc5VideoDeadlineMetFrames{0}; ///< col 11
        double   pc5VideoLatencySum{0.0};      ///< accumulator for col 12
        double   pc5VideoLatencyMax{-1.0};     ///< col 13; -1 = no data

        // ── y_recv^coop / BSM (reset each epoch) ──────────────────────────
        uint32_t coopRxPackets{0};    ///< col 14
        double   coopLatencySum{0.0}; ///< accumulator for col 15
        double   coopLatencyMax{-1.0};///< col 16; -1 = no data

        // ── y_svc (carry-over across epochs) ─────────────────────────────
        uint32_t consecutiveFreezeEpochs{0}; ///< col 18

        // ── r_veh — last received values (carry-over, never reset) ────────
        double lastRxX{0.0};          ///< col 19
        double lastRxY{0.0};          ///< col 20
        double lastRxSpeed{0.0};      ///< col 21
        double lastRxHeading{0.0};    ///< col 22
        double lastRxAccel{0.0};      ///< col 23; from AwarenessMessageHeader
        double lastVehicleStateRecvSec{-1.0}; ///< sim time of last r_veh update

        // ── v_scene (reset each epoch) ────────────────────────────────────
        uint32_t numIFrames{0}; ///< col 29
        uint32_t numPFrames{0}; ///< col 30
    };
    std::map<uint32_t, EpochState> m_epochStates;

    // ── Per-frame delivery tracking ───────────────────────────────────────────
    // Playout-aware model:
    //   usefulChunks = chunks arriving before sendTime + frameDeadline
    //   Frame accepted if usefulChunks/totalChunks >= threshold
    //     I-frame threshold: 0.90,  P-frame threshold: 0.70
    struct FrameState
    {
        uint16_t    totalChunks{0};
        uint16_t    receivedChunks{0};   ///< all arrived chunks (including late)
        uint16_t    usefulChunks{0};     ///< chunks arrived before playout deadline
        uint8_t     frameType{0};        ///< 0=P, 1=I
        double      sendTimeSec{0.0};
        double      firstArrivalSec{-1.0};
        double      lastArrivalSec{-1.0};
        std::string path{"Uu"};          ///< "Uu" or "PC5"
        bool        judged{false};       ///< true once playout deadline has fired
    };
    /// Key: {vehicleId, frameId}
    std::map<std::pair<uint32_t, uint32_t>, FrameState> m_frames;

    // ── Global counters ───────────────────────────────────────────────────────
    uint64_t m_epochCount{0};
    uint64_t m_totalVideoPackets{0};
    uint64_t m_totalMetaPackets{0};
    uint64_t m_totalCoopPackets{0};
    uint64_t m_framesComplete{0};
    uint64_t m_framesTimeout{0};
};

} // namespace ns3
