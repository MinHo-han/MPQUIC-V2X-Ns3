// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file v2x-headers.h
 * @brief ns-3 Header classes for V2X remote-support uplink simulation.
 *
 * Header hierarchy (composition, not inheritance):
 *   CommonUplinkFields — plain struct embedded in every specialized header.
 *   VideoPacketHeader  — one UDP datagram per video chunk (24 B fixed overhead).
 *   MetaPacketHeader   — per-frame telemetry metadata (49 B fixed overhead).
 *   AwarenessMessageHeader — periodic CAM-like cooperative awareness (41 B).
 *   HazardEventHeader      — event-driven DENM-like hazard notification (31 B).
 *
 * All wire formats use little-endian byte order.
 */
#pragma once

#include "ns3/buffer.h"
#include "ns3/header.h"

#include <cstdint>

namespace ns3
{

// ---------------------------------------------------------------------------
// AppType constants  (stored in CommonUplinkFields::appType)
// ---------------------------------------------------------------------------
static constexpr uint8_t V2X_APP_VIDEO     = 0;
static constexpr uint8_t V2X_APP_META      = 1;
static constexpr uint8_t V2X_APP_AWARENESS = 2;
static constexpr uint8_t V2X_APP_HAZARD    = 3;

// HazardType sub-codes  (stored in HazardEventHeader::hazardType)
static constexpr uint8_t HAZARD_HARD_BRAKE = 0;
static constexpr uint8_t HAZARD_OBSTACLE   = 1;
static constexpr uint8_t HAZARD_ACCIDENT   = 2;
static constexpr uint8_t HAZARD_WEATHER    = 3;

// ---------------------------------------------------------------------------
// CommonUplinkFields
//
// NOT a Header subclass — embedded by composition in each specialised header
// to avoid ns-3 TypeId conflicts when the same concrete type is deserialized.
//
// Wire format (13 bytes, little-endian):
//   vehicleId   : uint32_t  (4 B)
//   sendTimeSec : double    (8 B)   simulation send timestamp (s)
//   appType     : uint8_t   (1 B)   V2X_APP_* constant
// ---------------------------------------------------------------------------
struct CommonUplinkFields
{
    uint32_t vehicleId{0};
    double   sendTimeSec{0.0};
    uint8_t  appType{0};

    void     Serialize(Buffer::Iterator& i) const;
    void     Deserialize(Buffer::Iterator& i);
    uint32_t GetSerializedSize() const { return 13; }
};

// ---------------------------------------------------------------------------
// VideoPacketHeader — one UDP datagram per video chunk.
//
// Wire format (25 bytes):
//   common      : CommonUplinkFields  (13 B)
//   frameId     : uint32_t  (4 B)  monotonically increasing per vehicle
//   chunkId     : uint16_t  (2 B)  0-based chunk index within frame
//   totalChunks : uint16_t  (2 B)  total chunks in this frame
//   frameType   : uint8_t   (1 B)  0 = P-frame, 1 = I-frame (keyframe)
//   payloadSize : uint16_t  (2 B)  actual data bytes in this datagram
//   pathFlag    : uint8_t   (1 B)  1 = Uu, 0 = PC5
// ---------------------------------------------------------------------------
class VideoPacketHeader : public Header
{
  public:
    CommonUplinkFields common;
    uint32_t           frameId{0};
    uint16_t           chunkId{0};
    uint16_t           totalChunks{1};
    uint8_t            frameType{0};
    uint16_t           payloadSize{0};
    uint8_t            pathFlag{1};  ///< 1 = Uu, 0 = PC5

    static TypeId GetTypeId();
    TypeId        GetInstanceTypeId() const override;
    void          Serialize(Buffer::Iterator start) const override;
    uint32_t      Deserialize(Buffer::Iterator start) override;
    uint32_t      GetSerializedSize() const override { return 25; }
    void          Print(std::ostream& os) const override;
};

// ---------------------------------------------------------------------------
// MetaPacketHeader — one datagram per frame carrying vehicle telemetry.
//
// Wire format (49 bytes):
//   common     : CommonUplinkFields  (13 B)
//   frameId    : uint32_t  (4 B)
//   latitude   : double    (8 B)   vehicle X position (m, SUMO coords)
//   longitude  : double    (8 B)   vehicle Y position (m, SUMO coords)
//   heading    : float     (4 B)   degrees CW from north
//   speed      : float     (4 B)   m/s
//   cpuLoad    : float     (4 B)   0.0–1.0 (dummy)
//   memUsage   : float     (4 B)   MB (dummy)
// ---------------------------------------------------------------------------
class MetaPacketHeader : public Header
{
  public:
    CommonUplinkFields common;
    uint32_t           frameId{0};
    double             latitude{0.0};
    double             longitude{0.0};
    float              heading{0.0f};
    float              speed{0.0f};
    float              cpuLoad{0.0f};
    float              memUsage{0.0f};

    static TypeId GetTypeId();
    TypeId        GetInstanceTypeId() const override;
    void          Serialize(Buffer::Iterator start) const override;
    uint32_t      Deserialize(Buffer::Iterator start) override;
    uint32_t      GetSerializedSize() const override { return 49; }
    void          Print(std::ostream& os) const override;
};

// ---------------------------------------------------------------------------
// AwarenessMessageHeader — periodic cooperative awareness (CAM-like).
//
// Wire format (41 bytes):
//   common     : CommonUplinkFields  (13 B)
//   latitude   : double  (8 B)   vehicle X (m)
//   longitude  : double  (8 B)   vehicle Y (m)
//   speedMs    : float   (4 B)   m/s
//   headingDeg : float   (4 B)   degrees CW from north
//   accelMs2   : float   (4 B)   m/s²  (longitudinal, positive = accelerating)
// ---------------------------------------------------------------------------
class AwarenessMessageHeader : public Header
{
  public:
    CommonUplinkFields common;
    double             latitude{0.0};
    double             longitude{0.0};
    float              speedMs{0.0f};
    float              headingDeg{0.0f};
    float              accelMs2{0.0f};

    static TypeId GetTypeId();
    TypeId        GetInstanceTypeId() const override;
    void          Serialize(Buffer::Iterator start) const override;
    uint32_t      Deserialize(Buffer::Iterator start) override;
    uint32_t      GetSerializedSize() const override { return 41; }
    void          Print(std::ostream& os) const override;
};

// ---------------------------------------------------------------------------
// HazardEventHeader — event-driven hazard notification (DENM-like).
//
// Wire format (31 bytes):
//   common     : CommonUplinkFields  (13 B)
//   hazardType : uint8_t  (1 B)   HAZARD_* constant
//   latitude   : double   (8 B)   event X (m)
//   longitude  : double   (8 B)   event Y (m)
//   severity   : uint8_t  (1 B)   0 = low … 3 = critical
// ---------------------------------------------------------------------------
class HazardEventHeader : public Header
{
  public:
    CommonUplinkFields common;
    uint8_t            hazardType{0};
    double             latitude{0.0};
    double             longitude{0.0};
    uint8_t            severity{0};

    static TypeId GetTypeId();
    TypeId        GetInstanceTypeId() const override;
    void          Serialize(Buffer::Iterator start) const override;
    uint32_t      Deserialize(Buffer::Iterator start) override;
    uint32_t      GetSerializedSize() const override { return 31; }
    void          Print(std::ostream& os) const override;
};

} // namespace ns3
