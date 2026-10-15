// SPDX-License-Identifier: GPL-2.0-only
#include "v2x-headers.h"

#include "ns3/log.h"

#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("V2xHeaders");

// ---------------------------------------------------------------------------
// Internal helpers: IEEE-754 double/float ↔ uint64/32 via memcpy
// (Buffer::Iterator has no native float/double methods)
// ---------------------------------------------------------------------------
namespace
{

void
WriteDouble(Buffer::Iterator& i, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, 8);
    i.WriteHtolsbU64(bits);
}

double
ReadDouble(Buffer::Iterator& i)
{
    uint64_t bits = i.ReadLsbtohU64();
    double   v;
    std::memcpy(&v, &bits, 8);
    return v;
}

void
WriteFloat(Buffer::Iterator& i, float v)
{
    uint32_t bits;
    std::memcpy(&bits, &v, 4);
    i.WriteHtolsbU32(bits);
}

float
ReadFloat(Buffer::Iterator& i)
{
    uint32_t bits = i.ReadLsbtohU32();
    float    v;
    std::memcpy(&v, &bits, 4);
    return v;
}

} // unnamed namespace

// ---------------------------------------------------------------------------
// CommonUplinkFields
// ---------------------------------------------------------------------------
void
CommonUplinkFields::Serialize(Buffer::Iterator& i) const
{
    i.WriteHtolsbU32(vehicleId);
    WriteDouble(i, sendTimeSec);
    i.WriteU8(appType);
}

void
CommonUplinkFields::Deserialize(Buffer::Iterator& i)
{
    vehicleId   = i.ReadLsbtohU32();
    sendTimeSec = ReadDouble(i);
    appType     = i.ReadU8();
}

// ---------------------------------------------------------------------------
// VideoPacketHeader
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(VideoPacketHeader);

TypeId
VideoPacketHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::VideoPacketHeader")
                            .SetParent<Header>()
                            .SetGroupName("V2xRemoteSupport")
                            .AddConstructor<VideoPacketHeader>();
    return tid;
}

TypeId
VideoPacketHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
VideoPacketHeader::Serialize(Buffer::Iterator start) const
{
    common.Serialize(start);
    start.WriteHtolsbU32(frameId);
    start.WriteHtolsbU16(chunkId);
    start.WriteHtolsbU16(totalChunks);
    start.WriteU8(frameType);
    start.WriteHtolsbU16(payloadSize);
    start.WriteU8(pathFlag);
}

uint32_t
VideoPacketHeader::Deserialize(Buffer::Iterator start)
{
    common.Deserialize(start);
    frameId     = start.ReadLsbtohU32();
    chunkId     = start.ReadLsbtohU16();
    totalChunks = start.ReadLsbtohU16();
    frameType   = start.ReadU8();
    payloadSize = start.ReadLsbtohU16();
    pathFlag    = start.ReadU8();
    return GetSerializedSize();
}

void
VideoPacketHeader::Print(std::ostream& os) const
{
    os << "VideoPacket veh=" << common.vehicleId << " frame=" << frameId << "/"
       << (frameType ? 'I' : 'P') << " chunk=" << chunkId << "/" << totalChunks
       << " t=" << common.sendTimeSec << "s";
}

// ---------------------------------------------------------------------------
// MetaPacketHeader
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(MetaPacketHeader);

TypeId
MetaPacketHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MetaPacketHeader")
                            .SetParent<Header>()
                            .SetGroupName("V2xRemoteSupport")
                            .AddConstructor<MetaPacketHeader>();
    return tid;
}

TypeId
MetaPacketHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
MetaPacketHeader::Serialize(Buffer::Iterator start) const
{
    common.Serialize(start);
    start.WriteHtolsbU32(frameId);
    WriteDouble(start, latitude);
    WriteDouble(start, longitude);
    WriteFloat(start, heading);
    WriteFloat(start, speed);
    WriteFloat(start, cpuLoad);
    WriteFloat(start, memUsage);
}

uint32_t
MetaPacketHeader::Deserialize(Buffer::Iterator start)
{
    common.Deserialize(start);
    frameId   = start.ReadLsbtohU32();
    latitude  = ReadDouble(start);
    longitude = ReadDouble(start);
    heading   = ReadFloat(start);
    speed     = ReadFloat(start);
    cpuLoad   = ReadFloat(start);
    memUsage  = ReadFloat(start);
    return GetSerializedSize();
}

void
MetaPacketHeader::Print(std::ostream& os) const
{
    os << "Meta veh=" << common.vehicleId << " frame=" << frameId << " pos=(" << latitude << ","
       << longitude << ") v=" << speed << "m/s";
}

// ---------------------------------------------------------------------------
// AwarenessMessageHeader
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(AwarenessMessageHeader);

TypeId
AwarenessMessageHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::AwarenessMessageHeader")
                            .SetParent<Header>()
                            .SetGroupName("V2xRemoteSupport")
                            .AddConstructor<AwarenessMessageHeader>();
    return tid;
}

TypeId
AwarenessMessageHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
AwarenessMessageHeader::Serialize(Buffer::Iterator start) const
{
    common.Serialize(start);
    WriteDouble(start, latitude);
    WriteDouble(start, longitude);
    WriteFloat(start, speedMs);
    WriteFloat(start, headingDeg);
    WriteFloat(start, accelMs2);
}

uint32_t
AwarenessMessageHeader::Deserialize(Buffer::Iterator start)
{
    common.Deserialize(start);
    latitude   = ReadDouble(start);
    longitude  = ReadDouble(start);
    speedMs    = ReadFloat(start);
    headingDeg = ReadFloat(start);
    accelMs2   = ReadFloat(start);
    return GetSerializedSize();
}

void
AwarenessMessageHeader::Print(std::ostream& os) const
{
    os << "Awareness veh=" << common.vehicleId << " pos=(" << latitude << "," << longitude
       << ") v=" << speedMs << "m/s t=" << common.sendTimeSec << "s";
}

// ---------------------------------------------------------------------------
// HazardEventHeader
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED(HazardEventHeader);

TypeId
HazardEventHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::HazardEventHeader")
                            .SetParent<Header>()
                            .SetGroupName("V2xRemoteSupport")
                            .AddConstructor<HazardEventHeader>();
    return tid;
}

TypeId
HazardEventHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
HazardEventHeader::Serialize(Buffer::Iterator start) const
{
    common.Serialize(start);
    start.WriteU8(hazardType);
    WriteDouble(start, latitude);
    WriteDouble(start, longitude);
    start.WriteU8(severity);
}

uint32_t
HazardEventHeader::Deserialize(Buffer::Iterator start)
{
    common.Deserialize(start);
    hazardType = start.ReadU8();
    latitude   = ReadDouble(start);
    longitude  = ReadDouble(start);
    severity   = start.ReadU8();
    return GetSerializedSize();
}

void
HazardEventHeader::Print(std::ostream& os) const
{
    os << "Hazard veh=" << common.vehicleId << " type=" << (int)hazardType << " pos=(" << latitude
       << "," << longitude << ") sev=" << (int)severity;
}

} // namespace ns3
