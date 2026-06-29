// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nebuladec_core/packet_sniffer.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace nebuladec
{

namespace
{

using nebula::drivers::ReturnMode;
using nebula::drivers::SensorModel;

constexpr std::size_t k_velodyne_packet_size = 1206;
constexpr std::size_t k_velodyne_return_mode_offset = 1204;
constexpr std::size_t k_velodyne_product_id_offset = 1205;

// Hesai packets start with SOP 0xEEFF written in big-endian byte order,
// i.e. the byte pair {0xEE, 0xFF} at offsets 0 and 1. Confirmed against
// Pandar128 E3X/E4X captures.
bool has_hesai_sop(const std::uint8_t * data, std::size_t size)
{
  return size >= 2 && data[0] == 0xEE && data[1] == 0xFF;
}

// Velodyne block headers also encode 0xEEFF, but Velodyne streams it in
// little-endian order -> byte pair {0xFF, 0xEE} at offsets 0 and 1.
// Velodyne is further disambiguated by its fixed 1206-byte packet size.
bool has_velodyne_block_magic(const std::uint8_t * data, std::size_t size)
{
  return size >= 2 && data[0] == 0xFF && data[1] == 0xEE;
}

bool has_robosense_bpearl_v3_magic(const std::uint8_t * data, std::size_t size)
{
  static constexpr std::uint8_t k_magic[8] = {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0};
  if (size < 8) {
    return false;
  }
  for (std::size_t i = 0; i < 8; ++i) {
    if (data[i] != k_magic[i]) {
      return false;
    }
  }
  return true;
}

bool has_robosense_short_msop_magic(const std::uint8_t * data, std::size_t size)
{
  static constexpr std::uint8_t k_magic[4] = {0x55, 0xAA, 0x05, 0x5A};
  if (size < 4) {
    return false;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    if (data[i] != k_magic[i]) {
      return false;
    }
  }
  return true;
}

// Map (laser_num, block_num) to a Hesai SensorModel. For the 128-channel
// family (QT128 / AT128 / 128 E3X / 128 E4X) the packet size and, for the
// E3X/E4X split, the protocol major version disambiguate which model
// produced the bytes.
SensorModel hesai_model_from_header(
  std::uint8_t laser_num, std::uint8_t block_num, std::size_t size, std::uint8_t protocol_major)
{
  if (laser_num == 40 && block_num == 10) {
    return SensorModel::HESAI_PANDAR40P;
  }
  if (laser_num == 64 && block_num == 6) {
    return SensorModel::HESAI_PANDAR64;
  }
  if (laser_num == 64 && block_num == 4) {
    return SensorModel::HESAI_PANDARQT64;
  }
  if (laser_num == 16 && block_num == 8) {
    return SensorModel::HESAI_PANDARXT16;
  }
  if (laser_num == 32 && block_num == 8) {
    return SensorModel::HESAI_PANDARXT32;
  }
  if (laser_num == 32 && block_num == 6) {
    return SensorModel::HESAI_PANDARXT32M;
  }
  if (laser_num == 128 && block_num == 2) {
    // Packet size alone uniquely identifies the 128-channel family,
    // verified against Nebula's own ground-truth captures under
    // src/dependencies/nebula/.../test_resources/decoder_ground_truth/:
    //   QT128 (PacketQT128C2X): Header12B + body(Unit4B x128 x2 + CRC) +
    //                           FS + Tail -> 1127 bytes
    //   AT128 (PacketAT128): Header12B + body(FineAzBlock<Unit4B,128> x2) +
    //                        crc + Tail -> 1078 bytes
    //   OT128 (Packet128E4X, alias of Packet128E3X): Header12B +
    //         body(Unit3B x128 x2 + CRC) + FS + Tail -> 861 bytes
    // The protocol_major byte is a generic Hesai UDP protocol version
    // and does NOT split E3X vs E4X (real OT128 captures show
    // proto_major=1; real QT128 captures show proto_major=3) -- the
    // struct-level size is the reliable discriminator.
    (void)protocol_major;
    constexpr std::size_t k_qt128_size = 1127;
    constexpr std::size_t k_at128_size = 1078;
    constexpr std::size_t k_ot128_size = 861;
    if (size == k_qt128_size) {
      return SensorModel::HESAI_PANDARQT128;
    }
    if (size == k_at128_size) {
      return SensorModel::HESAI_PANDARAT128;
    }
    if (size == k_ot128_size) {
      return SensorModel::HESAI_PANDAR128_E4X;
    }
    return SensorModel::UNKNOWN;
  }
  return SensorModel::UNKNOWN;
}

ReturnMode hesai_return_mode_from_byte(std::uint8_t return_num)
{
  switch (return_num) {
    case 0x33:
      return ReturnMode::SINGLE_FIRST;
    case 0x37:
      return ReturnMode::SINGLE_STRONGEST;
    case 0x38:
      return ReturnMode::SINGLE_LAST;
    case 0x39:
      return ReturnMode::DUAL_LAST_STRONGEST;
    case 0x3b:
      return ReturnMode::DUAL_LAST_FIRST;
    case 0x3c:
      return ReturnMode::DUAL_FIRST_STRONGEST;
    case 0x3e:
      return ReturnMode::DUAL;
    default:
      return ReturnMode::UNKNOWN;
  }
}

SensorModel velodyne_model_from_product_id(std::uint8_t product_id)
{
  switch (product_id) {
    case 0x21:
      return SensorModel::VELODYNE_HDL32;
    case 0x22:  // VLP16
    case 0x23:  // Puck LITE
    case 0x24:  // Puck Hi-Res
      return SensorModel::VELODYNE_VLP16;
    case 0x28:  // VLP32C
      return SensorModel::VELODYNE_VLP32;
    case 0xA1:
      return SensorModel::VELODYNE_VLS128;
    default:
      return SensorModel::UNKNOWN;
  }
}

ReturnMode velodyne_return_mode_from_byte(std::uint8_t return_byte)
{
  switch (return_byte) {
    case 55:
      return ReturnMode::SINGLE_STRONGEST;
    case 56:
      return ReturnMode::SINGLE_LAST;
    case 57:
      return ReturnMode::DUAL;
    default:
      return ReturnMode::UNKNOWN;
  }
}

std::optional<Identity> sniff_hesai(const std::uint8_t * data, std::size_t size)
{
  // Hesai — SOP 0xEEFF with variable size; Header12B starts at offset 0.
  if (size < 12 || !has_hesai_sop(data, size)) {
    return std::nullopt;
  }
  const std::uint8_t protocol_major = data[2];
  const std::uint8_t laser_num = data[6];
  const std::uint8_t block_num = data[7];
  const std::uint8_t return_num = data[10];
  // Reject obvious garbage where laser_num / block_num are zero.
  if (laser_num == 0 || block_num == 0) {
    return std::nullopt;
  }
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = hesai_model_from_header(laser_num, block_num, size, protocol_major);
  id.return_mode = hesai_return_mode_from_byte(return_num);
  id.confidence = id.model == SensorModel::UNKNOWN ? 0.7F : 0.9F;
  return id;
}

std::optional<Identity> sniff_velodyne(const std::uint8_t * data, std::size_t size)
{
  // Velodyne — fixed packet size and a little-endian upper-bank marker.
  if (size != k_velodyne_packet_size || !has_velodyne_block_magic(data, size)) {
    return std::nullopt;
  }
  Identity id;
  id.vendor = Vendor::VELODYNE;
  id.model = velodyne_model_from_product_id(data[k_velodyne_product_id_offset]);
  id.return_mode = velodyne_return_mode_from_byte(data[k_velodyne_return_mode_offset]);
  id.confidence = 0.95F;
  return id;
}

std::optional<Identity> sniff_robosense(const std::uint8_t * data, std::size_t size)
{
  // Robosense — Bpearl V3 uses an 8-byte magic; newer families (Helios,
  // Bpearl V4, RS128, ...) use a 4-byte magic. Check the longer pattern
  // first because its first four bytes also match the shorter one.
  if (has_robosense_bpearl_v3_magic(data, size)) {
    Identity id;
    id.vendor = Vendor::ROBOSENSE;
    id.model = SensorModel::ROBOSENSE_BPEARL_V3;
    id.confidence = 0.95F;
    return id;
  }
  if (has_robosense_short_msop_magic(data, size)) {
    Identity id;
    id.vendor = Vendor::ROBOSENSE;
    // The 4-byte magic is shared across several models. Helios is the
    // canonical one in Nebula's SensorModel enum; Bpearl V4 and newer
    // Robosense variants need additional discrimination that lands with
    // the adapter implementation.
    id.model = SensorModel::ROBOSENSE_HELIOS;
    id.confidence = 0.8F;
    return id;
  }
  return std::nullopt;
}

// --- Continental radar (no cloud decoding; identification only) ----------
//
// ARS548: the UDP payload starts with a `HeaderPacket` of three
// big-endian fields {service_id:u16, method_id:u16, length:u32}. The
// service_id is always 0 and the method_id is one of a small set of
// known values, each paired with a specific UDP payload size:
//   390 (configuration)  -> 64
//   336 (detection_list) -> 35336
//   329 (object_list)    -> 9401 (common) or 37452 (fw40)
//   380 (sensor_status)  -> 84
//   396 (filter_status)  -> 330
//
// SRR520: the payload starts with a big-endian uint32 encoding a CAN
// message id. Each id has a fixed packet size:
//   900/1100/1200 (rdi/hrr/object headers)   -> 32
//   901/1101/1201 (rdi/hrr/object elements)  -> 64
//   800 (crc list)                           -> 4
//   700 (status)                             -> 64
//   53  (sync follow-up)                     -> 8
//   600 (veh dyn)                            -> 8
//   601 (sensor config)                      -> 16
//
// Matching both the 4-byte header AND the exact payload size keeps the
// false-positive rate extremely low even though SRR520 and ARS548 share
// the same leading-zero byte layout.

std::uint32_t read_be_uint32(const std::uint8_t * data)
{
  return (static_cast<std::uint32_t>(data[0]) << 24) | (static_cast<std::uint32_t>(data[1]) << 16) |
         (static_cast<std::uint32_t>(data[2]) << 8) | static_cast<std::uint32_t>(data[3]);
}

bool is_ars548_payload(std::uint16_t method_id, std::size_t size)
{
  switch (method_id) {
    case 390:
      return size == 64;
    case 336:
      return size == 35336;
    case 329:
      return size == 9401 || size == 37452;
    case 380:
      return size == 84;
    case 396:
      return size == 330;
    default:
      return false;
  }
}

bool is_srr520_payload(std::uint32_t can_id, std::size_t size)
{
  switch (can_id) {
    case 900:
    case 1100:
    case 1200:
      return size == 32;
    case 901:
    case 1101:
    case 1201:
      return size == 64;
    case 800:
      return size == 4;
    case 700:
      return size == 64;
    case 53:
      return size == 8;
    case 600:
      return size == 8;
    case 601:
      return size == 16;
    default:
      return false;
  }
}

std::optional<Identity> sniff_continental(const std::uint8_t * data, std::size_t size)
{
  if (size < 4) {
    return std::nullopt;
  }
  const std::uint32_t first4 = read_be_uint32(data);
  const std::uint16_t hi = static_cast<std::uint16_t>(first4 >> 16);
  const std::uint16_t lo = static_cast<std::uint16_t>(first4 & 0xFFFF);

  if (hi == 0 && is_ars548_payload(lo, size)) {
    Identity id;
    id.vendor = Vendor::CONTINENTAL;
    id.model = SensorModel::CONTINENTAL_ARS548;
    id.confidence = 0.95F;
    return id;
  }
  if (is_srr520_payload(first4, size)) {
    Identity id;
    id.vendor = Vendor::CONTINENTAL;
    id.model = SensorModel::CONTINENTAL_SRR520;
    id.confidence = 0.95F;
    return id;
  }
  return std::nullopt;
}

}  // namespace

std::optional<Identity> PacketSniffer::identify(
  const std::uint8_t * data, std::size_t size, Vendor vendor_hint) const
{
  if (data == nullptr || size == 0) {
    return std::nullopt;
  }

  // Hint path: restrict to the single vendor's detector. If the bytes do
  // not match we still return the hint vendor with model=UNKNOWN so the
  // caller can report the vendor confidently (it came from the ROS 2
  // message type) while being honest about the unknown model.
  auto with_hint_fallback = [&](Vendor v, std::optional<Identity> resolved) {
    if (resolved) {
      return resolved;
    }
    Identity id;
    id.vendor = v;
    id.model = SensorModel::UNKNOWN;
    id.confidence = 0.0F;
    return std::optional<Identity>{id};
  };

  switch (vendor_hint) {
    case Vendor::HESAI:
      return with_hint_fallback(Vendor::HESAI, sniff_hesai(data, size));
    case Vendor::VELODYNE:
      return with_hint_fallback(Vendor::VELODYNE, sniff_velodyne(data, size));
    case Vendor::ROBOSENSE:
      return with_hint_fallback(Vendor::ROBOSENSE, sniff_robosense(data, size));
    case Vendor::CONTINENTAL:
      return with_hint_fallback(Vendor::CONTINENTAL, sniff_continental(data, size));
    case Vendor::UNKNOWN:
    default:
      break;
  }

  // No hint — try every detector. Order matters only where packet prefixes
  // overlap (Velodyne vs Hesai share the 0xFFEE SOP; disambiguated by
  // packet size inside sniff_velodyne).
  if (auto velodyne = sniff_velodyne(data, size); velodyne) {
    return velodyne;
  }
  if (auto hesai = sniff_hesai(data, size); hesai) {
    return hesai;
  }
  if (auto robosense = sniff_robosense(data, size); robosense) {
    return robosense;
  }
  if (auto continental = sniff_continental(data, size); continental) {
    return continental;
  }
  return std::nullopt;
}

std::string to_string(Vendor vendor)
{
  switch (vendor) {
    case Vendor::HESAI:
      return "hesai";
    case Vendor::VELODYNE:
      return "velodyne";
    case Vendor::ROBOSENSE:
      return "robosense";
    case Vendor::CONTINENTAL:
      return "continental";
    case Vendor::UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace nebuladec
