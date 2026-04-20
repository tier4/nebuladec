// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

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

// Hesai SOP value is 0xEEFF in wire order, which on a little-endian host
// reads as the byte pair {0xFF, 0xEE} at offsets 0 and 1. The same pair
// also starts a Velodyne block, so the two vendors are disambiguated by
// packet size (Velodyne is always exactly 1206 bytes).
bool has_hesai_sop(const std::uint8_t * data, std::size_t size)
{
  return size >= 2 && data[0] == 0xFF && data[1] == 0xEE;
}

// Seyond SeyondPacketCommon.magic_number is a uint16 with value 0x176A.
// On a little-endian host this is the byte pair {0x6A, 0x17}.
bool has_seyond_magic(const std::uint8_t * data, std::size_t size)
{
  return size >= 2 && data[0] == 0x6A && data[1] == 0x17;
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
    // Packet sizes are derived from the packed Nebula packet structs:
    //   QT128    -> Header12B + BodyWithCrc<Block<Unit4B,128>,2> + FS + Tail
    //   AT128    -> Header12B + Body<FineAzBlock<Unit4B,128>,2> + crc + Tail
    //   128E3X/4X-> Header12B + BodyWithCrc<Block<Unit3B,128>,2> + FS + Tail
    constexpr std::size_t k_qt128_size = 1095;
    constexpr std::size_t k_at128_size = 1078;
    constexpr std::size_t k_128e_size = 861;
    if (size == k_qt128_size) {
      return SensorModel::HESAI_PANDARQT128;
    }
    if (size == k_at128_size) {
      return SensorModel::HESAI_PANDARAT128;
    }
    if (size == k_128e_size) {
      // Pandar128 E3X and E4X share the struct layout; Hesai's public UDP
      // protocol documents the major version as 3 for E3X and 4 for E4X.
      if (protocol_major == 3) {
        return SensorModel::HESAI_PANDAR128_E3X;
      }
      if (protocol_major == 4) {
        return SensorModel::HESAI_PANDAR128_E4X;
      }
      // Default to the newer variant when the version byte is missing
      // or unexpected; decoders for the two are interchangeable at the
      // struct level.
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

}  // namespace

std::optional<Identity> PacketSniffer::identify(const std::uint8_t * data, std::size_t size) const
{
  if (data == nullptr || size == 0) {
    return std::nullopt;
  }

  // Seyond — distinctive magic in the first two bytes; requires enough
  // room for the common header so we can read lidar_type in future
  // milestones.
  constexpr std::size_t k_seyond_min_size = 26;  // sizeof(SeyondPacketCommon)
  if (has_seyond_magic(data, size) && size >= k_seyond_min_size) {
    Identity id;
    id.vendor = Vendor::SEYOND;
    id.model = SensorModel::UNKNOWN;  // M2: vendor-only; sub-model comes later.
    id.return_mode = ReturnMode::UNKNOWN;
    id.confidence = 0.9F;
    return id;
  }

  // Velodyne — fixed packet size and an upper-bank marker.
  if (size == k_velodyne_packet_size && has_hesai_sop(data, size)) {
    Identity id;
    id.vendor = Vendor::VELODYNE;
    id.model = velodyne_model_from_product_id(data[k_velodyne_product_id_offset]);
    id.return_mode = velodyne_return_mode_from_byte(data[k_velodyne_return_mode_offset]);
    id.confidence = 0.95F;
    return id;
  }

  // Hesai — SOP 0xEEFF with variable size; Header12B starts at offset 0.
  if (size >= 12 && has_hesai_sop(data, size)) {
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

std::string to_string(Vendor vendor)
{
  switch (vendor) {
    case Vendor::HESAI:
      return "hesai";
    case Vendor::VELODYNE:
      return "velodyne";
    case Vendor::ROBOSENSE:
      return "robosense";
    case Vendor::SEYOND:
      return "seyond";
    case Vendor::UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace nebuladec
