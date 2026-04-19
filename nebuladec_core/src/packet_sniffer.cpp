// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_core/packet_sniffer.hpp"

#include <string>

namespace nebuladec
{

std::optional<Identity> PacketSniffer::identify(
  const std::uint8_t * /*data*/, std::size_t /*size*/) const
{
  // Skeleton: real implementation lands in milestone 2.
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
