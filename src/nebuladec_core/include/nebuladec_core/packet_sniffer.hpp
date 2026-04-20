// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef NEBULADEC_CORE__PACKET_SNIFFER_HPP_
#define NEBULADEC_CORE__PACKET_SNIFFER_HPP_

#include "nebuladec_core/identity.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace nebuladec
{

/// @brief Infers vendor/model/return-mode from raw packet bytes.
///
/// Stateless; callers that want robustness against corrupt or mixed streams
/// should run `consensus()` across multiple packets.
class PacketSniffer
{
public:
  PacketSniffer() = default;

  /// Identify a single packet. Returns nullopt if no vendor matches.
  std::optional<Identity> identify(const std::uint8_t * data, std::size_t size) const;

  std::optional<Identity> identify(const std::vector<std::uint8_t> & packet) const
  {
    return identify(packet.data(), packet.size());
  }
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__PACKET_SNIFFER_HPP_
