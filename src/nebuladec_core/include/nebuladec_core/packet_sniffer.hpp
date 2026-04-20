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
