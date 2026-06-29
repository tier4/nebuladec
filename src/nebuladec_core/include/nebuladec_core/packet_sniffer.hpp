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
///
/// Thread-safety: stateless and read-only; a single instance is safe to
/// share across any number of concurrent callers. Internally, identify()
/// only reads from the input buffer and from immutable compile-time
/// tables -- it touches no member state and acquires no locks.
///
/// `vendor_hint` lets the caller pre-commit the vendor (for example when it
/// is derivable from the ROS 2 message type at the bag layer). When set,
/// the sniffer restricts itself to that vendor's detectors and never
/// returns a different vendor. `Vendor::UNKNOWN` (the default) runs every
/// detector and is used for the generic `nebula_msgs/NebulaPackets`
/// container, which today carries Continental radar.
class PacketSniffer
{
public:
  PacketSniffer() = default;

  /// Identify a single packet. Returns nullopt if no vendor matches.
  std::optional<Identity> identify(
    const std::uint8_t * data, std::size_t size, Vendor vendor_hint = Vendor::UNKNOWN) const;

  std::optional<Identity> identify(
    const std::vector<std::uint8_t> & packet, Vendor vendor_hint = Vendor::UNKNOWN) const
  {
    return identify(packet.data(), packet.size(), vendor_hint);
  }
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__PACKET_SNIFFER_HPP_
