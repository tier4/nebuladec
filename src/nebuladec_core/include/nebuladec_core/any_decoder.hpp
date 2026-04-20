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

#ifndef NEBULADEC_CORE__ANY_DECODER_HPP_
#define NEBULADEC_CORE__ANY_DECODER_HPP_

#include "nebuladec_core/identity.hpp"

#include <nebula_core_common/point_types.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace nebuladec
{

/// @brief Vendor-agnostic adapter that wraps a Nebula driver.
///
/// Concrete implementations live in the `nebuladec_adapters` package.
class AnyDecoder
{
public:
  virtual ~AnyDecoder() = default;

  /// Feed a MSOP / main-data packet. Returns a cloud when a scan completes,
  /// nullopt otherwise (insufficient data, malformed packet, or mid-scan).
  virtual std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) = 0;

  /// Feed an info / DIFOP packet. Meaningful for Robosense; no-op for
  /// other vendors.
  virtual void feed_info(const std::vector<std::uint8_t> & /*packet*/) {}

  /// Flush any scan buffered inside the underlying driver at end-of-stream.
  /// Mechanical-LiDAR decoders emit a cloud only once the *next* scan's
  /// first packet crosses the cut angle, so the final scan of a bag is
  /// otherwise stuck in the driver. Default is a no-op (nullopt) for
  /// vendors / adapters that do not buffer across packets.
  virtual std::optional<nebula::drivers::NebulaPointCloudPtr> flush() { return std::nullopt; }

  virtual Identity identity() const = 0;
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__ANY_DECODER_HPP_
