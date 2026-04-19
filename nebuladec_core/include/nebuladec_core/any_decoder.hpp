// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

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

  virtual Identity identity() const = 0;
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__ANY_DECODER_HPP_
