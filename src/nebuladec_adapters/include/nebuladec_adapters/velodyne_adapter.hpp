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

#ifndef NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_
#define NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_

#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebula::drivers
{
class VelodyneDriver;
}  // namespace nebula::drivers

namespace nebuladec::adapters
{

class AcceleratedVelodyneDriver;

/// @brief Adapter that wraps Nebula's VelodyneDriver.
///
/// Loads per-model calibration YAML shipped by the upstream
/// `nebula_velodyne_decoders` package, so nothing external is required.
class VelodyneAdapter : public AnyDecoder
{
public:
  explicit VelodyneAdapter(const Identity & identity);
  ~VelodyneAdapter() override;

  VelodyneAdapter(const VelodyneAdapter &) = delete;
  VelodyneAdapter & operator=(const VelodyneAdapter &) = delete;

  [[nodiscard]] bool is_ready() const
  {
    return driver_ != nullptr || accelerated_driver_ != nullptr;
  }

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) override;

  /// Flush the final in-progress scan. Replays the cached first-scan
  /// packets so the decoder's azimuth-wrap detector fires once more,
  /// emitting the scan that would otherwise be stuck in `scan_pc_`.
  std::optional<nebula::drivers::NebulaPointCloudPtr> flush() override;

  Identity identity() const override { return identity_; }

private:
  Identity identity_;
  // Exactly one is non-null after a successful construction.
  // `accelerated_driver_` wins when AcceleratedVelodyneDriver::supports(model) is true
  // and NEBULADEC_ACCELERATED_VELODYNE != "0"; otherwise upstream VelodyneDriver.
  std::unique_ptr<nebula::drivers::VelodyneDriver> driver_;
  std::unique_ptr<AcceleratedVelodyneDriver> accelerated_driver_;
  /// Packets from the first full scan of the stream, captured until the
  /// driver emits its first cloud. Replaying them during flush()
  /// reproduces the original azimuth-wrap transition (cf. Hesai).
  std::vector<std::pair<std::vector<std::uint8_t>, double>> first_scan_packets_;
  bool first_scan_captured_{false};
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_
