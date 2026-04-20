// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef NEBULADEC_ADAPTERS__ROBOSENSE_ADAPTER_HPP_
#define NEBULADEC_ADAPTERS__ROBOSENSE_ADAPTER_HPP_

#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace nebula::drivers
{
class RobosenseDriver;
class RobosenseInfoDriver;
struct RobosenseSensorConfiguration;
}  // namespace nebula::drivers

namespace nebuladec::adapters
{

/// @brief Adapter that wraps Nebula's RobosenseDriver.
///
/// Robosense embeds per-unit calibration in DIFOP (device info) packets
/// rather than shipping it as an external file. The adapter consumes
/// these via AnyDecoder::feed_info() and lazily instantiates the
/// decoding driver once calibration has arrived. MSOP packets fed in
/// before that point are silently dropped, matching how Nebula's own
/// ROS wrapper behaves in offline replay mode.
class RobosenseAdapter : public AnyDecoder
{
public:
  explicit RobosenseAdapter(const Identity & identity);
  ~RobosenseAdapter() override;

  RobosenseAdapter(const RobosenseAdapter &) = delete;
  RobosenseAdapter & operator=(const RobosenseAdapter &) = delete;

  /// True once DIFOP has been parsed and the scan driver is constructed.
  [[nodiscard]] bool is_ready() const { return driver_ != nullptr; }

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) override;

  void feed_info(const std::vector<std::uint8_t> & packet) override;

  Identity identity() const override { return identity_; }

private:
  Identity identity_;
  std::shared_ptr<nebula::drivers::RobosenseSensorConfiguration> config_;
  std::unique_ptr<nebula::drivers::RobosenseInfoDriver> info_driver_;
  std::unique_ptr<nebula::drivers::RobosenseDriver> driver_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__ROBOSENSE_ADAPTER_HPP_
