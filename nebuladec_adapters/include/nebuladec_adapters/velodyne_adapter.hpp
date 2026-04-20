// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_
#define NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_

#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace nebula::drivers
{
class VelodyneDriver;
}

namespace nebuladec::adapters
{

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

  [[nodiscard]] bool is_ready() const { return driver_ != nullptr; }

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) override;

  Identity identity() const override { return identity_; }

private:
  Identity identity_;
  std::unique_ptr<nebula::drivers::VelodyneDriver> driver_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_
