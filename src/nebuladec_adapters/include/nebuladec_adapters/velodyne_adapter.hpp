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

  [[nodiscard]] bool is_ready() const {return driver_ != nullptr;}

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) override;

  Identity identity() const override {return identity_;}

private:
  Identity identity_;
  std::unique_ptr<nebula::drivers::VelodyneDriver> driver_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__VELODYNE_ADAPTER_HPP_
