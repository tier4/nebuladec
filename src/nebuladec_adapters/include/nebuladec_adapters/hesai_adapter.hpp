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

#ifndef NEBULADEC_ADAPTERS__HESAI_ADAPTER_HPP_
#define NEBULADEC_ADAPTERS__HESAI_ADAPTER_HPP_

#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

namespace nebula::drivers
{
class HesaiDriver;
}

namespace nebuladec::adapters
{

/// @brief Adapter that wraps Nebula's HesaiDriver.
///
/// Calibration is loaded from the CSV / .dat files shipped by the
/// upstream `nebula_hesai_decoders` package. Users never need to supply
/// a calibration path unless they want to override the factory-nominal
/// defaults (e.g. for per-unit accuracy).
class HesaiAdapter : public AnyDecoder
{
public:
  explicit HesaiAdapter(const Identity & identity);
  ~HesaiAdapter() override;

  HesaiAdapter(const HesaiAdapter &) = delete;
  HesaiAdapter & operator=(const HesaiAdapter &) = delete;

  /// True iff the adapter fully initialised (identified model with a
  /// loadable calibration). A false value makes feed() a no-op.
  [[nodiscard]] bool is_ready() const {return driver_ != nullptr;}

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) override;

  Identity identity() const override {return identity_;}

private:
  Identity identity_;
  std::unique_ptr<nebula::drivers::HesaiDriver> driver_;
  std::deque<nebula::drivers::NebulaPointCloudPtr> ready_clouds_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__HESAI_ADAPTER_HPP_
