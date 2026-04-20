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

#ifndef NEBULADEC_ADAPTERS__SEYOND_ADAPTER_HPP_
#define NEBULADEC_ADAPTERS__SEYOND_ADAPTER_HPP_

#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

namespace nebula::drivers
{
class SeyondDecoder;
}

namespace nebuladec::adapters
{

/// @brief Adapter that wraps Nebula's SeyondDecoder.
///
/// Seyond's decoder delivers completed scans through a callback; this
/// adapter buffers them and surfaces them via the AnyDecoder::feed()
/// return value, matching the other vendor adapters.
class SeyondAdapter : public AnyDecoder
{
public:
  explicit SeyondAdapter(const Identity & identity);
  ~SeyondAdapter() override;

  SeyondAdapter(const SeyondAdapter &) = delete;
  SeyondAdapter & operator=(const SeyondAdapter &) = delete;

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) override;

  Identity identity() const override {return identity_;}

private:
  Identity identity_;
  std::unique_ptr<nebula::drivers::SeyondDecoder> decoder_;
  std::deque<nebula::drivers::NebulaPointCloudPtr> ready_clouds_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__SEYOND_ADAPTER_HPP_
