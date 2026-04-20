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

#include "nebuladec_adapters/decoder.hpp"

#include "nebuladec_adapters/hesai_adapter.hpp"
#include "nebuladec_adapters/robosense_adapter.hpp"
#include "nebuladec_adapters/seyond_adapter.hpp"
#include "nebuladec_adapters/velodyne_adapter.hpp"

#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebuladec
{

std::unique_ptr<AnyDecoder> make_adapter(const Identity & identity)
{
  switch (identity.vendor) {
    case Vendor::SEYOND:
      return std::make_unique<adapters::SeyondAdapter>(identity);
    case Vendor::HESAI: {
      auto adapter = std::make_unique<adapters::HesaiAdapter>(identity);
      if (!adapter->is_ready()) {
        return nullptr;
      }
      return adapter;
    }
    case Vendor::VELODYNE: {
      auto adapter = std::make_unique<adapters::VelodyneAdapter>(identity);
      if (!adapter->is_ready()) {
        return nullptr;
      }
      return adapter;
    }
    case Vendor::ROBOSENSE:
      // Robosense needs DIFOP to populate its calibration, so the
      // adapter is not yet is_ready() at construction. Returning it
      // anyway lets Decoder::feed_info() drive the initialisation.
      // UNKNOWN model is rejected here since it can never become ready.
      if (identity.model == nebula::drivers::SensorModel::UNKNOWN) {
        return nullptr;
      }
      return std::make_unique<adapters::RobosenseAdapter>(identity);
    case Vendor::UNKNOWN:
    default:
      return nullptr;
  }
}

struct Decoder::Impl
{
  PacketSniffer sniffer;
  std::optional<Identity> identity;
  std::unique_ptr<AnyDecoder> adapter;
  std::size_t min_points{1024};
};

Decoder::Decoder() : impl_(std::make_unique<Impl>())
{
}
Decoder::~Decoder() = default;
Decoder::Decoder(Decoder &&) noexcept = default;
Decoder & Decoder::operator=(Decoder &&) noexcept = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> Decoder::feed(
  const std::vector<std::uint8_t> & packet, double stamp_sec)
{
  if (!impl_->adapter) {
    auto sniffed = impl_->sniffer.identify(packet);
    if (!sniffed.has_value()) {
      return std::nullopt;
    }
    impl_->identity = sniffed;
    impl_->adapter = make_adapter(*sniffed);
    if (!impl_->adapter) {
      return std::nullopt;
    }
  }

  auto cloud = impl_->adapter->feed(packet, stamp_sec);
  if (cloud && *cloud && (*cloud)->size() < impl_->min_points) {
    return std::nullopt;
  }
  return cloud;
}

void Decoder::feed_info(const std::vector<std::uint8_t> & packet)
{
  if (impl_->adapter) {
    impl_->adapter->feed_info(packet);
  }
}

std::optional<Identity> Decoder::identity() const
{
  return impl_->identity;
}

void Decoder::set_min_points(std::size_t min_points)
{
  impl_->min_points = min_points;
}

std::size_t Decoder::min_points() const
{
  return impl_->min_points;
}

}  // namespace nebuladec
