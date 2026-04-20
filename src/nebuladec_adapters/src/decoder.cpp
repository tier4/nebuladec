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
    case Vendor::CONTINENTAL:
      // Continental radar: identified, not decoded. nebuladec_adapters
      // does not (yet) emit point clouds from ARS548 / SRR520 packets.
      return nullptr;
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
  Vendor vendor_hint{Vendor::UNKNOWN};
  /// Set once the sniffer has returned a resolved identity so subsequent
  /// packets skip the re-sniff path even when no adapter is available
  /// (e.g. CONTINENTAL radar — identified, not decoded).
  bool identity_locked{false};
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
  if (!impl_->adapter && !impl_->identity_locked) {
    auto sniffed = impl_->sniffer.identify(packet, impl_->vendor_hint);
    if (!sniffed.has_value()) {
      return std::nullopt;
    }
    impl_->identity = sniffed;
    impl_->adapter = make_adapter(*sniffed);
    // Lock the identity only once it carries useful information. With a
    // vendor hint the sniffer always returns *something* (hint + UNKNOWN
    // model) so the model-search would otherwise stop on the first
    // packet even before the real model has been seen. Requiring either
    // a ready adapter or a resolved model keeps the search alive for
    // LiDAR while still terminating it for radar (vendor-only identity
    // with no adapter to build).
    const bool has_model =
      sniffed->model != nebula::drivers::SensorModel::UNKNOWN || sniffed->seyond_model.has_value();
    if (impl_->adapter || (sniffed->vendor == Vendor::CONTINENTAL) || has_model) {
      impl_->identity_locked = true;
    }
    if (!impl_->adapter) {
      return std::nullopt;
    }
  }
  if (!impl_->adapter) {
    return std::nullopt;
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

void Decoder::set_vendor_hint(Vendor vendor)
{
  impl_->vendor_hint = vendor;
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
