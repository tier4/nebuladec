// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_core/decoder.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace nebuladec
{

struct Decoder::Impl
{
  PacketSniffer sniffer;
  std::optional<Identity> identity;
  std::unique_ptr<AnyDecoder> adapter;
};

Decoder::Decoder() : impl_(std::make_unique<Impl>())
{
}
Decoder::~Decoder() = default;
Decoder::Decoder(Decoder &&) noexcept = default;
Decoder & Decoder::operator=(Decoder &&) noexcept = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> Decoder::feed(
  const std::vector<std::uint8_t> & /*packet*/, double /*stamp_sec*/)
{
  // Skeleton: wiring lands in milestone 3.
  return std::nullopt;
}

void Decoder::feed_info(const std::vector<std::uint8_t> & /*packet*/)
{
  // Skeleton: wiring lands in milestone 6 (Robosense).
}

std::optional<Identity> Decoder::identity() const
{
  return impl_->identity;
}

}  // namespace nebuladec
