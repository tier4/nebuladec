// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

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

  Identity identity() const override { return identity_; }

private:
  Identity identity_;
  std::unique_ptr<nebula::drivers::SeyondDecoder> decoder_;
  std::deque<nebula::drivers::NebulaPointCloudPtr> ready_clouds_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__SEYOND_ADAPTER_HPP_
