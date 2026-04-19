// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef NEBULADEC_CORE__DECODER_HPP_
#define NEBULADEC_CORE__DECODER_HPP_

#include "nebuladec_core/identity.hpp"
#include "nebuladec_core/packet_sniffer.hpp"

#include <nebula_core_common/point_types.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace nebuladec
{

/// @brief Vendor-agnostic adapter that wraps a Nebula driver.
///
/// Implementations live in the `nebuladec_adapters` package.
class AnyDecoder
{
public:
  virtual ~AnyDecoder() = default;

  /// Feed a MSOP / main-data packet. Returns a cloud when a scan completes.
  virtual std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec) = 0;

  /// Feed an info / DIFOP packet (Robosense). No-op for other vendors.
  virtual void feed_info(const std::vector<std::uint8_t> & /*packet*/) {}

  virtual Identity identity() const = 0;
};

/// @brief Top-level SDK facade.
///
/// `feed()` identifies the stream on the first few packets, instantiates the
/// appropriate adapter behind the scenes, and returns completed point clouds
/// as they become available.
class Decoder
{
public:
  Decoder();
  ~Decoder();

  Decoder(const Decoder &) = delete;
  Decoder & operator=(const Decoder &) = delete;
  Decoder(Decoder &&) noexcept;
  Decoder & operator=(Decoder &&) noexcept;

  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec);

  void feed_info(const std::vector<std::uint8_t> & packet);

  std::optional<Identity> identity() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__DECODER_HPP_
