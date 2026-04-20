// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef NEBULADEC_ADAPTERS__DECODER_HPP_
#define NEBULADEC_ADAPTERS__DECODER_HPP_

#include <nebula_core_common/point_types.hpp>
#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace nebuladec
{

/// @brief Top-level SDK facade.
///
/// On the first identifiable packet, `feed()` selects the appropriate
/// vendor adapter and caches it. Subsequent packets flow straight through.
/// Callers do not need to specify vendor, model, or any calibration: the
/// Decoder recovers everything needed for decoding from the bytes alone.
class Decoder
{
public:
  Decoder();
  ~Decoder();

  Decoder(const Decoder &) = delete;
  Decoder & operator=(const Decoder &) = delete;
  Decoder(Decoder &&) noexcept;
  Decoder & operator=(Decoder &&) noexcept;

  /// Feed a MSOP / main-data packet. Returns a point cloud when a scan
  /// completes, nullopt otherwise.
  std::optional<nebula::drivers::NebulaPointCloudPtr> feed(
    const std::vector<std::uint8_t> & packet, double stamp_sec);

  /// Feed an info / DIFOP packet. Currently forwarded unconditionally so
  /// that Robosense calibration can be reconstructed (M6). Safe to call
  /// on non-Robosense streams.
  void feed_info(const std::vector<std::uint8_t> & packet);

  /// The resolved identity of the stream, once enough packets have been
  /// sniffed. Empty until the first identifiable packet is seen.
  std::optional<Identity> identity() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// @brief Construct an adapter for the given identity.
///
/// Exposed so that advanced users can bypass Decoder's auto-routing and
/// build pipelines directly on AnyDecoder. Returns nullptr if the vendor
/// is not yet supported by nebuladec_adapters.
std::unique_ptr<AnyDecoder> make_adapter(const Identity & identity);

}  // namespace nebuladec

#endif  // NEBULADEC_ADAPTERS__DECODER_HPP_
