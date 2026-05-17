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

class AcceleratedSeyondDecoder;

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

  /// Flush the final buffered scan. Replays the cached first-scan
  /// packets so either path A (frame_idx change) or path B
  /// (is_last_sub_frame) fires again and delivers the trailing cloud.
  std::optional<nebula::drivers::NebulaPointCloudPtr> flush() override;

  Identity identity() const override { return identity_; }

private:
  Identity identity_;
  // Exactly one of these is non-null at any time. `accelerated_decoder_` is
  // preferred when `AcceleratedSeyondDecoder::supports(model)` returns true and
  // the `NEBULADEC_ACCELERATED_SEYOND` environment variable is not set to "0";
  // otherwise the adapter falls back to the upstream nebula decoder.
  std::unique_ptr<nebula::drivers::SeyondDecoder> decoder_;
  std::unique_ptr<AcceleratedSeyondDecoder> accelerated_decoder_;
  std::deque<nebula::drivers::NebulaPointCloudPtr> ready_clouds_;
  /// Packets from the first scan of the stream, captured until the
  /// decoder fires its first callback. Replayed by flush() so the
  /// decoder's frame-boundary detector (either the next-frame-idx path
  /// or the is_last_sub_frame path) triggers on the trailing scan that
  /// is still buffered in `current_scan_cloud_` at end-of-stream.
  std::vector<std::vector<std::uint8_t>> first_scan_packets_;
  bool first_scan_captured_{false};
  /// Whether the most recent feed() already completed a scan inside the
  /// decoder. Used by flush() to skip replay when there is nothing to
  /// recover -- otherwise a clean end-of-bag would leak a duplicate of
  /// the first scan through the is_last_sub_frame path.
  bool last_feed_scan_complete_{false};
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__SEYOND_ADAPTER_HPP_
