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

#ifndef NEBULADEC_ADAPTERS__DECODER_HPP_
#define NEBULADEC_ADAPTERS__DECODER_HPP_

#include <nebula_core_common/point_types.hpp>
#include <nebuladec_core/any_decoder.hpp>
#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>

#include <cstddef>
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

  /// Flush the final buffered scan, if any, once no more packets will be
  /// fed. Returns the last scan that would otherwise remain inside the
  /// driver (see AnyDecoder::flush). The min_points filter still applies.
  std::optional<nebula::drivers::NebulaPointCloudPtr> flush();

  /// The resolved identity of the stream, once enough packets have been
  /// sniffed. Empty until the first identifiable packet is seen.
  std::optional<Identity> identity() const;

  /// Pre-commit the vendor before any packet is fed. Useful when the bag
  /// layer already knows the vendor from the ROS 2 message type (e.g.
  /// `pandar_msgs/PandarScan` unambiguously implies HESAI). When set, the
  /// sniffer restricts itself to that vendor's model detector and will
  /// never flip to a different vendor, so a stream of malformed packets
  /// cannot be misclassified. Pass `Vendor::UNKNOWN` to clear.
  void set_vendor_hint(Vendor vendor);

  /// Minimum number of points a scan must contain to be surfaced from
  /// feed(). Clouds with fewer points are silently dropped. Defaults to
  /// 1024, which filters the first, partial-revolution scan that
  /// Hesai / Velodyne / Robosense decoders emit when replay starts
  /// mid-rotation. Pass 0 to disable the filter.
  void set_min_points(std::size_t min_points);

  /// The current minimum-points threshold.
  [[nodiscard]] std::size_t min_points() const;

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
