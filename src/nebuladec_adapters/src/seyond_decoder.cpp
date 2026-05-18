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

#include "seyond_decoder.hpp"

#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_decoders/seyond_packet.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

/// Mirror of nebula's static `packet_timestamp_us_to_ns` so the
/// angle_hv consume path can return a faithful timestamp without
/// linking against nebula's private helper.
std::uint64_t ts_us_to_ns(double ts_us)
{
  if (!std::isfinite(ts_us) || ts_us < 0.0) {
    return 0;
  }
  const long double ts_ns_ld = static_cast<long double>(ts_us) * 1000.0L;
  return static_cast<std::uint64_t>(std::min<long double>(
    ts_ns_ld, static_cast<long double>(std::numeric_limits<std::uint64_t>::max())));
}

bool is_angle_hv_type(std::uint8_t type)
{
  using nebula::drivers::detail::hummingbird_angle_hv_table_type;
  using nebula::drivers::detail::robine1x_angle_hv_table_type;
  using nebula::drivers::detail::robine2x_angle_hv_table_type;
  using nebula::drivers::detail::robinw_angle_hv_table_type;
  return type == robinw_angle_hv_table_type || type == robine1x_angle_hv_table_type ||
         type == hummingbird_angle_hv_table_type || type == robine2x_angle_hv_table_type;
}

/// True iff the packet is large enough to host a `SeyondDataPacket`
/// header, carries the Seyond magic, and its `type` field matches one
/// of the four known angle_hv_table type codes.
bool is_angle_hv_table_packet(const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < sizeof(nebula::drivers::SeyondDataPacket)) {
    return false;
  }
  // Reinterpret is unavoidable for parsing a packed UDP wire header; the
  // same pattern is used throughout nebula's own packet parsers.
  // NOLINTNEXTLINE
  const auto * header = reinterpret_cast<const nebula::drivers::SeyondDataPacket *>(packet.data());
  if (header->common.magic_number != nebula::drivers::detail::seyond_data_packet_magic) {
    return false;
  }
  return is_angle_hv_type(static_cast<std::uint8_t>(header->type));
}

/// Strip the 70-byte `SeyondDataPacket` header and return the body
/// bytes. Returns `nullopt` if `common.size` is not a sane self-report
/// (smaller than the header, or larger than what actually arrived) --
/// the caller must not latch `angle_hv_applied_` in that case, so a
/// later well-formed packet can still apply.
std::optional<std::vector<std::uint8_t>> extract_angle_hv_payload(
  const std::vector<std::uint8_t> & packet)
{
  constexpr auto k_header_size = sizeof(nebula::drivers::SeyondDataPacket);
  if (packet.size() < k_header_size) {
    return std::nullopt;
  }
  // NOLINTNEXTLINE
  const auto * header = reinterpret_cast<const nebula::drivers::SeyondDataPacket *>(packet.data());
  const auto declared_size = static_cast<std::size_t>(header->common.size);
  if (declared_size < k_header_size || declared_size > packet.size()) {
    return std::nullopt;
  }
  const auto body_begin = packet.cbegin() + static_cast<std::ptrdiff_t>(k_header_size);
  const auto body_end = packet.cbegin() + static_cast<std::ptrdiff_t>(declared_size);
  return std::vector<std::uint8_t>(body_begin, body_end);
}

std::uint64_t read_header_timestamp_ns(const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < sizeof(nebula::drivers::SeyondPacketCommon)) {
    return 0;
  }
  // NOLINTNEXTLINE
  const auto * common =
    reinterpret_cast<const nebula::drivers::SeyondPacketCommon *>(packet.data());
  return ts_us_to_ns(common->ts_start_us);
}

}  // namespace

SeyondDecoder::SeyondDecoder(
  nebula::drivers::SeyondSensorConfiguration config, pointcloud_callback_t pointcloud_cb)
: config_(std::move(config)), callback_(std::move(pointcloud_cb))
{
  inner_ = std::make_unique<nebula::drivers::SeyondDecoder>(
    config_, callback_, nebula::drivers::SeyondCalibrationData{});
}

nebula::drivers::SeyondPacketDecodeResult SeyondDecoder::unpack(
  const std::vector<std::uint8_t> & packet)
{
  if (is_angle_hv_table_packet(packet)) {
    const auto ts_ns = read_header_timestamp_ns(packet);

    if (!angle_hv_applied_) {
      if (auto payload = extract_angle_hv_payload(packet); payload) {
        nebula::drivers::SeyondCalibrationData calibration;
        calibration.angle_hv_table = std::move(*payload);
        // Rebuilding the inner decoder drops any partial scan it was
        // accumulating. That partial scan was decoded with no
        // calibration -- those points are unreliable by design -- so
        // discarding it is the correct behavior.
        inner_ = std::make_unique<nebula::drivers::SeyondDecoder>(
          config_, callback_, std::move(calibration));
        angle_hv_applied_ = true;
      }
    }
    // Match nebula's `supported_layout = false` rejection signature.
    return {ts_ns, 0, false};
  }

  return inner_->unpack(packet);
}

}  // namespace nebuladec::adapters
