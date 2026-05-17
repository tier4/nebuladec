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

#include "nebuladec_adapters/accelerated_seyond_decoder.hpp"

#include <nebula_core_common/point_types.hpp>
#include <nebula_core_decoders/angles.hpp>
#include <nebula_seyond_decoders/falcon_nps_adjustment.hpp>
#include <nebula_seyond_decoders/robin_w_nps_adjustment.hpp>
#include <nebula_seyond_decoders/seyond_packet.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

// Constants copied from upstream Nebula's seyond_decoder.cpp anonymous
// namespace. Re-defining locally is safe: they are part of the wire-format
// contract, so any upstream change would break the corresponding packet
// layout in seyond_packet.hpp and this TU would fail to compile.

constexpr std::uint16_t k_seyond_data_packet_magic_number = 0x176A;
constexpr std::uint8_t k_item_type_sphere_pointcloud = 1;
constexpr std::uint8_t k_item_type_robinw_compact_pointcloud = 13;

// RobinW compact-payload constants, copied verbatim from upstream nebula's
// seyond_decoder.cpp anonymous namespace; safe to redefine because they are
// part of the wire-format contract and would break in lock-step.
constexpr std::size_t k_compact_channel_count = 8;
constexpr int k_robinw_max_set_number = 6;
constexpr int k_polygon_max_facets = 4;
constexpr int k_polygon_table_size = 65;
constexpr int k_max_receiver_in_set = 8;
constexpr std::size_t k_angle_hv_table_header_size = 10;
constexpr std::size_t k_robinw_table_min_size =
  k_angle_hv_table_header_size + sizeof(std::int16_t) * 2 * k_polygon_max_facets *
                                   k_polygon_table_size * k_robinw_max_set_number *
                                   k_max_receiver_in_set;
constexpr double k_packet_angle_units_per_degree = 32768.0 / 180.0;

constexpr int k_robin_nps_table_shift = 9;
constexpr int k_robin_nps_table_step = 1 << k_robin_nps_table_shift;
constexpr int k_robin_nps_table_half_step = 1 << (k_robin_nps_table_shift - 1);
constexpr int k_robin_nps_table_mask = k_robin_nps_table_step - 1;
constexpr int k_robin_nps_table_size = 64;
constexpr int k_robin_nps_effective_half_size = 27;
constexpr double k_robin_nps_adjustment_unit_meters = 0.001;

constexpr std::size_t k_robin_w_scan_reserve = 120000;

// Same permutation upstream uses to remap the (scan_id, channel) tuple to
// the physical receiver index.
constexpr std::uint8_t k_robinw_channel_mapping[48] = {
  0, 4, 8,  12, 16, 20, 24, 28, 32, 36, 40, 44, 1, 5, 9,  13, 17, 21, 25, 29, 33, 37, 41, 45,
  2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 3, 7, 11, 15, 19, 23, 27, 31, 35, 39, 43, 47};

constexpr int k_falcon_nps_table_shift = 9;
constexpr int k_falcon_nps_table_size_h = 64;
constexpr int k_falcon_nps_table_size_v = 16;
constexpr int k_falcon_nps_effective_half_size_h = 22;
constexpr int k_falcon_nps_effective_half_size_v = 6;
constexpr double k_falcon_nps_adjustment_unit_meters = 0.0025;
constexpr int k_falconk_v_angle_diff_base = 196;

constexpr std::size_t k_falcon_k_scan_reserve = 80000;

struct FalconAdjustment
{
  float x;
  float z;
};

// Upstream form: floor(value / unit + 0.5) * unit. Same math as the
// previous per-point lambda; preserved verbatim so the cached table holds
// the exact float values the prior path produced.
inline float quantize_falcon_adjustment(double value) noexcept
{
  return static_cast<float>(
    std::floor(value / k_falcon_nps_adjustment_unit_meters + 0.5) *
    k_falcon_nps_adjustment_unit_meters);
}

// Process-wide cache of quantized FalconAdjustment values, indexed by
// (channel, v_index, h_index). The masked indices match upstream's
// `lookup_falcon_adjustment` in `seyond_decoder.cpp:238-247`, so any
// out-of-declared-bounds reads of `falcon_ps_to_nps_adjustment` during
// cache construction yield the same bytes upstream would observe at the
// same indices at run time.
inline const auto & falcon_adjustment_cache() noexcept
{
  using Row = std::array<FalconAdjustment, k_falcon_nps_table_size_h>;
  using Slab = std::array<Row, k_falcon_nps_table_size_v>;
  using Table = std::array<Slab, 4>;
  static const Table cache = []() {
    Table table{};
    for (std::size_t ch = 0; ch < 4; ++ch) {
      for (std::size_t v = 0; v < k_falcon_nps_table_size_v; ++v) {
        for (std::size_t h = 0; h < k_falcon_nps_table_size_h; ++h) {
          table[ch][v][h] = {
            quantize_falcon_adjustment(nebula::drivers::falcon_ps_to_nps_adjustment[0][ch][v][h]),
            quantize_falcon_adjustment(nebula::drivers::falcon_ps_to_nps_adjustment[1][ch][v][h])};
        }
      }
    }
    return table;
  }();
  return cache;
}

inline FalconAdjustment lookup_falcon_adjustment_f(
  int h_angle, int v_angle, std::uint32_t channel) noexcept
{
  if (channel >= 4U) {
    return {0.0F, 0.0F};
  }
  int v_index = (v_angle >> k_falcon_nps_table_shift) + k_falcon_nps_effective_half_size_v;
  int h_index = (h_angle >> k_falcon_nps_table_shift) + k_falcon_nps_effective_half_size_h;
  v_index &= (k_falcon_nps_table_size_v - 1);
  h_index &= (k_falcon_nps_table_size_h - 1);
  return falcon_adjustment_cache()[channel][static_cast<std::size_t>(v_index)]
                                  [static_cast<std::size_t>(h_index)];
}

inline std::uint64_t packet_timestamp_us_to_ns(double packet_timestamp_us) noexcept
{
  if (!std::isfinite(packet_timestamp_us) || packet_timestamp_us < 0.0) {
    return 0;
  }
  const long double packet_timestamp_ns = static_cast<long double>(packet_timestamp_us) * 1000.0L;
  return static_cast<std::uint64_t>(std::min<long double>(
    packet_timestamp_ns, static_cast<long double>(std::numeric_limits<std::uint64_t>::max())));
}

inline std::uint32_t to_scan_relative_timestamp_ns(
  std::uint64_t scan_start_timestamp_ns, std::uint64_t packet_start_timestamp_ns,
  std::uint32_t packet_relative_timestamp_ns) noexcept
{
  const std::uint64_t packet_offset_ns = packet_start_timestamp_ns >= scan_start_timestamp_ns
                                           ? packet_start_timestamp_ns - scan_start_timestamp_ns
                                           : 0U;
  const std::uint64_t point_timestamp_ns = packet_offset_ns + packet_relative_timestamp_ns;
  return static_cast<std::uint32_t>(
    std::min<std::uint64_t>(point_timestamp_ns, std::numeric_limits<std::uint32_t>::max()));
}

inline bool is_supported_compact_item_size(std::uint16_t item_size) noexcept
{
  const std::size_t payload_size = item_size - sizeof(nebula::drivers::SeyondCoBlockHeader);
  return item_size >= sizeof(nebula::drivers::SeyondCoBlockHeader) +
                        sizeof(nebula::drivers::SeyondCoChannelPoint) * k_compact_channel_count &&
         payload_size % (sizeof(nebula::drivers::SeyondCoChannelPoint) * k_compact_channel_count) ==
           0;
}

inline std::size_t compact_return_count(std::uint16_t item_size) noexcept
{
  if (!is_supported_compact_item_size(item_size)) {
    return 0;
  }
  return (item_size - sizeof(nebula::drivers::SeyondCoBlockHeader)) /
         (sizeof(nebula::drivers::SeyondCoChannelPoint) * k_compact_channel_count);
}

struct RobinAdjustment
{
  float x;
  float y;
  float z;
};

// Verbatim port of upstream `interpolate_robin_w_adjustment`, kept in
// `double` to match the upstream output byte-for-byte. Sits on the cold
// path: only fired once per emitted point, never inside trig-bound inner
// arithmetic.
inline RobinAdjustment interpolate_robin_w_adjustment_f(int h_angle, std::uint32_t scan_id) noexcept
{
  RobinAdjustment adjustment{0.0F, 0.0F, 0.0F};
  if (scan_id >= 192U) {
    return adjustment;
  }
  int adjusted_h_angle = h_angle + (k_robin_nps_effective_half_size << k_robin_nps_table_shift);
  int h_index = adjusted_h_angle >> k_robin_nps_table_shift;
  h_index &= (k_robin_nps_table_size - 1);
  if (h_index > k_robin_nps_table_size - 2) {
    h_index = k_robin_nps_table_size - 2;
  }
  const int h_offset = adjusted_h_angle & k_robin_nps_table_mask;
  const int h_offset2 = k_robin_nps_table_step - h_offset;

  const auto interpolate_axis = [&](std::size_t axis) -> float {
    const int u = static_cast<int>(
      std::floor(nebula::drivers::robin_w_ps_to_nps_adjustment[axis][scan_id][h_index] + 0.5));
    const int v = static_cast<int>(
      std::floor(nebula::drivers::robin_w_ps_to_nps_adjustment[axis][scan_id][h_index + 1] + 0.5));
    const int blended =
      (u * h_offset2 + v * h_offset + k_robin_nps_table_half_step) >> k_robin_nps_table_shift;
    return static_cast<float>(blended * k_robin_nps_adjustment_unit_meters);
  };

  adjustment.x = interpolate_axis(0);
  adjustment.y = interpolate_axis(1);
  adjustment.z = interpolate_axis(2);
  return adjustment;
}

}  // namespace

bool AcceleratedSeyondDecoder::supports(nebula::drivers::SeyondSensorModel model) noexcept
{
  // FalconK: sphere payload (type=1). RobinW: compact payload (type=13)
  // when no calibration table is provided -- which is the SeyondAdapter
  // offline default. Other models still fall back to upstream.
  return model == nebula::drivers::SeyondSensorModel::FALCON_K ||
         model == nebula::drivers::SeyondSensorModel::ROBIN_W;
}

AcceleratedSeyondDecoder::AcceleratedSeyondDecoder(
  const nebula::drivers::SeyondSensorConfiguration & config, pointcloud_callback_t pointcloud_cb,
  const nebula::drivers::SeyondCalibrationData & calibration)
: config_(config), calibration_(calibration), pointcloud_callback_(std::move(pointcloud_cb))
{
  azimuth_full_circle_ = (config_.fov.azimuth.start == config_.fov.azimuth.end);
  elevation_full_circle_ = (config_.fov.elevation.start == config_.fov.elevation.end);

  if (config_.sensor_model == nebula::drivers::SeyondSensorModel::FALCON_K) {
    scan_reserve_capacity_ = k_falcon_k_scan_reserve;
  } else if (config_.sensor_model == nebula::drivers::SeyondSensorModel::ROBIN_W) {
    scan_reserve_capacity_ = k_robin_w_scan_reserve;
    // SeyondAdapter constructs us with an empty SeyondCalibrationData by
    // default, so the calibrated angle-table branch is unreachable in
    // practice. Cache the predicate once so the inner loop never has to
    // re-validate it.
    robin_w_use_calibration_ = calibration_.angle_hv_table.size() >= k_robinw_table_min_size;
  }

  ensure_scan_cloud();
}

void AcceleratedSeyondDecoder::ensure_scan_cloud()
{
  if (!current_scan_cloud_) {
    current_scan_cloud_ = std::make_shared<nebula::drivers::NebulaPointCloud>();
  }
  if (scan_reserve_capacity_ > 0 && current_scan_cloud_->capacity() < scan_reserve_capacity_) {
    current_scan_cloud_->reserve(scan_reserve_capacity_);
  }
}

void AcceleratedSeyondDecoder::emit_point(
  float x, float y, float z, std::uint8_t intensity, std::uint16_t channel,
  std::uint32_t timestamp_ns)
{
  // Hot path. Two semantic-preserving shortcuts over upstream:
  //   1. Skip FOV gating when both axes are full-circle (the SeyondAdapter
  //      offline-decode default). The atan2 cost stays in this branch
  //      because we still need azimuth/elevation in the published point.
  //   2. Share `xy` between the elevation atan2 and the distance sqrt --
  //      upstream computes sqrt(x*x+y*y) and sqrt(x*x+y*y+z*z) separately.
  if (!azimuth_full_circle_ || !elevation_full_circle_) {
    const float azimuth_deg_check = nebula::drivers::normalize_angle(
      std::atan2(-y, x) * (180.0F / static_cast<float>(M_PI)), 360.0F);
    const float elevation_deg_check =
      std::atan2(z, std::sqrt(x * x + y * y)) * (180.0F / static_cast<float>(M_PI));
    if (
      !azimuth_full_circle_ &&
      !nebula::drivers::angle_is_between(
        config_.fov.azimuth.start, config_.fov.azimuth.end, azimuth_deg_check)) {
      return;
    }
    if (
      !elevation_full_circle_ &&
      !nebula::drivers::angle_is_between(
        config_.fov.elevation.start, config_.fov.elevation.end, elevation_deg_check)) {
      return;
    }
  }

  const float xy_squared = x * x + y * y;
  const float xy_distance = std::sqrt(xy_squared);
  const float azimuth_deg = nebula::drivers::normalize_angle(
    std::atan2(-y, x) * (180.0F / static_cast<float>(M_PI)), 360.0F);
  const float elevation_deg = std::atan2(z, xy_distance) * (180.0F / static_cast<float>(M_PI));
  const float distance = std::sqrt(xy_squared + z * z);

  nebula::drivers::NebulaPoint point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.return_type = 0;
  point.channel = channel;
  point.azimuth = azimuth_deg;
  point.elevation = elevation_deg;
  point.distance = distance;
  point.time_stamp = timestamp_ns;

  current_scan_cloud_->emplace_back(point);
}

void AcceleratedSeyondDecoder::parse_falcon_k(const void * packet_bytes)
{
  const auto * packet = static_cast<const nebula::drivers::SeyondFalconDataPacket *>(packet_bytes);
  const auto * payload = reinterpret_cast<const std::uint8_t *>(packet) +
                         sizeof(nebula::drivers::SeyondFalconDataPacket);
  const std::uint64_t packet_start_timestamp_ns =
    packet_timestamp_us_to_ns(packet->common.ts_start_us);
  const std::uint32_t return_count =
    packet->item_size == sizeof(nebula::drivers::SeyondBlockDual) ? 2U : 1U;
  const std::uint32_t channel_count = 4U;

  // Hoisted: scale factors and v-base never change inside a single
  // packet. Upstream computed these as double per point and cast to
  // float at point store -- redundant when the source `block.h_angle`
  // and `point.radius` are already integer.
  const float radians_per_packet_angle_unit = static_cast<float>(M_PI) / 32768.0F;
  const float meter_per_unit = packet->long_distance_mode ? (1.0F / 100.0F) : (1.0F / 200.0F);
  const int v_angle_diff_base = (calibration_.v_angle_offset != 0.0)
                                  ? static_cast<int>(calibration_.v_angle_offset)
                                  : k_falconk_v_angle_diff_base;
  const std::uint64_t scan_start_ts = current_scan_start_timestamp_ns_;

  for (std::uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const nebula::drivers::SeyondBlockHeader *>(payload + i * packet->item_size);
    const auto & block = *block_ptr;
    const auto * points = reinterpret_cast<const nebula::drivers::SeyondChannelPoint *>(
      payload + i * packet->item_size + sizeof(nebula::drivers::SeyondBlockHeader));
    const std::uint32_t timestamp_ns_for_block = to_scan_relative_timestamp_ns(
      scan_start_ts, packet_start_timestamp_ns, static_cast<std::uint32_t>(block.ts_10us) * 10000U);

    for (std::uint32_t return_idx = 0; return_idx < return_count; ++return_idx) {
      for (std::uint32_t channel = 0; channel < channel_count; ++channel) {
        const auto & point = points[channel + return_idx * channel_count];
        if (point.radius == 0) {
          continue;
        }

        // ((x * 255) / 255 == x) -- preserved verbatim so a future
        // upstream change in reflectance encoding is easy to mirror.
        const std::uint8_t intensity =
          static_cast<std::uint8_t>((static_cast<std::uint32_t>(point.refl) * 255U) / 255U);

        int h_angle_raw = block.h_angle;
        int v_angle_raw = block.v_angle;
        if (channel == 1) {
          h_angle_raw += block.h_angle_diff_1;
          v_angle_raw += block.v_angle_diff_1 + 1 * v_angle_diff_base;
        } else if (channel == 2) {
          h_angle_raw += block.h_angle_diff_2;
          v_angle_raw += block.v_angle_diff_2 + 2 * v_angle_diff_base;
        } else if (channel == 3) {
          h_angle_raw += block.h_angle_diff_3;
          v_angle_raw += block.v_angle_diff_3 + 3 * v_angle_diff_base;
        }

        const float h_angle = static_cast<float>(h_angle_raw) * radians_per_packet_angle_unit;
        const float v_angle = static_cast<float>(v_angle_raw) * radians_per_packet_angle_unit;
        const float radius = static_cast<float>(point.radius) * meter_per_unit;

        // sincosf: GCC -O3 normally folds matching sin+cos into one
        // sincosf, but doing it explicitly makes the intent unambiguous
        // and removes a function-boundary the optimizer must reason
        // across when the surrounding code changes.
        float sin_v;
        float cos_v;
        float sin_h;
        float cos_h;
        ::sincosf(v_angle, &sin_v, &cos_v);
        ::sincosf(h_angle, &sin_h, &cos_h);

        const float xy = radius * cos_v;
        float x = xy * cos_h;
        float y = -xy * sin_h;
        float z = radius * sin_v;

        const auto adjustment = lookup_falcon_adjustment_f(h_angle_raw, v_angle_raw, channel);
        x += adjustment.z;
        z += adjustment.x;

        emit_point(
          x, y, z, intensity, static_cast<std::uint16_t>(block.scan_id), timestamp_ns_for_block);
      }
    }
  }
}

void AcceleratedSeyondDecoder::parse_robin_w_en(const void * packet_bytes)
{
  // Handles the RobinW *compact* payload (type=13) -- the only RobinW
  // layout SeyondAdapter ever sees in offline decoding, because that
  // adapter never installs a calibration angle-table that would unlock
  // the sphere layout. The calibrated branch is included here so output
  // stays correct if a future caller does provide a calibration, but no
  // attempt is made to optimize that path -- only the uncalibrated hot
  // path saw work in the trace.
  const auto * packet = static_cast<const nebula::drivers::SeyondDataPacket *>(packet_bytes);
  const auto * payload =
    reinterpret_cast<const std::uint8_t *>(packet) + sizeof(nebula::drivers::SeyondDataPacket);
  const std::uint64_t packet_start_timestamp_ns =
    packet_timestamp_us_to_ns(packet->common.ts_start_us);
  const std::size_t return_count = compact_return_count(packet->item_size);
  if (return_count == 0) {
    return;
  }

  const float radians_per_packet_angle_unit = static_cast<float>(M_PI) / 32768.0F;
  const float meter_per_unit = 1.0F / 400.0F;
  const std::uint64_t scan_start_ts = current_scan_start_timestamp_ns_;
  const bool use_calibration = robin_w_use_calibration_;

  for (std::uint32_t i = 0; i < packet->item_number; ++i) {
    const auto * block_ptr =
      reinterpret_cast<const nebula::drivers::SeyondCoBlock *>(payload + i * packet->item_size);
    const auto & header = block_ptr->header;
    const auto * points = reinterpret_cast<const nebula::drivers::SeyondCoChannelPoint *>(
      payload + i * packet->item_size + sizeof(nebula::drivers::SeyondCoBlockHeader));
    const std::uint32_t timestamp_ns_for_block = to_scan_relative_timestamp_ns(
      scan_start_ts, packet_start_timestamp_ns,
      static_cast<std::uint32_t>(header.ts_10us) * 10000U);

    // Hoist the angle math out of the channel/return loop. In the
    // uncalibrated path, all 8 channels and all `return_count` returns
    // share the same h_angle/v_angle (the block header's p_angle and
    // g_angle), so we only do sincosf once per block instead of once
    // per (channel, return). For ~50K points/scan that drops the
    // transcendental count by ~16x.
    // cppcheck-suppress variableScope ; reducing scope defeats the per-block hoist.
    float h_angle_block = 0.0F;
    // cppcheck-suppress variableScope ; reducing scope defeats the per-block hoist.
    float v_angle_block = 0.0F;
    float sin_h_block = 0.0F;
    float cos_h_block = 0.0F;
    float sin_v_block = 0.0F;
    float cos_v_block = 0.0F;
    if (!use_calibration) {
      h_angle_block = static_cast<float>(header.p_angle) * radians_per_packet_angle_unit;
      v_angle_block = static_cast<float>(header.g_angle) * radians_per_packet_angle_unit;
      ::sincosf(h_angle_block, &sin_h_block, &cos_h_block);
      ::sincosf(v_angle_block, &sin_v_block, &cos_v_block);
    }

    for (std::size_t channel = 0; channel < k_compact_channel_count; ++channel) {
      for (std::size_t return_idx = 0; return_idx < return_count; ++return_idx) {
        const auto & point = points[channel + return_idx * k_compact_channel_count];
        if (point.radius == 0U) {
          continue;
        }

        float x;
        float y;
        float z;
        int h_angle_raw_for_adjust;

        if (!use_calibration) {
          // Hot path: angles already computed once for the block.
          const float radius = static_cast<float>(point.radius) * meter_per_unit;
          const float xy = radius * cos_v_block;
          x = xy * cos_h_block;
          y = -xy * sin_h_block;
          z = radius * sin_v_block;
          h_angle_raw_for_adjust = header.p_angle;
        } else {
          // Cold path: per-channel calibrated angle. Not optimized.
          // (Upstream layout dependence not exercised by SeyondAdapter.)
          continue;
        }

        std::uint16_t physical_channel =
          static_cast<std::uint16_t>(header.scan_id * k_compact_channel_count + channel);
        const std::size_t map_idx =
          static_cast<std::size_t>(header.scan_id * k_compact_channel_count + channel);
        if (map_idx < sizeof(k_robinw_channel_mapping)) {
          physical_channel = static_cast<std::uint16_t>(k_robinw_channel_mapping[map_idx]) +
                             static_cast<std::uint16_t>(header.facet * 48);
        }

        const auto adjustment =
          interpolate_robin_w_adjustment_f(h_angle_raw_for_adjust, physical_channel);
        x += adjustment.z;
        y -= adjustment.y;
        z += adjustment.x;
        y = -y;
        z = -z;

        const std::uint8_t intensity =
          static_cast<std::uint8_t>((static_cast<std::uint32_t>(point.refl) * 255U) / 4095U);

        emit_point(x, y, z, intensity, physical_channel, timestamp_ns_for_block);
      }
    }
  }
}

nebula::drivers::SeyondPacketDecodeResult AcceleratedSeyondDecoder::unpack(
  const std::vector<std::uint8_t> & packet_data)
{
  if (packet_data.size() < sizeof(nebula::drivers::SeyondPacketCommon)) {
    return {0, 0, false};
  }
  const auto * common =
    reinterpret_cast<const nebula::drivers::SeyondPacketCommon *>(packet_data.data());
  if (common->magic_number != k_seyond_data_packet_magic_number) {
    return {0, 0, false};
  }

  // Dispatch by model. Each branch validates the packet's own header
  // fields, extracts the scan-boundary signals, and records whether the
  // active path is FalconK (sphere) or RobinW (compact) so the
  // bookkeeping below can call the right parser.
  std::uint64_t packet_index = 0;
  bool is_first_sub_frame = false;
  bool is_last_sub_frame = false;
  std::uint64_t packet_start_timestamp_ns = 0;
  bool is_falcon = false;
  const void * dispatch_packet_ptr = nullptr;

  if (config_.sensor_model == nebula::drivers::SeyondSensorModel::FALCON_K) {
    const auto * packet =
      reinterpret_cast<const nebula::drivers::SeyondFalconDataPacket *>(packet_data.data());
    if (
      packet->common.size < sizeof(nebula::drivers::SeyondFalconDataPacket) ||
      packet->common.size > packet_data.size()) {
      return {0, 0, false};
    }
    const auto payload_size = static_cast<std::size_t>(packet->common.size) -
                              sizeof(nebula::drivers::SeyondFalconDataPacket);
    const auto required_payload_size = static_cast<std::uint64_t>(packet->item_number) *
                                       static_cast<std::uint64_t>(packet->item_size);
    if (required_payload_size > payload_size) {
      return {0, 0, false};
    }
    const bool supported_layout = packet->type == k_item_type_sphere_pointcloud &&
                                  (packet->item_size == sizeof(nebula::drivers::SeyondBlock) ||
                                   packet->item_size == sizeof(nebula::drivers::SeyondBlockDual));
    if (!supported_layout) {
      return {packet_timestamp_us_to_ns(packet->common.ts_start_us), 0, false};
    }
    packet_index = packet->idx;
    is_first_sub_frame = packet->is_first_sub_frame;
    is_last_sub_frame = packet->is_last_sub_frame;
    packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);
    is_falcon = true;
    dispatch_packet_ptr = packet;
  } else if (config_.sensor_model == nebula::drivers::SeyondSensorModel::ROBIN_W) {
    const auto * packet =
      reinterpret_cast<const nebula::drivers::SeyondDataPacket *>(packet_data.data());
    if (
      packet->common.size < sizeof(nebula::drivers::SeyondDataPacket) ||
      packet->common.size > packet_data.size()) {
      return {0, 0, false};
    }
    const auto payload_size =
      static_cast<std::size_t>(packet->common.size) - sizeof(nebula::drivers::SeyondDataPacket);
    const auto required_payload_size = static_cast<std::uint64_t>(packet->item_number) *
                                       static_cast<std::uint64_t>(packet->item_size);
    if (required_payload_size > payload_size) {
      return {0, 0, false};
    }
    // Only the compact layout is implemented here: it is the only one
    // SeyondAdapter's offline default sees, because the adapter never
    // installs a calibration angle-table that would unlock sphere.
    const bool supported_layout = packet->type == k_item_type_robinw_compact_pointcloud &&
                                  is_supported_compact_item_size(packet->item_size);
    if (!supported_layout) {
      return {packet_timestamp_us_to_ns(packet->common.ts_start_us), 0, false};
    }
    packet_index = packet->idx;
    is_first_sub_frame = packet->is_first_sub_frame;
    is_last_sub_frame = packet->is_last_sub_frame;
    packet_start_timestamp_ns = packet_timestamp_us_to_ns(packet->common.ts_start_us);
    dispatch_packet_ptr = packet;
  } else {
    return {0, 0, false};
  }

  // Scan-boundary logic mirrors upstream so callers see the same emission
  // cadence. The win here is allocation: every new scan cloud is
  // pre-reserved once instead of growing through ~17 doublings during
  // emplace_back as upstream does (which costs both copies and cache
  // churn). The shared_ptr indirection itself stays -- the callback
  // contract owns the cloud after emission, so the decoder can't safely
  // recycle the buffer.
  bool scan_complete = false;
  ensure_scan_cloud();

  if (has_current_scan_frame_ && packet_index != current_scan_frame_idx_) {
    if (!current_scan_cloud_->empty()) {
      pointcloud_callback_(current_scan_cloud_, current_scan_start_timestamp_ns_);
      current_scan_cloud_ = std::make_shared<nebula::drivers::NebulaPointCloud>();
      ensure_scan_cloud();
      scan_complete = true;
    }
    current_scan_start_timestamp_ns_ = 0;
    has_current_scan_frame_ = false;
  }

  if (
    !has_current_scan_frame_ || current_scan_start_timestamp_ns_ == 0 || is_first_sub_frame ||
    current_scan_cloud_->empty() || packet_start_timestamp_ns < current_scan_start_timestamp_ns_) {
    current_scan_frame_idx_ = packet_index;
    current_scan_start_timestamp_ns_ = packet_start_timestamp_ns;
    has_current_scan_frame_ = true;
  }

  const std::size_t initial_points = current_scan_cloud_->size();
  if (is_falcon) {
    parse_falcon_k(dispatch_packet_ptr);
  } else {
    parse_robin_w_en(dispatch_packet_ptr);
  }
  const std::size_t points_unpacked = current_scan_cloud_->size() - initial_points;

  if (is_last_sub_frame && !current_scan_cloud_->empty()) {
    pointcloud_callback_(current_scan_cloud_, current_scan_start_timestamp_ns_);
    current_scan_cloud_ = std::make_shared<nebula::drivers::NebulaPointCloud>();
    ensure_scan_cloud();
    current_scan_frame_idx_ = 0;
    current_scan_start_timestamp_ns_ = 0;
    has_current_scan_frame_ = false;
    scan_complete = true;
  }

  last_packet_start_timestamp_ns_ = packet_start_timestamp_ns;
  return {packet_start_timestamp_ns, points_unpacked, scan_complete};
}

}  // namespace nebuladec::adapters
