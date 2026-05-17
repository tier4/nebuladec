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

#include "nebuladec_adapters/accelerated_velodyne_decoder.hpp"

#include <angles/angles.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

// Verbatim ports of upstream nebula Velodyne decoders. Each `unpack()` is
// a line-for-line copy of the upstream body with two safe hoists:
//   - Local `const &` aliases for `*sensor_configuration_` and the
//     `velodyne_calibration` block (saves the per-iteration shared_ptr
//     deref).
//   - Per-packet integer scaling of the `cloud_min_angle * 100` and
//     `cloud_max_angle * 100` FOV bounds (precomputed once instead of
//     once per branch evaluation).
// All other math, floating-point conversion order, and emit/skip
// branching follow the upstream source byte-for-byte.

namespace nebuladec::adapters
{

using nebula::drivers::g_blocks_per_packet;
using nebula::drivers::g_lower_bank;
using nebula::drivers::g_raw_scan_size;
using nebula::drivers::g_return_mode_dual;
using nebula::drivers::g_return_mode_index;
using nebula::drivers::g_return_mode_last;
using nebula::drivers::g_return_mode_strongest;
using nebula::drivers::g_rotation_max_units;
using nebula::drivers::g_rotation_resolution;
using nebula::drivers::g_scans_per_block;
using nebula::drivers::g_upper_bank;
using nebula::drivers::g_vlp128_distance_resolution;
using nebula::drivers::g_vlp16_block_duration;
using nebula::drivers::g_vlp16_dsr_toffset;
using nebula::drivers::g_vlp16_firing_toffset;
using nebula::drivers::g_vlp16_firings_per_block;
using nebula::drivers::g_vlp16_scans_per_firing;
using nebula::drivers::g_vlp32_channel_duration;
using nebula::drivers::g_vlp32_seq_duration;
using nebula::drivers::g_vls128_bank_1;
using nebula::drivers::g_vls128_bank_2;
using nebula::drivers::g_vls128_bank_3;
using nebula::drivers::g_vls128_bank_4;
using nebula::drivers::g_vls128_channel_duration;
using nebula::drivers::g_vls128_seq_duration;
using nebula::drivers::NebulaPoint;
using nebula::drivers::NebulaPointCloud;
using nebula::drivers::raw_block_t;
using nebula::drivers::raw_packet_t;
using nebula::drivers::ReturnMode;
using nebula::drivers::ReturnType;
using nebula::drivers::two_bytes;
using nebula::drivers::VelodyneCalibrationConfiguration;
using nebula::drivers::VelodyneLaserCorrection;
using nebula::drivers::VelodyneSensorConfiguration;

namespace
{
// Helper: precompute the integer FOV bounds the upstream decoders
// re-derive on every iteration.
struct FovBoundsHundredths
{
  int min;
  int max;
  bool wraps;
};
inline FovBoundsHundredths fov_bounds(const VelodyneSensorConfiguration & cfg) noexcept
{
  return {
    static_cast<int>(cfg.cloud_min_angle) * 100, static_cast<int>(cfg.cloud_max_angle) * 100,
    cfg.cloud_min_angle > cfg.cloud_max_angle};
}
}  // namespace

// ============================================================================
// AcceleratedVlp16Decoder
// ============================================================================

namespace accelerated_vlp16
{

AcceleratedVlp16Decoder::AcceleratedVlp16Decoder(
  const std::shared_ptr<const VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const VelodyneCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  calibration_configuration_ = calibration_configuration;
  scan_timestamp_ = -1;
  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);
  for (std::uint16_t rot_index = 0; rot_index < g_rotation_max_units; ++rot_index) {
    float rotation = angles::from_degrees(g_rotation_resolution * rot_index);
    rotation_radians_[rot_index] = rotation;
    cos_rot_table_[rot_index] = std::cos(rotation);
    sin_rot_table_[rot_index] = std::sin(rotation);
  }
  timing_offsets_.resize(g_blocks_per_packet);
  for (std::size_t i = 0; i < timing_offsets_.size(); ++i) {
    timing_offsets_[i].resize(32);
  }
  double full_firing_cycle_s = 55.296 * 1e-6;
  double single_firing_s = 2.304 * 1e-6;
  double data_block_index;
  double data_point_index;
  bool dual_mode = sensor_configuration_->return_mode == ReturnMode::DUAL;
  for (std::size_t x = 0; x < timing_offsets_.size(); ++x) {
    for (std::size_t y = 0; y < timing_offsets_[x].size(); ++y) {
      if (dual_mode) {
        data_block_index = (x - (x % 2)) + (y / 16);
      } else {
        data_block_index = (x * 2) + (y / 16);
      }
      data_point_index = y % 16;
      timing_offsets_[x][y] =
        (full_firing_cycle_s * data_block_index) + (single_firing_s * data_point_index);
    }
  }
  phase_ = static_cast<int>(std::round(sensor_configuration_->scan_phase * 100));
}

std::tuple<nebula::drivers::NebulaPointCloudPtr, double> AcceleratedVlp16Decoder::get_pointcloud()
{
  double phase = angles::from_degrees(sensor_configuration_->scan_phase);
  if (!scan_pc_->empty()) {
    auto current_azimuth = scan_pc_->back().azimuth;
    auto phase_diff =
      static_cast<std::size_t>(angles::to_degrees(2 * M_PI + current_azimuth - phase)) % 360;
    while (phase_diff < M_PI_2 && !scan_pc_->empty()) {
      overflow_pc_->push_back(scan_pc_->back());
      scan_pc_->pop_back();
      current_azimuth = scan_pc_->back().azimuth;
      phase_diff =
        static_cast<std::size_t>(angles::to_degrees(2 * M_PI + current_azimuth - phase)) % 360;
    }
  }
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int AcceleratedVlp16Decoder::points_per_packet()
{
  return g_blocks_per_packet * g_vlp16_firings_per_block * g_vlp16_scans_per_firing;
}

void AcceleratedVlp16Decoder::reset_pointcloud(double time_stamp)
{
  scan_pc_->clear();
  reset_overflow(time_stamp);
}

void AcceleratedVlp16Decoder::reset_overflow(double time_stamp)
{
  if (overflow_pc_->size() == 0) {
    scan_timestamp_ = -1;
    overflow_pc_->reserve(max_pts_);
    return;
  }
  const double last_overflow_time_stamp = scan_timestamp_ + 1e-9 * overflow_pc_->back().time_stamp;
  if (time_stamp - last_overflow_time_stamp > 0.05) {
    scan_timestamp_ = -1;
    overflow_pc_->clear();
    overflow_pc_->reserve(max_pts_);
    return;
  }
  while (overflow_pc_->size() > 0) {
    auto overflow_point = overflow_pc_->back();
    double new_timestamp_seconds =
      scan_timestamp_ + 1e-9 * overflow_point.time_stamp - last_block_timestamp_;
    overflow_point.time_stamp =
      static_cast<std::uint32_t>(new_timestamp_seconds < 0.0 ? 0.0 : 1e9 * new_timestamp_seconds);
    scan_pc_->emplace_back(overflow_point);
    overflow_pc_->pop_back();
  }
  scan_timestamp_ = last_block_timestamp_;
  overflow_pc_->clear();
  overflow_pc_->reserve(max_pts_);
}

void AcceleratedVlp16Decoder::unpack(
  const std::vector<std::uint8_t> & packet, double packet_seconds)
{
  check_and_handle_scan_complete(packet, packet_seconds, phase_);

  const auto & sensor_cfg = *sensor_configuration_;
  const auto & calib = calibration_configuration_->velodyne_calibration;
  const auto fov = fov_bounds(sensor_cfg);

  const raw_packet_t * raw = reinterpret_cast<const raw_packet_t *>(packet.data());
  float last_azimuth_diff = 0;
  std::uint16_t azimuth_next = 0;
  const std::uint8_t return_mode = packet[g_return_mode_index];
  const bool dual_return = (return_mode == g_return_mode_dual);

  for (unsigned int block = 0; block < g_blocks_per_packet; block++) {
    const raw_block_t & current_block = raw->blocks[block];
    if (g_upper_bank != raw->blocks[block].header) {
      return;
    }
    float azimuth_diff;
    std::uint16_t azimuth;
    if (block == 0) {
      azimuth = current_block.rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (block < static_cast<unsigned int>(g_blocks_per_packet - (1 + dual_return))) {
      azimuth_next = raw->blocks[block + (1 + dual_return)].rotation;
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff =
        (block == static_cast<unsigned int>(g_blocks_per_packet - dual_return - 1)
           ? 0
           : last_azimuth_diff);
    }

    if ((!fov.wraps && azimuth >= fov.min && azimuth <= fov.max) || fov.wraps) {
      for (int firing = 0, k = 0; firing < g_vlp16_firings_per_block; ++firing) {
        for (int dsr = 0; dsr < g_vlp16_scans_per_firing; dsr++, k += g_raw_scan_size) {
          two_bytes current_return;
          two_bytes other_return;
          current_return.bytes[0] = current_block.data[k];
          current_return.bytes[1] = current_block.data[k + 1];

          if (dual_return) {
            other_return.bytes[0] =
              block % 2 ? raw->blocks[block - 1].data[k] : raw->blocks[block + 1].data[k];
            other_return.bytes[1] =
              block % 2 ? raw->blocks[block - 1].data[k + 1] : raw->blocks[block + 1].data[k + 1];
          }
          auto block_timestamp = packet_seconds;
          if (scan_timestamp_ < 0) {
            scan_timestamp_ = block_timestamp;
          }
          if (
            (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
            (dual_return && block % 2 && other_return.bytes[0] == current_return.bytes[0] &&
             other_return.bytes[1] == current_return.bytes[1])) {
            continue;
          }
          const VelodyneLaserCorrection & corrections = calib.laser_corrections[dsr];
          float distance = current_return.uint * calib.distance_resolution_m;
          if (distance > 1e-6) {
            distance += corrections.dist_correction;
          }
          if (distance > sensor_cfg.min_range && distance < sensor_cfg.max_range) {
            float azimuth_corrected_f =
              azimuth +
              (azimuth_diff * ((dsr * g_vlp16_dsr_toffset) + (firing * g_vlp16_firing_toffset)) /
               g_vlp16_block_duration) -
              corrections.rot_correction * 180.0 / M_PI * 100;
            if (azimuth_corrected_f < 0.0) {
              azimuth_corrected_f += 36000.0;
            }
            const std::uint16_t azimuth_corrected =
              (static_cast<std::uint16_t>(std::round(azimuth_corrected_f))) % 36000;
            if (
              (!fov.wraps && azimuth_corrected >= fov.min && azimuth_corrected <= fov.max) ||
              (fov.wraps && (azimuth_corrected <= fov.max || azimuth_corrected >= fov.min))) {
              const float cos_vert_angle = corrections.cos_vert_correction;
              const float sin_vert_angle = corrections.sin_vert_correction;
              const float cos_rot_angle = cos_rot_table_[azimuth_corrected];
              const float sin_rot_angle = sin_rot_table_[azimuth_corrected];
              const float xy_distance = distance * cos_vert_angle;
              const float x_coord = xy_distance * cos_rot_angle;
              const float y_coord = -(xy_distance * sin_rot_angle);
              const float z_coord = distance * sin_vert_angle;
              const std::uint8_t intensity = current_block.data[k + 2];
              last_block_timestamp_ = block_timestamp;
              double point_time_offset = timing_offsets_[block][firing * 16 + dsr];

              std::uint8_t return_type;
              switch (return_mode) {
                case g_return_mode_dual:
                  if (
                    (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                    (other_return.bytes[0] == current_return.bytes[0] &&
                     other_return.bytes[1] == current_return.bytes[1])) {
                    return_type = static_cast<std::uint8_t>(ReturnType::IDENTICAL);
                  } else {
                    const std::uint8_t other_intensity = block % 2
                                                           ? raw->blocks[block - 1].data[k + 2]
                                                           : raw->blocks[block + 1].data[k + 2];
                    bool first = current_return.uint > other_return.uint;
                    bool strongest = intensity > other_intensity;
                    if (other_intensity == intensity) {
                      strongest = !first;
                    }
                    if (first && strongest) {
                      return_type = static_cast<std::uint8_t>(ReturnType::FIRST_STRONGEST);
                    } else if (!first && strongest) {
                      return_type = static_cast<std::uint8_t>(ReturnType::LAST_STRONGEST);
                    } else if (first && !strongest) {
                      return_type = static_cast<std::uint8_t>(ReturnType::FIRST_WEAK);
                    } else if (!first && !strongest) {
                      return_type = static_cast<std::uint8_t>(ReturnType::LAST_WEAK);
                    } else {
                      return_type = static_cast<std::uint8_t>(ReturnType::UNKNOWN);
                    }
                  }
                  break;
                case g_return_mode_strongest:
                  return_type = static_cast<std::uint8_t>(ReturnType::STRONGEST);
                  break;
                case g_return_mode_last:
                  return_type = static_cast<std::uint8_t>(ReturnType::LAST);
                  break;
                default:
                  return_type = static_cast<std::uint8_t>(ReturnType::UNKNOWN);
              }
              NebulaPoint current_point{};
              current_point.x = x_coord;
              current_point.y = y_coord;
              current_point.z = z_coord;
              current_point.return_type = return_type;
              current_point.channel = corrections.laser_ring;
              current_point.azimuth = rotation_radians_[azimuth_corrected];
              current_point.elevation = sin_vert_angle;
              auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
              if (point_ts < 0) {
                point_ts = 0;
              }
              current_point.time_stamp = static_cast<std::uint32_t>(point_ts * 1e9);
              current_point.intensity = intensity;
              current_point.distance = distance;
              scan_pc_->emplace_back(current_point);
            }
          }
        }
      }
    }
  }
}

bool AcceleratedVlp16Decoder::parse_packet(
  const velodyne_msgs::msg::VelodynePacket & /*velodyne_packet*/)
{
  return false;
}

}  // namespace accelerated_vlp16

// ============================================================================
// AcceleratedVlp32Decoder
// ============================================================================

namespace accelerated_vlp32
{

AcceleratedVlp32Decoder::AcceleratedVlp32Decoder(
  const std::shared_ptr<const VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const VelodyneCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  calibration_configuration_ = calibration_configuration;
  scan_timestamp_ = -1;
  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);
  for (std::uint16_t rot_index = 0; rot_index < g_rotation_max_units; ++rot_index) {
    float rotation = angles::from_degrees(g_rotation_resolution * rot_index);
    rotation_radians_[rot_index] = rotation;
    cos_rot_table_[rot_index] = std::cos(rotation);
    sin_rot_table_[rot_index] = std::sin(rotation);
  }
  phase_ = static_cast<int>(std::round(sensor_configuration_->scan_phase * 100));

  timing_offsets_.resize(12);
  for (std::size_t i = 0; i < timing_offsets_.size(); ++i) {
    timing_offsets_[i].resize(32);
  }
  double full_firing_cycle = 55.296 * 1e-6;
  double single_firing = 2.304 * 1e-6;
  double data_block_index;
  double data_point_index;
  bool dual_mode = sensor_configuration_->return_mode == ReturnMode::DUAL;
  for (std::size_t x = 0; x < timing_offsets_.size(); ++x) {
    for (std::size_t y = 0; y < timing_offsets_[x].size(); ++y) {
      if (dual_mode) {
        data_block_index = x / 2;
      } else {
        data_block_index = x;
      }
      data_point_index = y / 2;
      timing_offsets_[x][y] =
        (full_firing_cycle * data_block_index) + (single_firing * data_point_index);
    }
  }
}

std::tuple<nebula::drivers::NebulaPointCloudPtr, double> AcceleratedVlp32Decoder::get_pointcloud()
{
  double phase = angles::from_degrees(sensor_configuration_->scan_phase);
  if (!scan_pc_->empty()) {
    auto current_azimuth = scan_pc_->back().azimuth;
    auto phase_diff = (2 * M_PI + current_azimuth - phase);
    while (phase_diff < M_PI_2 && !scan_pc_->empty()) {
      overflow_pc_->push_back(scan_pc_->back());
      scan_pc_->pop_back();
      current_azimuth = scan_pc_->back().azimuth;
      phase_diff = (2 * M_PI + current_azimuth - phase);
    }
  }
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int AcceleratedVlp32Decoder::points_per_packet()
{
  return g_blocks_per_packet * g_scans_per_block;
}

void AcceleratedVlp32Decoder::reset_pointcloud(double time_stamp)
{
  scan_pc_->clear();
  reset_overflow(time_stamp);
}

void AcceleratedVlp32Decoder::reset_overflow(double time_stamp)
{
  if (overflow_pc_->size() == 0) {
    scan_timestamp_ = -1;
    overflow_pc_->reserve(max_pts_);
    return;
  }
  const double last_overflow_time_stamp = scan_timestamp_ + 1e-9 * overflow_pc_->back().time_stamp;
  if (time_stamp - last_overflow_time_stamp > 0.05) {
    scan_timestamp_ = -1;
    overflow_pc_->clear();
    overflow_pc_->reserve(max_pts_);
    return;
  }
  while (overflow_pc_->size() > 0) {
    auto overflow_point = overflow_pc_->back();
    double new_timestamp_seconds =
      scan_timestamp_ + 1e-9 * overflow_point.time_stamp - last_block_timestamp_;
    overflow_point.time_stamp =
      static_cast<std::uint32_t>(new_timestamp_seconds < 0.0 ? 0.0 : 1e9 * new_timestamp_seconds);
    scan_pc_->emplace_back(overflow_point);
    overflow_pc_->pop_back();
  }
  scan_timestamp_ = last_block_timestamp_;
  overflow_pc_->clear();
  overflow_pc_->reserve(max_pts_);
}

void AcceleratedVlp32Decoder::unpack(
  const std::vector<std::uint8_t> & packet, double packet_seconds)
{
  check_and_handle_scan_complete(packet, packet_seconds, phase_);

  const auto & sensor_cfg = *sensor_configuration_;
  const auto & calib = calibration_configuration_->velodyne_calibration;
  const auto fov = fov_bounds(sensor_cfg);

  const raw_packet_t * raw = reinterpret_cast<const raw_packet_t *>(packet.data());
  float last_azimuth_diff = 0;
  std::uint16_t azimuth_next = 0;
  std::uint8_t return_mode = packet[g_return_mode_index];
  const bool dual_return = (return_mode == g_return_mode_dual);

  for (unsigned int i = 0; i < g_blocks_per_packet; i++) {
    int bank_origin = 0;
    if (raw->blocks[i].header == g_lower_bank) {
      bank_origin = 32;
    }
    float azimuth_diff;
    std::uint16_t azimuth;
    if (i == 0) {
      azimuth = raw->blocks[i].rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (i < static_cast<unsigned int>(g_blocks_per_packet - (1 + dual_return))) {
      azimuth_next = raw->blocks[i + (1 + dual_return)].rotation;
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = (i == static_cast<unsigned int>(g_blocks_per_packet - (4 * dual_return) - 1))
                       ? 0
                       : last_azimuth_diff;
    }
    for (unsigned int j = 0, k = 0; j < g_scans_per_block; j++, k += g_raw_scan_size) {
      // cppcheck-suppress variableScope ; verbatim from upstream vlp32_decoder.cpp.
      float x;
      // cppcheck-suppress variableScope ; verbatim from upstream vlp32_decoder.cpp.
      float y;
      // cppcheck-suppress variableScope ; verbatim from upstream vlp32_decoder.cpp.
      float z;
      std::uint8_t intensity;
      const std::uint8_t laser_number = j + bank_origin;
      const VelodyneLaserCorrection & corrections = calib.laser_corrections[laser_number];

      const raw_block_t & block = raw->blocks[i];
      two_bytes current_return;
      current_return.bytes[0] = block.data[k];
      current_return.bytes[1] = block.data[k + 1];

      two_bytes other_return;
      if (dual_return) {
        other_return.bytes[0] = i % 2 ? raw->blocks[i - 1].data[k] : raw->blocks[i + 1].data[k];
        other_return.bytes[1] =
          i % 2 ? raw->blocks[i - 1].data[k + 1] : raw->blocks[i + 1].data[k + 1];
      }
      auto block_timestamp = packet_seconds;
      if (scan_timestamp_ < 0) {
        scan_timestamp_ = block_timestamp;
      }
      if (
        (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
        (dual_return && i % 2 && other_return.bytes[0] == current_return.bytes[0] &&
         other_return.bytes[1] == current_return.bytes[1])) {
        continue;
      }
      float distance = current_return.uint * calib.distance_resolution_m;
      if (distance > 1e-6) {
        distance += corrections.dist_correction;
      }
      if (distance > sensor_cfg.min_range && distance < sensor_cfg.max_range) {
        if (
          (!fov.wraps && block.rotation >= fov.min && block.rotation <= fov.max) ||
          (fov.wraps &&
           (raw->blocks[i].rotation <= fov.max || raw->blocks[i].rotation >= fov.min))) {
          const float cos_vert_angle = corrections.cos_vert_correction;
          const float sin_vert_angle = corrections.sin_vert_correction;
          float azimuth_corrected_f =
            azimuth + (azimuth_diff * g_vlp32_channel_duration / g_vlp32_seq_duration * j) -
            corrections.rot_correction * 180.0 / M_PI * 100;
          if (azimuth_corrected_f < 0) {
            azimuth_corrected_f += 36000;
          }
          const std::uint16_t azimuth_corrected =
            (static_cast<std::uint16_t>(std::round(azimuth_corrected_f))) % 36000;
          const float cos_rot_angle = cos_rot_table_[azimuth_corrected];
          const float sin_rot_angle = sin_rot_table_[azimuth_corrected];
          const float horiz_offset = corrections.horiz_offset_correction;
          const float vert_offset = corrections.vert_offset_correction;
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) {
            xx = -xx;
          }
          if (yy < 0) {
            yy = -yy;
          }
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                                (xx - 2.4) / (25.04 - 2.4) +
                              corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                                (yy - 1.93) / (25.04 - 1.93) +
                              corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
          const float distance_x = distance + distance_corr_x;
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          const float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          const float x_coord = y;
          const float y_coord = -x;
          const float z_coord = z;
          const float min_intensity = corrections.min_intensity;
          const float max_intensity = corrections.max_intensity;
          intensity = raw->blocks[i].data[k + 2];
          last_block_timestamp_ = block_timestamp;

          const float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                                     (1 - corrections.focal_distance / 13100);
          const float focal_slope = corrections.focal_slope;
          float sqr = (1 - static_cast<float>(current_return.uint) / 65535) *
                      (1 - static_cast<float>(current_return.uint) / 65535);
          intensity += focal_slope * (std::abs(focal_offset - 256 * sqr));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;
          double point_time_offset = timing_offsets_[i][j];

          ReturnType return_type;
          switch (return_mode) {
            case g_return_mode_dual:
              if (
                (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                (other_return.bytes[0] == current_return.bytes[0] &&
                 other_return.bytes[1] == current_return.bytes[1])) {
                return_type = ReturnType::IDENTICAL;
              } else {
                const float other_intensity =
                  i % 2 ? raw->blocks[i - 1].data[k + 2] : raw->blocks[i + 1].data[k + 2];
                // Equivalent to upstream's `other.uint < current.uint ? 0 : 1`
                // and similar, spelled without integer-to-bool ternaries so
                // modernize-use-bool-literals / readability-simplify-boolean-expr
                // pass cleanly.
                bool first = !(other_return.uint < current_return.uint);
                bool strongest = other_intensity < intensity;
                if (other_intensity == intensity) {
                  strongest = !first;
                }
                if (first && strongest) {
                  return_type = ReturnType::FIRST_STRONGEST;
                } else if (!first && strongest) {
                  return_type = ReturnType::LAST_STRONGEST;
                } else if (first && !strongest) {
                  return_type = ReturnType::FIRST_WEAK;
                } else if (!first && !strongest) {
                  return_type = ReturnType::LAST_WEAK;
                } else {
                  return_type = ReturnType::UNKNOWN;
                }
              }
              break;
            case g_return_mode_strongest:
              return_type = ReturnType::STRONGEST;
              break;
            case g_return_mode_last:
              return_type = ReturnType::LAST;
              break;
            default:
              return_type = ReturnType::UNKNOWN;
          }
          NebulaPoint current_point{};
          current_point.x = x_coord;
          current_point.y = y_coord;
          current_point.z = z_coord;
          current_point.return_type = static_cast<std::uint8_t>(return_type);
          current_point.channel = corrections.laser_ring;
          current_point.azimuth = rotation_radians_[block.rotation];
          current_point.elevation = sin_vert_angle;
          auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
          if (point_ts < 0) {
            point_ts = 0;
          }
          current_point.time_stamp = static_cast<std::uint32_t>(point_ts * 1e9);
          current_point.distance = distance;
          current_point.intensity = intensity;
          scan_pc_->emplace_back(current_point);
        }
      }
    }
  }
}

bool AcceleratedVlp32Decoder::parse_packet(
  const velodyne_msgs::msg::VelodynePacket & /*velodyne_packet*/)
{
  return false;
}

}  // namespace accelerated_vlp32

// ============================================================================
// AcceleratedVls128Decoder
// ============================================================================

namespace accelerated_vls128
{

AcceleratedVls128Decoder::AcceleratedVls128Decoder(
  const std::shared_ptr<const VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const VelodyneCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  calibration_configuration_ = calibration_configuration;
  scan_timestamp_ = -1;
  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);
  for (std::uint16_t rot_index = 0; rot_index < g_rotation_max_units; ++rot_index) {
    float rotation = angles::from_degrees(g_rotation_resolution * rot_index);
    rotation_radians_[rot_index] = rotation;
    cos_rot_table_[rot_index] = std::cos(rotation);
    sin_rot_table_[rot_index] = std::sin(rotation);
  }
  phase_ = static_cast<int>(std::round(sensor_configuration_->scan_phase * 100));
  for (std::uint8_t i = 0; i < 16; i++) {
    vls_128_laser_azimuth_cache_[i] =
      (g_vls128_channel_duration / g_vls128_seq_duration) * (i + i / 8);
  }
  timing_offsets_.resize(3);
  for (std::size_t i = 0; i < timing_offsets_.size(); ++i) {
    timing_offsets_[i].resize(17);
  }
  double full_firing_cycle_s = 53.3 * 1e-6;
  double single_firing_s = 2.665 * 1e-6;
  double offset_packet_time = 8.7 * 1e-6;
  for (std::size_t x = 0; x < timing_offsets_.size(); ++x) {
    for (std::size_t y = 0; y < timing_offsets_[x].size(); ++y) {
      double sequence_index = x;
      double firing_group_index = y;
      timing_offsets_[x][y] = (full_firing_cycle_s * sequence_index) +
                              (single_firing_s * firing_group_index) - offset_packet_time;
    }
  }
}

std::tuple<nebula::drivers::NebulaPointCloudPtr, double> AcceleratedVls128Decoder::get_pointcloud()
{
  double phase = angles::from_degrees(sensor_configuration_->scan_phase);
  if (!scan_pc_->empty()) {
    auto current_azimuth = scan_pc_->back().azimuth;
    auto phase_diff =
      static_cast<std::size_t>(angles::to_degrees(2 * M_PI + current_azimuth - phase)) % 360;
    while (phase_diff < M_PI_2 && !scan_pc_->empty()) {
      overflow_pc_->push_back(scan_pc_->back());
      scan_pc_->pop_back();
      current_azimuth = scan_pc_->back().azimuth;
      phase_diff =
        static_cast<std::size_t>(angles::to_degrees(2 * M_PI + current_azimuth - phase)) % 360;
    }
  }
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int AcceleratedVls128Decoder::points_per_packet()
{
  return g_blocks_per_packet * g_scans_per_block;
}

void AcceleratedVls128Decoder::reset_pointcloud(double time_stamp)
{
  scan_pc_->clear();
  reset_overflow(time_stamp);
}

void AcceleratedVls128Decoder::reset_overflow(double time_stamp)
{
  if (overflow_pc_->size() == 0) {
    scan_timestamp_ = -1;
    overflow_pc_->reserve(max_pts_);
    return;
  }
  const double last_overflow_time_stamp = scan_timestamp_ + 1e-9 * overflow_pc_->back().time_stamp;
  if (time_stamp - last_overflow_time_stamp > 0.05) {
    scan_timestamp_ = -1;
    overflow_pc_->clear();
    overflow_pc_->reserve(max_pts_);
    return;
  }
  while (overflow_pc_->size() > 0) {
    auto overflow_point = overflow_pc_->back();
    double new_timestamp_seconds =
      scan_timestamp_ + 1e-9 * overflow_point.time_stamp - last_block_timestamp_;
    overflow_point.time_stamp =
      static_cast<std::uint32_t>(new_timestamp_seconds < 0.0 ? 0.0 : 1e9 * new_timestamp_seconds);
    scan_pc_->emplace_back(overflow_point);
    overflow_pc_->pop_back();
  }
  scan_timestamp_ = last_block_timestamp_;
  overflow_pc_->clear();
  overflow_pc_->reserve(max_pts_);
}

void AcceleratedVls128Decoder::unpack(
  const std::vector<std::uint8_t> & packet, double packet_seconds)
{
  check_and_handle_scan_complete(packet, packet_seconds, phase_);

  const auto & sensor_cfg = *sensor_configuration_;
  const auto & calib = calibration_configuration_->velodyne_calibration;
  const auto fov = fov_bounds(sensor_cfg);

  const raw_packet_t * raw = reinterpret_cast<const raw_packet_t *>(packet.data());
  float last_azimuth_diff = 0;
  std::uint16_t azimuth_next = 0;
  const std::uint8_t return_mode = packet[g_return_mode_index];
  const bool dual_return = (return_mode == g_return_mode_dual);

  for (unsigned int block = 0;
       block < static_cast<unsigned int>(g_blocks_per_packet - (4 * dual_return)); block++) {
    const raw_block_t & current_block = raw->blocks[block];
    unsigned int bank_origin = 0;
    switch (current_block.header) {
      case g_vls128_bank_1:
        bank_origin = 0;
        break;
      case g_vls128_bank_2:
        bank_origin = 32;
        break;
      case g_vls128_bank_3:
        bank_origin = 64;
        break;
      case g_vls128_bank_4:
        bank_origin = 96;
        break;
      default:
        return;
    }
    float azimuth_diff;
    std::uint16_t azimuth;
    if (block == 0) {
      azimuth = current_block.rotation;
    } else {
      azimuth = azimuth_next;
    }
    if (block < static_cast<unsigned int>(g_blocks_per_packet - (1 + dual_return))) {
      azimuth_next = raw->blocks[block + (1 + dual_return)].rotation;
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff =
        (block == static_cast<unsigned int>(g_blocks_per_packet - (4 * dual_return) - 1))
          ? 0
          : last_azimuth_diff;
    }

    if ((!fov.wraps && azimuth >= fov.min && azimuth <= fov.max) || fov.wraps) {
      for (std::size_t j = 0, k = 0; j < g_scans_per_block; j++, k += g_raw_scan_size) {
        two_bytes current_return{};
        two_bytes other_return{};
        current_return.bytes[0] = current_block.data[k];
        current_return.bytes[1] = current_block.data[k + 1];
        if (dual_return) {
          other_return.bytes[0] =
            block % 2 ? raw->blocks[block - 1].data[k] : raw->blocks[block + 1].data[k];
          other_return.bytes[1] =
            block % 2 ? raw->blocks[block - 1].data[k + 1] : raw->blocks[block + 1].data[k + 1];
        }
        auto block_timestamp = packet_seconds;
        if (scan_timestamp_ < 0) {
          scan_timestamp_ = block_timestamp;
        }
        if (
          (current_return.bytes[0] == 0 && current_return.bytes[1] == 0) ||
          (dual_return && block % 2 && other_return.bytes[0] == current_return.bytes[0] &&
           other_return.bytes[1] == current_return.bytes[1])) {
          continue;
        }
        const unsigned int laser_number = j + bank_origin;
        const unsigned int firing_order = laser_number / 8;
        const VelodyneLaserCorrection & corrections = calib.laser_corrections[laser_number];

        float distance = current_return.uint * g_vlp128_distance_resolution;
        if (distance > 1e-6) {
          distance += corrections.dist_correction;
        }
        float azimuth_corrected_f = azimuth +
                                    (azimuth_diff * vls_128_laser_azimuth_cache_[firing_order]) -
                                    corrections.rot_correction * 180.0 / M_PI * 100;
        if (azimuth_corrected_f < 0.0) {
          azimuth_corrected_f += 36000.0;
        }
        const std::uint16_t azimuth_corrected =
          (static_cast<std::uint16_t>(std::round(azimuth_corrected_f))) % 36000;

        if (distance > sensor_cfg.min_range && distance < sensor_cfg.max_range) {
          if (
            (!fov.wraps && azimuth_corrected >= fov.min && azimuth_corrected <= fov.max) ||
            (fov.wraps && (azimuth_corrected <= fov.max || azimuth_corrected >= fov.min))) {
            const float cos_vert_angle = corrections.cos_vert_correction;
            const float sin_vert_angle = corrections.sin_vert_correction;
            const float cos_rot_angle = cos_rot_table_[azimuth_corrected];
            const float sin_rot_angle = sin_rot_table_[azimuth_corrected];
            const float xy_distance = distance * cos_vert_angle;
            const float x_coord = xy_distance * cos_rot_angle;
            const float y_coord = -(xy_distance * sin_rot_angle);
            const float z_coord = distance * sin_vert_angle;
            const std::uint8_t intensity = current_block.data[k + 2];
            last_block_timestamp_ = block_timestamp;
            double point_time_offset = timing_offsets_[block / 4][firing_order + laser_number / 64];

            std::uint8_t return_type;
            switch (return_mode) {
              case g_return_mode_dual:
                if (
                  (other_return.bytes[0] == 0 && other_return.bytes[1] == 0) ||
                  (other_return.bytes[0] == current_return.bytes[0] &&
                   other_return.bytes[1] == current_return.bytes[1])) {
                  return_type = static_cast<std::uint8_t>(ReturnType::IDENTICAL);
                } else {
                  const float other_intensity = block % 2 ? raw->blocks[block - 1].data[k + 2]
                                                          : raw->blocks[block + 1].data[k + 2];
                  bool first = other_return.uint >= current_return.uint;
                  bool strongest = other_intensity < intensity;
                  if (other_intensity == intensity) {
                    strongest = !first;
                  }
                  if (first && strongest) {
                    return_type = static_cast<std::uint8_t>(ReturnType::FIRST_STRONGEST);
                  } else if (!first && strongest) {
                    return_type = static_cast<std::uint8_t>(ReturnType::LAST_STRONGEST);
                  } else if (first && !strongest) {
                    return_type = static_cast<std::uint8_t>(ReturnType::FIRST_WEAK);
                  } else if (!first && !strongest) {
                    return_type = static_cast<std::uint8_t>(ReturnType::LAST_WEAK);
                  } else {
                    return_type = static_cast<std::uint8_t>(ReturnType::UNKNOWN);
                  }
                }
                break;
              case g_return_mode_strongest:
                return_type = static_cast<std::uint8_t>(ReturnType::STRONGEST);
                break;
              case g_return_mode_last:
                return_type = static_cast<std::uint8_t>(ReturnType::LAST);
                break;
              default:
                return_type = static_cast<std::uint8_t>(ReturnType::UNKNOWN);
            }
            NebulaPoint current_point{};
            current_point.x = x_coord;
            current_point.y = y_coord;
            current_point.z = z_coord;
            current_point.return_type = return_type;
            current_point.channel = corrections.laser_ring;
            current_point.azimuth = rotation_radians_[azimuth_corrected];
            current_point.elevation = sin_vert_angle;
            current_point.distance = distance;
            auto point_ts = block_timestamp - scan_timestamp_ + point_time_offset;
            if (point_ts < 0) {
              point_ts = 0;
            }
            current_point.time_stamp = static_cast<std::uint32_t>(point_ts * 1e9);
            current_point.intensity = intensity;
            scan_pc_->emplace_back(current_point);
          }
        }
      }
    }
  }
}

bool AcceleratedVls128Decoder::parse_packet(
  const velodyne_msgs::msg::VelodynePacket & /*velodyne_packet*/)
{
  return false;
}

}  // namespace accelerated_vls128

}  // namespace nebuladec::adapters
