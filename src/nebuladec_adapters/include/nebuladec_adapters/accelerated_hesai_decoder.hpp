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

#ifndef NEBULADEC_ADAPTERS__ACCELERATED_HESAI_DECODER_HPP_
#define NEBULADEC_ADAPTERS__ACCELERATED_HESAI_DECODER_HPP_

// Parallel re-implementation of `nebula::drivers::HesaiDecoder<SensorT>`
// that conforms to upstream's `nebula::drivers::HesaiScanDecoder` virtual
// interface so it slots into the same callback contract without forking
// nebula. Only the hot decode path is reimplemented; functional-safety,
// packet-loss-detector and blockage-mask plugins are dropped because the
// nebuladec offline-decode workflow never installs them. Output points
// are bit-identical up to floating-point precision.
//
// Differences from upstream's `HesaiDecoder<SensorT>`:
//   * `return_units` is a stack-allocated `std::array` instead of a heap
//     `std::vector`, removing a heap touch per (channel, block group).
//   * `n_returns`, `dis_unit_f`, and `packet_timestamp_ns` are computed
//     once per packet and threaded into `convert_returns` instead of
//     being re-derived inside the per-channel inner loop.
//   * The `n_returns == 1` path bypasses the multi-return duplicate
//     detection entirely. Single-return is the dominant mode in offline
//     decoding (and the only mode the test bag uses).
//
// `AcceleratedHesaiDecoder<SensorT>` is templated on SensorT, matching upstream
// so the per-sensor packet layout (`SensorT::packet_t`) and angle
// corrector (`SensorT::angle_corrector_t`) are picked up by the type
// system. Instantiate via the model dispatcher in `accelerated_hesai_driver.hpp`.

#include "nebula_core_decoders/scan_cutter.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/stopwatch.hpp>  // nebula::util::Stopwatch
#include <nebula_hesai_common/hesai_common.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

template <typename SensorT>
class AcceleratedHesaiDecoder : public nebula::drivers::HesaiScanDecoder
{
public:
  using packet_t = typename SensorT::packet_t;
  using calib_t = typename SensorT::angle_corrector_t::correction_data_t;

  AcceleratedHesaiDecoder(
    std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_configuration,
    std::shared_ptr<const calib_t> correction_data,
    std::shared_ptr<nebula::drivers::loggers::Logger> logger)
  : sensor_configuration_(std::move(sensor_configuration)),
    angle_corrector_(std::move(correction_data)),
    scan_cutter_(
      2 * M_PIf, nebula::drivers::deg2rad(sensor_configuration_->cut_angle),
      nebula::drivers::deg2rad(sensor_configuration_->cloud_min_angle),
      nebula::drivers::deg2rad(sensor_configuration_->cloud_max_angle),
      [this](uint8_t buffer_index) { on_scan_complete(buffer_index); },
      [this](uint8_t buffer_index) { on_set_timestamp(buffer_index); }),
    logger_(std::move(logger)),
    frame_buffers_{initialize_frame(), initialize_frame()}
  {
  }

  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = std::move(callback);
  }

  nebula::drivers::PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    nebula::util::Stopwatch decode_watch;
    callback_time_ns_ = 0;
    did_scan_complete_ = false;

    if (!parse_packet(packet)) {
      return {
        nebula::drivers::PerformanceCounters{decode_watch.elapsed_ns()},
        nebula::drivers::DecodeError::PACKET_PARSE_FAILED};
    }

    const auto return_mode_enum =
      static_cast<nebula::drivers::hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode);
    const std::size_t n_returns =
      nebula::drivers::hesai_packet::get_n_returns(packet_.tail.return_mode);
    const float dis_unit_f =
      static_cast<float>(nebula::drivers::hesai_packet::get_dis_unit(packet_));
    const std::uint64_t packet_timestamp_ns =
      nebula::drivers::hesai_packet::get_timestamp_ns(packet_);

    for (std::size_t block_id = 0; block_id < SensorT::packet_t::n_blocks; block_id += n_returns) {
      const auto block_azimuth = packet_.body.blocks[block_id].get_azimuth();
      const auto channel_azimuths_out = angle_corrector_.get_corrected_azimuths(block_azimuth);
      current_block_id_ = block_id;
      const auto & scan_state = scan_cutter_.step(channel_azimuths_out);
      if (scan_state.does_block_intersect_fov()) {
        convert_returns(
          block_id, n_returns, scan_state, return_mode_enum, dis_unit_f, packet_timestamp_ns);
      }
    }

    const std::uint64_t decode_duration_ns = decode_watch.elapsed_ns();
    nebula::drivers::PacketMetadata metadata;
    metadata.packet_timestamp_ns = packet_timestamp_ns;
    metadata.did_scan_complete = did_scan_complete_;
    return {nebula::drivers::PerformanceCounters{decode_duration_ns - callback_time_ns_}, metadata};
  }

private:
  struct DecodeFrame
  {
    nebula::drivers::NebulaPointCloudPtr pointcloud;
    std::uint64_t scan_timestamp_ns{0};
  };

  DecodeFrame initialize_frame() const
  {
    DecodeFrame frame{std::make_shared<nebula::drivers::NebulaPointCloud>(), 0};
    frame.pointcloud->reserve(SensorT::max_scan_buffer_points);
    return frame;
  }

  bool parse_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(packet_t)) {
      return false;
    }
    std::memcpy(&packet_, packet.data(), sizeof(packet_t));
    return true;
  }

  void convert_returns(
    std::size_t start_block_id, std::size_t n_blocks,
    const typename nebula::drivers::ScanCutter<SensorT::packet_t::n_channels, float>::State &
      scan_state,
    nebula::drivers::hesai_packet::return_mode::ReturnMode return_mode_enum, float dis_unit_f,
    std::uint64_t packet_timestamp_ns)
  {
    constexpr std::size_t k_max_returns = SensorT::packet_t::max_returns;
    std::array<const typename packet_t::body_t::block_t::unit_t *, k_max_returns> return_units{};

    SensorT sensor{};

    for (std::size_t channel_id = 0; channel_id < SensorT::packet_t::n_channels; ++channel_id) {
      // Fill return_units with all returns in this channel's group.
      // Upstream uses `std::vector<unit_t*>` + clear/push_back per
      // channel; the stack array removes the heap allocation and the
      // implicit size bookkeeping.
      for (std::size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        return_units[block_offset] =
          &packet_.body.blocks[block_offset + start_block_id].units[channel_id];
      }

      for (std::size_t block_offset = 0; block_offset < n_blocks; ++block_offset) {
        const auto & unit = *return_units[block_offset];

        bool point_is_valid = unit.distance != 0;
        const float distance = static_cast<float>(unit.distance) * dis_unit_f;

        if (
          distance < SensorT::min_range || SensorT::max_range < distance ||
          distance < sensor_configuration_->min_range ||
          sensor_configuration_->max_range < distance) {
          point_is_valid = false;
        }

        nebula::drivers::ReturnType return_type{nebula::drivers::ReturnType::UNKNOWN};
        if (n_blocks == 1) {
          // Hot path: single-return mode. Inline the only switch arm
          // that fires in `HesaiSensor::get_return_type` and bypass the
          // multi-return duplicate-detection and is-below-threshold
          // gates entirely.
          switch (return_mode_enum) {
            case nebula::drivers::hesai_packet::return_mode::SINGLE_FIRST:
              return_type = nebula::drivers::ReturnType::FIRST;
              break;
            case nebula::drivers::hesai_packet::return_mode::SINGLE_SECOND:
              return_type = nebula::drivers::ReturnType::SECOND;
              break;
            case nebula::drivers::hesai_packet::return_mode::SINGLE_STRONGEST:
              return_type = nebula::drivers::ReturnType::STRONGEST;
              break;
            case nebula::drivers::hesai_packet::return_mode::SINGLE_LAST:
              return_type = nebula::drivers::ReturnType::LAST;
              break;
            default: {
              // Defensive: malformed packet reporting dual/triple mode
              // with `n_blocks == 1`. Delegate to upstream dispatch.
              std::vector<const typename packet_t::body_t::block_t::unit_t *> tmp(
                return_units.begin(), return_units.begin() + n_blocks);
              return_type = sensor.get_return_type(return_mode_enum, block_offset, tmp);
              break;
            }
          }
        } else {
          // Multi-return: preserve upstream semantics. The vector adapter
          // is cheap relative to the dual/triple-return overhead it gates.
          std::vector<const typename packet_t::body_t::block_t::unit_t *> tmp(
            return_units.begin(), return_units.begin() + n_blocks);
          return_type = sensor.get_return_type(return_mode_enum, block_offset, tmp);

          if (
            return_type == nebula::drivers::ReturnType::IDENTICAL && block_offset != n_blocks - 1) {
            point_is_valid = false;
          }
          if (block_offset != n_blocks - 1) {
            bool below_threshold = false;
            for (std::size_t r = 0; r < n_blocks; ++r) {
              if (r == block_offset) continue;
              const float other_distance =
                static_cast<float>(return_units[r]->distance) * dis_unit_f;
              if (
                std::fabs(other_distance - distance) <
                static_cast<float>(sensor_configuration_->dual_return_distance_threshold)) {
                below_threshold = true;
                break;
              }
            }
            if (below_threshold) {
              point_is_valid = false;
            }
          }
        }

        if (!point_is_valid) {
          continue;
        }
        if (!scan_state.channels_in_fov[channel_id]) {
          continue;
        }

        const std::uint32_t raw_azimuth =
          packet_.body.blocks[start_block_id + block_offset].get_azimuth();
        const auto corrected = angle_corrector_.get_corrected_angle_data(raw_azimuth, channel_id);
        auto & frame = frame_buffers_[scan_state.channel_buffer_indices[channel_id]];

        const float xy_distance = distance * corrected.cos_elevation;

        nebula::drivers::NebulaPoint point;
        point.distance = distance;
        point.intensity = unit.reflectivity;
        point.time_stamp = get_point_time_relative(
          frame.scan_timestamp_ns, packet_timestamp_ns,
          static_cast<std::uint32_t>(start_block_id + block_offset),
          static_cast<std::uint32_t>(channel_id), sensor);
        point.return_type = static_cast<std::uint8_t>(return_type);
        point.channel = static_cast<std::uint16_t>(channel_id);
        point.x = xy_distance * corrected.sin_azimuth;
        point.y = xy_distance * corrected.cos_azimuth;
        point.z = distance * corrected.sin_elevation;
        point.azimuth = corrected.azimuth_rad;
        point.elevation = corrected.elevation_rad;

        frame.pointcloud->emplace_back(point);
      }
    }
  }

  std::uint32_t get_point_time_relative(
    std::uint64_t scan_timestamp_ns, std::uint64_t packet_timestamp_ns, std::uint32_t block_id,
    std::uint32_t channel_id, SensorT & sensor)
  {
    const auto point_to_packet_offset_ns =
      sensor.get_packet_relative_point_time_offset(block_id, channel_id, packet_);
    const auto packet_to_scan_offset_ns =
      static_cast<std::uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns + point_to_packet_offset_ns;
  }

  void on_scan_complete(std::uint8_t buffer_index)
  {
    did_scan_complete_ = true;
    auto & completed_frame = frame_buffers_[buffer_index];
    constexpr std::uint64_t ns_per_s = 1'000'000'000ULL;
    const double scan_timestamp_s =
      static_cast<double>(completed_frame.scan_timestamp_ns / ns_per_s) +
      static_cast<double>(completed_frame.scan_timestamp_ns % ns_per_s) / 1e9;
    if (pointcloud_callback_) {
      nebula::util::Stopwatch sw;
      pointcloud_callback_(completed_frame.pointcloud, scan_timestamp_s);
      callback_time_ns_ += sw.elapsed_ns();
    }
    completed_frame.pointcloud->clear();
  }

  void on_set_timestamp(std::uint8_t buffer_index)
  {
    SensorT sensor{};
    auto & frame = frame_buffers_[buffer_index];
    frame.scan_timestamp_ns = nebula::drivers::hesai_packet::get_timestamp_ns(packet_);
    frame.scan_timestamp_ns +=
      sensor.get_earliest_point_time_offset_for_block(current_block_id_, packet_);
  }

  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> sensor_configuration_;
  typename SensorT::angle_corrector_t angle_corrector_;
  nebula::drivers::ScanCutter<SensorT::packet_t::n_channels, float> scan_cutter_;
  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;
  pointcloud_callback_t pointcloud_callback_;

  packet_t packet_;
  std::uint64_t callback_time_ns_{0};
  bool did_scan_complete_{false};
  std::size_t current_block_id_{0};
  std::array<DecodeFrame, 2> frame_buffers_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__ACCELERATED_HESAI_DECODER_HPP_
