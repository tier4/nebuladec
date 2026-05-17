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

#ifndef NEBULADEC_ADAPTERS__FAST_VELODYNE_DECODER_HPP_
#define NEBULADEC_ADAPTERS__FAST_VELODYNE_DECODER_HPP_

// Verbatim ports of upstream Velodyne decoders (VLP16, VLP32, VLS128)
// implemented as parallel classes that inherit upstream's
// `nebula::drivers::VelodyneScanDecoder` virtual interface, so they slot
// into the same construction and callback contract without forking.
//
// **Output identity is the top priority for this family** because there
// is no Velodyne sample bag available to A/B against. The
// implementations mirror the upstream `unpack()` logic line-for-line
// except for safe hoisting (cache `sensor_configuration_` / calibration
// dereferences into local `const &` bindings, precompute *100 angle
// bounds once per packet). These changes are observable-behaviour-
// preserving: a `const &` is just an alias, and the multiplied bounds
// are integers that bit-for-bit match the per-iteration computation.

#include "nebula_velodyne_decoders/decoders/velodyne_scan_decoder.hpp"

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

namespace nebuladec::adapters
{

namespace fast_vlp16
{

constexpr std::uint32_t k_max_points = 300000;

class FastVlp16Decoder : public nebula::drivers::VelodyneScanDecoder
{
public:
  FastVlp16Decoder(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> &
      sensor_configuration,
    const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> &
      calibration_configuration);
  void unpack(const std::vector<std::uint8_t> & packet, double packet_seconds) override;
  int points_per_packet() override;
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> get_pointcloud() override;
  void reset_pointcloud(double time_stamp) override;
  void reset_overflow(double time_stamp) override;

private:
  bool parse_packet(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  std::array<float, nebula::drivers::g_rotation_max_units> sin_rot_table_{};
  std::array<float, nebula::drivers::g_rotation_max_units> cos_rot_table_{};
  std::array<float, nebula::drivers::g_rotation_max_units> rotation_radians_{};
  int phase_{0};
  int max_pts_{static_cast<int>(k_max_points)};
  double last_block_timestamp_{0.0};
  std::vector<std::vector<float>> timing_offsets_;
};

}  // namespace fast_vlp16

namespace fast_vlp32
{

constexpr std::uint32_t k_max_points = 300000;

class FastVlp32Decoder : public nebula::drivers::VelodyneScanDecoder
{
public:
  FastVlp32Decoder(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> &
      sensor_configuration,
    const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> &
      calibration_configuration);
  void unpack(const std::vector<std::uint8_t> & packet, double packet_seconds) override;
  int points_per_packet() override;
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> get_pointcloud() override;
  void reset_pointcloud(double time_stamp) override;
  void reset_overflow(double time_stamp) override;

private:
  bool parse_packet(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  std::array<float, nebula::drivers::g_rotation_max_units> sin_rot_table_{};
  std::array<float, nebula::drivers::g_rotation_max_units> cos_rot_table_{};
  std::array<float, nebula::drivers::g_rotation_max_units> rotation_radians_{};
  int phase_{0};
  int max_pts_{static_cast<int>(k_max_points)};
  double last_block_timestamp_{0.0};
  std::vector<std::vector<float>> timing_offsets_;
};

}  // namespace fast_vlp32

namespace fast_vls128
{

constexpr std::uint32_t k_max_points = 300000;

class FastVls128Decoder : public nebula::drivers::VelodyneScanDecoder
{
public:
  FastVls128Decoder(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> &
      sensor_configuration,
    const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> &
      calibration_configuration);
  void unpack(const std::vector<std::uint8_t> & packet, double packet_seconds) override;
  int points_per_packet() override;
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> get_pointcloud() override;
  void reset_pointcloud(double time_stamp) override;
  void reset_overflow(double time_stamp) override;

private:
  bool parse_packet(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  std::array<float, nebula::drivers::g_rotation_max_units> sin_rot_table_{};
  std::array<float, nebula::drivers::g_rotation_max_units> cos_rot_table_{};
  std::array<float, nebula::drivers::g_rotation_max_units> rotation_radians_{};
  std::array<float, 16> vls_128_laser_azimuth_cache_{};
  int phase_{0};
  int max_pts_{static_cast<int>(k_max_points)};
  double last_block_timestamp_{0.0};
  std::vector<std::vector<float>> timing_offsets_;
};

}  // namespace fast_vls128

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__FAST_VELODYNE_DECODER_HPP_
