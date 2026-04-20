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

#include "nebuladec_adapters/robosense_adapter.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_robosense_common/robosense_common.hpp>
#include <nebula_robosense_decoders/robosense_driver.hpp>
#include <nebula_robosense_decoders/robosense_info_driver.hpp>

#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

using nebula::Status;
using nebula::drivers::ReturnMode;
using nebula::drivers::RobosenseCalibrationConfiguration;
using nebula::drivers::RobosenseDriver;
using nebula::drivers::RobosenseInfoDriver;
using nebula::drivers::RobosenseSensorConfiguration;
using nebula::drivers::SensorModel;

std::shared_ptr<RobosenseSensorConfiguration> make_offline_config(
  SensorModel model, ReturnMode return_mode)
{
  auto config = std::make_shared<RobosenseSensorConfiguration>();
  config->sensor_model = model;
  config->frame_id = "robosense";
  config->host_ip = "";
  config->sensor_ip = "";
  config->data_port = 0;
  config->return_mode =
    return_mode == ReturnMode::UNKNOWN ? ReturnMode::SINGLE_STRONGEST : return_mode;
  config->packet_mtu_size = 1500;
  config->min_range = 0.1;
  config->max_range = 300.0;
  config->use_sensor_time = false;
  config->gnss_port = 0;
  config->scan_phase = 0.0;
  config->dual_return_distance_threshold = 0.1;
  return config;
}

}  // namespace

RobosenseAdapter::RobosenseAdapter(const Identity & identity) : identity_(identity)
{
  if (identity.vendor != Vendor::ROBOSENSE || identity.model == SensorModel::UNKNOWN) {
    return;
  }

  config_ = make_offline_config(identity.model, identity.return_mode);
  try {
    info_driver_ = std::make_unique<RobosenseInfoDriver>(
      std::static_pointer_cast<const RobosenseSensorConfiguration>(config_));
  } catch (const std::exception &) {
    info_driver_.reset();
  }
}

RobosenseAdapter::~RobosenseAdapter() = default;

void RobosenseAdapter::feed_info(const std::vector<std::uint8_t> & packet)
{
  if (!info_driver_ || driver_ || packet.empty()) {
    return;
  }

  if (info_driver_->decode_info_packet(packet) != Status::OK) {
    return;
  }

  // Pull calibration + return mode out of the info packet and build the
  // scan driver. This mirrors robosense_ros_wrapper.cpp's lazy init.
  auto calibration = info_driver_->get_sensor_calibration();
  calibration.create_corrected_channels();
  auto calibration_ptr =
    std::make_shared<const RobosenseCalibrationConfiguration>(std::move(calibration));

  auto updated = std::make_shared<RobosenseSensorConfiguration>(*config_);
  updated->return_mode = info_driver_->get_return_mode();
  updated->use_sensor_time = info_driver_->get_sync_status();
  config_ = updated;

  try {
    driver_ = std::make_unique<RobosenseDriver>(
      std::static_pointer_cast<const RobosenseSensorConfiguration>(config_), calibration_ptr);
    identity_.return_mode = config_->return_mode;
  } catch (const std::exception &) {
    driver_.reset();
  }
}

std::optional<nebula::drivers::NebulaPointCloudPtr> RobosenseAdapter::feed(
  const std::vector<std::uint8_t> & packet, double /*stamp_sec*/)
{
  if (!driver_ || packet.empty()) {
    return std::nullopt;
  }

  if (!first_scan_captured_) {
    first_scan_packets_.push_back(packet);
  }

  auto result = driver_->parse_cloud_packet(packet);
  auto & cloud = std::get<0>(result);
  const bool emitted = cloud && !cloud->empty();

  if (!first_scan_captured_ && emitted) {
    first_scan_captured_ = true;
    first_scan_packets_.shrink_to_fit();
  }

  if (!emitted) {
    return std::nullopt;
  }
  return cloud;
}

std::optional<nebula::drivers::NebulaPointCloudPtr> RobosenseAdapter::flush()
{
  // RobosenseDecoder swaps `decode_pc_` into `output_pc_` only when the
  // angle corrector detects a scan-phase crossing on the *next* packet.
  // At end-of-bag the trailing scan never swaps out. Replaying the
  // cached first-scan MSOPs reproduces the original crossing (those
  // packets are known to have triggered one), and the driver surfaces
  // the trailing cloud on the crossing packet.
  if (!driver_ || first_scan_packets_.empty()) {
    return std::nullopt;
  }
  for (const auto & pkt : first_scan_packets_) {
    auto result = driver_->parse_cloud_packet(pkt);
    auto & cloud = std::get<0>(result);
    if (cloud && !cloud->empty()) {
      return cloud;
    }
  }
  return std::nullopt;
}

}  // namespace nebuladec::adapters
