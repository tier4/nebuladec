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

#include "nebuladec_adapters/velodyne_adapter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_velodyne_common/velodyne_common.hpp>
#include <nebula_velodyne_decoders/velodyne_driver.hpp>

#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

using nebula::Status;
using nebula::drivers::ReturnMode;
using nebula::drivers::SensorModel;
using nebula::drivers::VelodyneCalibrationConfiguration;
using nebula::drivers::VelodyneDriver;
using nebula::drivers::VelodyneSensorConfiguration;

std::optional<std::string> model_to_file_stem(SensorModel model)
{
  switch (model) {
    case SensorModel::VELODYNE_VLP16:
      return "VLP16";
    case SensorModel::VELODYNE_VLP32:
      return "VLP32";
    case SensorModel::VELODYNE_VLS128:
      return "VLS128";
    case SensorModel::VELODYNE_HDL32:
      return "HDL32";
    case SensorModel::VELODYNE_HDL64:
      // Nebula ships several HDL64 revisions; HDL64e_s3 is the most
      // common factory-default target for this driver.
      return "HDL64e_s3";
    default:
      return std::nullopt;
  }
}

std::shared_ptr<const VelodyneCalibrationConfiguration> load_shipped_calibration(SensorModel model)
{
  const auto stem = model_to_file_stem(model);
  if (!stem) {
    return nullptr;
  }

  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory("nebula_velodyne_decoders");
  } catch (const std::exception &) {
    return nullptr;
  }

  const std::filesystem::path path =
    std::filesystem::path(share_dir) / "calibration" / (*stem + ".yaml");

  auto calibration = std::make_shared<VelodyneCalibrationConfiguration>();
  if (calibration->load_from_file(path.string()) != Status::OK) {
    return nullptr;
  }
  calibration->calibration_file = path.string();
  return calibration;
}

std::shared_ptr<const VelodyneSensorConfiguration> make_offline_config(
  SensorModel model, ReturnMode return_mode)
{
  auto config = std::make_shared<VelodyneSensorConfiguration>();
  config->sensor_model = model;
  config->frame_id = "velodyne";
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
  config->rotation_speed = 600;
  config->cloud_min_angle = 0;
  config->cloud_max_angle = 360;
  return config;
}

}  // namespace

VelodyneAdapter::VelodyneAdapter(const Identity & identity)
: identity_(identity)
{
  if (identity.model == SensorModel::UNKNOWN) {
    return;
  }

  auto calibration = load_shipped_calibration(identity.model);
  if (!calibration) {
    return;
  }

  auto config = make_offline_config(identity.model, identity.return_mode);

  try {
    driver_ = std::make_unique<VelodyneDriver>(config, calibration);
  } catch (const std::exception &) {
    driver_.reset();
  }
}

VelodyneAdapter::~VelodyneAdapter() = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> VelodyneAdapter::feed(
  const std::vector<std::uint8_t> & packet, double stamp_sec)
{
  if (!driver_ || packet.empty()) {
    return std::nullopt;
  }

  auto result = driver_->parse_cloud_packet(packet, stamp_sec);
  auto & cloud = std::get<0>(result);
  if (!cloud || cloud->empty()) {
    return std::nullopt;
  }
  return cloud;
}

}  // namespace nebuladec::adapters
