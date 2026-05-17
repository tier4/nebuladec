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

#include "nebuladec_adapters/fast_velodyne_driver.hpp"

#include "nebuladec_adapters/fast_velodyne_decoder.hpp"

#include <memory>
#include <tuple>
#include <vector>

namespace nebuladec::adapters
{

bool FastVelodyneDriver::supports(nebula::drivers::SensorModel model) noexcept
{
  switch (model) {
    case nebula::drivers::SensorModel::VELODYNE_VLS128:
    case nebula::drivers::SensorModel::VELODYNE_VLP32:
    case nebula::drivers::SensorModel::VELODYNE_HDL64:
    case nebula::drivers::SensorModel::VELODYNE_HDL32:
    case nebula::drivers::SensorModel::VELODYNE_VLP16:
      return true;
    default:
      return false;
  }
}

FastVelodyneDriver::FastVelodyneDriver(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> &
    calibration_configuration)
{
  switch (sensor_configuration->sensor_model) {
    case nebula::drivers::SensorModel::VELODYNE_VLS128:
      scan_decoder_ = std::make_shared<fast_vls128::FastVls128Decoder>(
        sensor_configuration, calibration_configuration);
      driver_status_ = nebula::Status::OK;
      break;
    case nebula::drivers::SensorModel::VELODYNE_VLP32:
    case nebula::drivers::SensorModel::VELODYNE_HDL64:
    case nebula::drivers::SensorModel::VELODYNE_HDL32:
      scan_decoder_ = std::make_shared<fast_vlp32::FastVlp32Decoder>(
        sensor_configuration, calibration_configuration);
      driver_status_ = nebula::Status::OK;
      break;
    case nebula::drivers::SensorModel::VELODYNE_VLP16:
      scan_decoder_ = std::make_shared<fast_vlp16::FastVlp16Decoder>(
        sensor_configuration, calibration_configuration);
      driver_status_ = nebula::Status::OK;
      break;
    default:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
  }
}

std::tuple<nebula::drivers::NebulaPointCloudPtr, double> FastVelodyneDriver::parse_cloud_packet(
  const std::vector<std::uint8_t> & packet, double packet_seconds)
{
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud;
  if (driver_status_ != nebula::Status::OK || !scan_decoder_) {
    return pointcloud;
  }
  scan_decoder_->unpack(packet, packet_seconds);
  if (scan_decoder_->has_scanned()) {
    pointcloud = scan_decoder_->get_pointcloud();
  }
  return pointcloud;
}

}  // namespace nebuladec::adapters
