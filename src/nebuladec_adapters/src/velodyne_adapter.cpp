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

#include "nebuladec_adapters/fast_velodyne_driver.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_velodyne_common/velodyne_common.hpp>
#include <nebula_velodyne_decoders/velodyne_driver.hpp>
#include <nebuladec_core/profiling.hpp>

#include <cstdint>
#include <cstdlib>
#include <cstring>
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

// NEBULADEC_FAST_VELODYNE opt-out: defaults to enabled. Set to "0" to
// force the upstream nebula::drivers::VelodyneDriver. Useful for A/B
// timing comparisons and for falling back if a future change to the
// fast decoders regresses output identity.
bool fast_velodyne_opted_out() noexcept
{
  const char * raw = std::getenv("NEBULADEC_FAST_VELODYNE");
  return raw != nullptr && std::strcmp(raw, "0") == 0;
}

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

VelodyneAdapter::VelodyneAdapter(const Identity & identity) : identity_(identity)
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
    if (FastVelodyneDriver::supports(identity.model) && !fast_velodyne_opted_out()) {
      fast_driver_ = std::make_unique<FastVelodyneDriver>(config, calibration);
    } else {
      driver_ = std::make_unique<VelodyneDriver>(config, calibration);
    }
  } catch (const std::exception &) {
    driver_.reset();
    fast_driver_.reset();
  }
}

VelodyneAdapter::~VelodyneAdapter() = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> VelodyneAdapter::feed(
  const std::vector<std::uint8_t> & packet, double stamp_sec)
{
  NEBULADEC_PROFILE_SCOPE("velodyne_adapter_feed_total");
  if ((!driver_ && !fast_driver_) || packet.empty()) {
    return std::nullopt;
  }

  if (!first_scan_captured_) {
    first_scan_packets_.emplace_back(packet, stamp_sec);
  }

  auto result = [&] {
    if (fast_driver_) {
      NEBULADEC_PROFILE_SCOPE("fast_velodyne_driver_parse_cloud_packet");
      return fast_driver_->parse_cloud_packet(packet, stamp_sec);
    }
    NEBULADEC_PROFILE_SCOPE("velodyne_driver_parse_cloud_packet");
    return driver_->parse_cloud_packet(packet, stamp_sec);
  }();
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

std::optional<nebula::drivers::NebulaPointCloudPtr> VelodyneAdapter::flush()
{
  // Velodyne's scan decoder returns the previous scan only when the
  // current packet's azimuth wraps past the scan phase. At end-of-bag
  // there is no next packet to trigger the wrap, so the last scan sits
  // in `scan_pc_` forever. Replaying the first-scan packets reproduces
  // the original wrap transition (they are known to have triggered one,
  // which is why we stopped capturing) and lets the driver emit the
  // trailing cloud.
  if ((!driver_ && !fast_driver_) || first_scan_packets_.empty()) {
    return std::nullopt;
  }
  for (const auto & [pkt, stamp] : first_scan_packets_) {
    auto result = fast_driver_ ? fast_driver_->parse_cloud_packet(pkt, stamp)
                               : driver_->parse_cloud_packet(pkt, stamp);
    auto & cloud = std::get<0>(result);
    if (cloud && !cloud->empty()) {
      return cloud;
    }
  }
  return std::nullopt;
}

}  // namespace nebuladec::adapters
