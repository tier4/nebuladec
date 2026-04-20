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

#include "nebuladec_adapters/hesai_adapter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nebula_core_common/loggers/console_logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/hesai_driver.hpp>

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

using nebula::Status;
using nebula::drivers::HesaiCalibrationConfiguration;
using nebula::drivers::HesaiCalibrationConfigurationBase;
using nebula::drivers::HesaiCorrection;
using nebula::drivers::HesaiDriver;
using nebula::drivers::HesaiSensorConfiguration;
using nebula::drivers::PtpProfile;
using nebula::drivers::PtpSwitchType;
using nebula::drivers::PtpTransportType;
using nebula::drivers::ReturnMode;
using nebula::drivers::SensorModel;

// Maps a Hesai sensor model to the file stem shipped in
// nebula_hesai_decoders/calibration/.
std::optional<std::string> model_to_file_stem(SensorModel model)
{
  switch (model) {
    case SensorModel::HESAI_PANDAR40P:
      return "Pandar40P";
    case SensorModel::HESAI_PANDAR64:
      return "Pandar64";
    case SensorModel::HESAI_PANDARQT64:
      return "PandarQT64";
    case SensorModel::HESAI_PANDARQT128:
      return "PandarQT128";
    case SensorModel::HESAI_PANDARXT16:
      return "PandarXT16";
    case SensorModel::HESAI_PANDARXT32:
      return "PandarXT32";
    case SensorModel::HESAI_PANDARXT32M:
      return "PandarXT32M";
    case SensorModel::HESAI_PANDARAT128:
      return "PandarAT128";
    case SensorModel::HESAI_PANDAR128_E4X:
      return "Pandar128E4X";
    default:
      return std::nullopt;
  }
}

bool uses_correction_file(SensorModel model)
{
  // AT128 ships a binary .dat correction file instead of the CSV format.
  return model == SensorModel::HESAI_PANDARAT128;
}

std::shared_ptr<const HesaiCalibrationConfigurationBase> load_shipped_calibration(SensorModel model)
{
  const auto stem = model_to_file_stem(model);
  if (!stem) {
    return nullptr;
  }

  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory("nebula_hesai_decoders");
  } catch (const std::exception &) {
    return nullptr;
  }

  const std::filesystem::path calibration_dir = std::filesystem::path(share_dir) / "calibration";

  if (uses_correction_file(model)) {
    auto correction = std::make_shared<HesaiCorrection>();
    const auto path = (calibration_dir / (*stem + ".dat")).string();
    if (correction->load_from_file(path) != Status::OK) {
      return nullptr;
    }
    correction->calibration_file = path;
    return correction;
  }

  auto calibration = std::make_shared<HesaiCalibrationConfiguration>();
  const auto path = (calibration_dir / (*stem + ".csv")).string();
  if (calibration->load_from_file(path) != Status::OK) {
    return nullptr;
  }
  calibration->calibration_file = path;
  return calibration;
}

std::shared_ptr<const HesaiSensorConfiguration> make_offline_config(
  SensorModel model, ReturnMode return_mode)
{
  auto config = std::make_shared<HesaiSensorConfiguration>();
  config->sensor_model = model;
  config->frame_id = "hesai";
  config->host_ip = "";
  config->sensor_ip = "";
  config->data_port = 0;
  config->return_mode =
    return_mode == ReturnMode::UNKNOWN ? ReturnMode::SINGLE_STRONGEST : return_mode;
  config->packet_mtu_size = 1500;
  config->min_range = 0.1;
  config->max_range = 300.0;
  config->use_sensor_time = false;
  config->multicast_ip = "";
  config->gnss_port = 0;
  config->udp_socket_receive_buffer_size_bytes = 0;
  config->sync_angle = 0;
  config->cut_angle = 0.0;
  config->dual_return_distance_threshold = 0.1;
  config->calibration_path = "";
  config->calibration_download_enabled = false;
  config->rotation_speed = 600;
  config->cloud_min_angle = 0;
  config->cloud_max_angle = 360;
  config->ptp_profile = PtpProfile::UNKNOWN_PROFILE;
  config->ptp_domain = 0;
  config->ptp_transport_type = PtpTransportType::UNKNOWN_TRANSPORT;
  config->ptp_switch_type = PtpSwitchType::UNKNOWN_SWITCH;
  config->ptp_lock_threshold = 100;
  config->hires_mode = false;
  return config;
}

}  // namespace

HesaiAdapter::HesaiAdapter(const Identity & identity) : identity_(identity)
{
  if (identity.model == SensorModel::UNKNOWN) {
    return;  // Not enough to pick a decoder/calibration; is_ready() stays false.
  }

  auto calibration = load_shipped_calibration(identity.model);
  if (!calibration) {
    return;
  }

  auto config = make_offline_config(identity.model, identity.return_mode);

  auto callback = [this](
                    const nebula::drivers::NebulaPointCloudPtr & cloud, double /*timestamp_s*/) {
    if (cloud && !cloud->empty()) {
      ready_clouds_.push_back(cloud);
    }
  };

  try {
    auto logger = std::make_shared<nebula::drivers::loggers::ConsoleLogger>("nebuladec.hesai");
    driver_ = std::make_unique<HesaiDriver>(config, calibration, logger, callback);
  } catch (const std::exception &) {
    driver_.reset();
  }
}

HesaiAdapter::~HesaiAdapter() = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> HesaiAdapter::feed(
  const std::vector<std::uint8_t> & packet, double /*stamp_sec*/)
{
  if (!driver_ || packet.empty()) {
    return std::nullopt;
  }

  driver_->parse_cloud_packet(packet);

  if (ready_clouds_.empty()) {
    return std::nullopt;
  }
  auto cloud = std::move(ready_clouds_.front());
  ready_clouds_.pop_front();
  return cloud;
}

}  // namespace nebuladec::adapters
