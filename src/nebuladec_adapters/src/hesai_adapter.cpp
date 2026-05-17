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

#include "nebuladec_adapters/fast_hesai_driver.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nebula_core_common/loggers/console_logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/hesai_driver.hpp>
#include <nebuladec_core/profiling.hpp>

#include <cstdint>
#include <cstdlib>
#include <cstring>
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

// NEBULADEC_FAST_HESAI opt-out: defaults to enabled. Set to "0" to force
// the upstream nebula::drivers::HesaiDriver; useful for A/B timing
// comparisons or for falling back if FastHesaiDriver regresses on a new
// sensor variant. Read once at construction so the choice is stable for
// the lifetime of the adapter.
bool fast_hesai_opted_out() noexcept
{
  const char * raw = std::getenv("NEBULADEC_FAST_HESAI");
  return raw != nullptr && std::strcmp(raw, "0") == 0;
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

  // Nebula's HesaiDecoder::on_scan_complete clears the underlying vector
  // immediately after invoking this callback, so storing the shared_ptr
  // directly would hand the adapter an empty cloud. Swap the vector
  // contents into an owned shared_ptr instead of copying: swap is O(1)
  // (avoids ~70K point copies per scan on QT128), and we restore the
  // decoder's pre-allocated capacity so the next scan fills without
  // triggering a reallocation.
  auto callback = [this](
                    const nebula::drivers::NebulaPointCloudPtr & cloud, double /*timestamp_s*/) {
    if (!cloud || cloud->empty()) {
      return;
    }
    const auto capacity = cloud->capacity();
    auto owned = std::make_shared<nebula::drivers::NebulaPointCloud>();
    owned->swap(*cloud);
    cloud->reserve(capacity);
    ready_clouds_.push_back(std::move(owned));
  };

  try {
    auto logger = std::make_shared<nebula::drivers::loggers::ConsoleLogger>("nebuladec.hesai");
    if (FastHesaiDriver::supports(identity.model) && !fast_hesai_opted_out()) {
      fast_driver_ = std::make_unique<FastHesaiDriver>(config, calibration, logger, callback);
    } else {
      driver_ = std::make_unique<HesaiDriver>(config, calibration, logger, callback);
    }
  } catch (const std::exception &) {
    driver_.reset();
    fast_driver_.reset();
  }
}

HesaiAdapter::~HesaiAdapter() = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> HesaiAdapter::feed(
  const std::vector<std::uint8_t> & packet, double /*stamp_sec*/)
{
  NEBULADEC_PROFILE_SCOPE("hesai_adapter_feed_total");
  if ((!driver_ && !fast_driver_) || packet.empty()) {
    return std::nullopt;
  }

  if (!first_scan_captured_) {
    first_scan_packets_.push_back(packet);
  }

  if (fast_driver_) {
    NEBULADEC_PROFILE_SCOPE("fast_hesai_driver_parse_cloud_packet");
    fast_driver_->parse_cloud_packet(packet);
  } else {
    NEBULADEC_PROFILE_SCOPE("hesai_driver_parse_cloud_packet");
    driver_->parse_cloud_packet(packet);
  }

  // Stop capturing once the decoder has produced at least one cloud:
  // replaying the packets that led up to (and including) the first cut
  // is sufficient to trigger another cut crossing during flush().
  if (!first_scan_captured_ && !ready_clouds_.empty()) {
    first_scan_captured_ = true;
    first_scan_packets_.shrink_to_fit();
  }

  if (ready_clouds_.empty()) {
    return std::nullopt;
  }
  auto cloud = std::move(ready_clouds_.front());
  ready_clouds_.pop_front();
  return cloud;
}

std::optional<nebula::drivers::NebulaPointCloudPtr> HesaiAdapter::flush()
{
  // Hesai's decoder only emits a scan when the *next* packet crosses
  // the cut angle. At end-of-stream the final scan is buffered inside
  // the driver with no packet left to trigger the crossing. Replaying
  // the first-scan packets reproduces the original cut transition:
  // those packets are known to have triggered at least one cut (that
  // is precisely the condition under which we stopped capturing), so
  // feeding them again guarantees the trailing buffer is emitted.
  if ((!driver_ && !fast_driver_) || first_scan_packets_.empty()) {
    return std::nullopt;
  }
  for (const auto & pkt : first_scan_packets_) {
    if (fast_driver_) {
      fast_driver_->parse_cloud_packet(pkt);
    } else {
      driver_->parse_cloud_packet(pkt);
    }
    if (!ready_clouds_.empty()) {
      break;  // Got the trailing cloud; stop replaying.
    }
  }
  if (ready_clouds_.empty()) {
    return std::nullopt;
  }
  auto cloud = std::move(ready_clouds_.front());
  ready_clouds_.pop_front();
  return cloud;
}

}  // namespace nebuladec::adapters
