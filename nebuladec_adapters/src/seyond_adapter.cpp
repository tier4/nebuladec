// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_adapters/seyond_adapter.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

// Offline decoding does not need a live sensor endpoint; these values are
// only stored as metadata and never used by SeyondDecoder::unpack().
nebula::drivers::SeyondConnectionConfiguration make_offline_connection()
{
  nebula::drivers::SeyondConnectionConfiguration connection;
  connection.host_ip = "";
  connection.sensor_ip = "";
  connection.netmask = "";
  connection.gateway = "";
  connection.udp_port = 0;
  connection.udp_message_port = 0;
  connection.udp_status_port = 0;
  return connection;
}

nebula::drivers::SeyondSensorModel pick_seyond_model()
{
  // The Seyond data-packet header does not carry an unambiguous model
  // identifier that Nebula already maps to SeyondSensorModel. Falcon K
  // is Seyond's flagship and the most common deployment; use it as the
  // default until an explicit discriminator lands (see M3 follow-ups).
  return nebula::drivers::SeyondSensorModel::FALCON_K;
}

}  // namespace

SeyondAdapter::SeyondAdapter(const Identity & identity) : identity_(identity)
{
  nebula::drivers::SeyondSensorConfiguration config;
  config.sensor_model = pick_seyond_model();
  config.connection = make_offline_connection();
  config.fov.azimuth = {0.0F, 0.0F};    // full-circle sentinel
  config.fov.elevation = {0.0F, 0.0F};  // full-circle sentinel
  config.use_sensor_time = false;
  config.frame_id = "seyond";
  config.setup_sensor = false;
  config.return_mode = nebula::drivers::ReturnMode::STRONGEST;

  auto callback = [this](
                    nebula::drivers::NebulaPointCloudPtr cloud, std::uint64_t /*timestamp_ns*/) {
    if (cloud && !cloud->empty()) {
      ready_clouds_.push_back(std::move(cloud));
    }
  };

  decoder_ = std::make_unique<nebula::drivers::SeyondDecoder>(
    config, callback, nebula::drivers::SeyondCalibrationData{});
}

SeyondAdapter::~SeyondAdapter() = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> SeyondAdapter::feed(
  const std::vector<std::uint8_t> & packet, double /*stamp_sec*/)
{
  if (!decoder_ || packet.empty()) {
    return std::nullopt;
  }

  decoder_->unpack(packet);

  if (ready_clouds_.empty()) {
    return std::nullopt;
  }
  auto cloud = std::move(ready_clouds_.front());
  ready_clouds_.pop_front();
  return cloud;
}

}  // namespace nebuladec::adapters
