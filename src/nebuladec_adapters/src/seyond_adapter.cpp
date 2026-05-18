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

#include "nebuladec_adapters/seyond_adapter.hpp"

#include "seyond_decoder.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>
#include <nebuladec_core/profiling.hpp>

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

nebula::drivers::SeyondSensorModel pick_seyond_model(const Identity & identity)
{
  // Prefer the sub-model the sniffer recovered from the packet header's
  // `lidar_type` byte. Fall back to Falcon K - Seyond's flagship - when
  // the stream uses a variant Nebula does not model (e.g. RobinE2X).
  if (
    identity.seyond_model &&
    *identity.seyond_model != nebula::drivers::SeyondSensorModel::UNKNOWN) {
    return *identity.seyond_model;
  }
  return nebula::drivers::SeyondSensorModel::FALCON_K;
}

}  // namespace

SeyondAdapter::SeyondAdapter(const Identity & identity) : identity_(identity)
{
  nebula::drivers::SeyondSensorConfiguration config;
  config.sensor_model = pick_seyond_model(identity);
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

  decoder_ = std::make_unique<SeyondDecoder>(std::move(config), callback);
}

SeyondAdapter::~SeyondAdapter() = default;

std::optional<nebula::drivers::NebulaPointCloudPtr> SeyondAdapter::feed(
  const std::vector<std::uint8_t> & packet, double /*stamp_sec*/)
{
  NEBULADEC_PROFILE_SCOPE("seyond_adapter_feed_total");
  if (!decoder_ || packet.empty()) {
    return std::nullopt;
  }

  if (!first_scan_captured_) {
    first_scan_packets_.push_back(packet);
  }

  nebula::drivers::SeyondPacketDecodeResult result{};
  {
    NEBULADEC_PROFILE_SCOPE("seyond_decoder_unpack");
    result = decoder_->unpack(packet);
  }
  last_feed_scan_complete_ = result.scan_complete;

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

std::optional<nebula::drivers::NebulaPointCloudPtr> SeyondAdapter::flush()
{
  // SeyondDecoder emits the current scan either when the packet's
  // frame_idx differs from the accumulating scan's (the "next-packet"
  // path, same failure mode as mechanical LiDARs) or when the packet
  // carries is_last_sub_frame=true (a self-signal that dodges the
  // failure when the bag ends on a clean frame boundary). At end-of-
  // bag neither path fires if the final packet was mid-frame.
  //
  // Skip flush when the final packet already emitted a scan -- in that
  // case `current_scan_cloud_` is empty and replaying first-scan
  // packets would synthesise a spurious duplicate of the first scan
  // via the is_last_sub_frame path.
  if (!decoder_ || first_scan_packets_.empty() || last_feed_scan_complete_) {
    return std::nullopt;
  }
  for (const auto & pkt : first_scan_packets_) {
    decoder_->unpack(pkt);
    if (!ready_clouds_.empty()) {
      break;
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
