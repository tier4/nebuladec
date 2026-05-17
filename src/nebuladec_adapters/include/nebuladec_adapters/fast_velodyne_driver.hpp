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

#ifndef NEBULADEC_ADAPTERS__FAST_VELODYNE_DRIVER_HPP_
#define NEBULADEC_ADAPTERS__FAST_VELODYNE_DRIVER_HPP_

// Drop-in alternative to `nebula::drivers::VelodyneDriver` for the
// sensor models nebuladec ships a fast path for. Same construction
// contract (sensor configuration, calibration) and same
// `parse_cloud_packet` API; internally instantiates the FastVlp* decoder
// matching the configured model.

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_velodyne_common/velodyne_common.hpp>
#include <nebula_velodyne_decoders/decoders/velodyne_scan_decoder.hpp>

#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

namespace nebuladec::adapters
{

class FastVelodyneDriver
{
public:
  static bool supports(nebula::drivers::SensorModel model) noexcept;

  FastVelodyneDriver(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> &
      sensor_configuration,
    const std::shared_ptr<const nebula::drivers::VelodyneCalibrationConfiguration> &
      calibration_configuration);

  [[nodiscard]] nebula::Status get_status() const noexcept { return driver_status_; }

  /// @brief Decode a packet. Mirrors upstream's `VelodyneDriver::
  /// parse_cloud_packet` return contract: returns the pointcloud and
  /// timestamp when the underlying decoder reports `has_scanned()`,
  /// otherwise returns an empty (default-constructed) pointcloud.
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> parse_cloud_packet(
    const std::vector<std::uint8_t> & packet, double packet_seconds);

private:
  nebula::Status driver_status_{nebula::Status::NOT_INITIALIZED};
  std::shared_ptr<nebula::drivers::VelodyneScanDecoder> scan_decoder_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__FAST_VELODYNE_DRIVER_HPP_
