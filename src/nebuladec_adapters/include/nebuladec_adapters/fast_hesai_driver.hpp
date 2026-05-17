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

#ifndef NEBULADEC_ADAPTERS__FAST_HESAI_DRIVER_HPP_
#define NEBULADEC_ADAPTERS__FAST_HESAI_DRIVER_HPP_

// Drop-in alternative to `nebula::drivers::HesaiDriver` for the sensor
// models nebuladec supports a fast path for. Same construction-time
// contract (sensor configuration, calibration, logger, pointcloud
// callback) and same `parse_cloud_packet` API; internally instantiates
// `FastHesaiDecoder<SensorT>` instead of upstream's `HesaiDecoder<SensorT>`.
//
// Why a parallel driver: `nebula::drivers::HesaiDriver` keeps its
// `scan_decoder_` member private and instantiates it in the constructor
// via an internal template factory, so injecting a different decoder
// from outside requires forking. The alternative is to ship our own
// driver that mirrors the same construction wiring. That driver is this
// header.

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebuladec::adapters
{

class FastHesaiDriver
{
public:
  /// @return true iff `FastHesaiDriver` covers this model. Caller should
  /// fall back to `nebula::drivers::HesaiDriver` when this returns false.
  static bool supports(nebula::drivers::SensorModel model) noexcept;

  FastHesaiDriver(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & calibration,
    const std::shared_ptr<nebula::drivers::loggers::Logger> & logger,
    nebula::drivers::HesaiScanDecoder::pointcloud_callback_t pointcloud_cb);

  [[nodiscard]] nebula::Status get_status() const noexcept { return driver_status_; }

  /// @brief Decode a packet through the fast path.
  void parse_cloud_packet(const std::vector<std::uint8_t> & packet);

private:
  nebula::Status driver_status_{nebula::Status::NOT_INITIALIZED};
  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;
  std::shared_ptr<nebula::drivers::HesaiScanDecoder> scan_decoder_;
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__FAST_HESAI_DRIVER_HPP_
