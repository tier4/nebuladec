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

#include "nebuladec_adapters/accelerated_hesai_driver.hpp"

#include "nebuladec_adapters/accelerated_hesai_decoder.hpp"

#include <nebula_hesai_decoders/decoders/pandar_128e4x.hpp>
#include <nebula_hesai_decoders/decoders/pandar_40.hpp>
#include <nebula_hesai_decoders/decoders/pandar_64.hpp>
#include <nebula_hesai_decoders/decoders/pandar_at128.hpp>
#include <nebula_hesai_decoders/decoders/pandar_qt128.hpp>
#include <nebula_hesai_decoders/decoders/pandar_qt64.hpp>
#include <nebula_hesai_decoders/decoders/pandar_xt16.hpp>
#include <nebula_hesai_decoders/decoders/pandar_xt32.hpp>
#include <nebula_hesai_decoders/decoders/pandar_xt32m.hpp>

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nebuladec::adapters
{

namespace
{

template <typename SensorT>
std::shared_ptr<nebula::drivers::HesaiScanDecoder> make_accelerated_decoder(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & calibration,
  const std::shared_ptr<nebula::drivers::loggers::Logger> & logger)
{
  using CalibT = typename SensorT::angle_corrector_t::correction_data_t;
  auto typed = std::dynamic_pointer_cast<const CalibT>(calibration);
  if (!typed) {
    throw std::runtime_error("AcceleratedHesaiDriver: calibration type mismatch");
  }
  return std::make_shared<AcceleratedHesaiDecoder<SensorT>>(
    sensor_configuration, std::move(typed), logger);
}

}  // namespace

bool AcceleratedHesaiDriver::supports(nebula::drivers::SensorModel model) noexcept
{
  // The accelerated decoder is a template over upstream's `HesaiSensor<PacketT>`
  // family; every Hesai model exposed by `hesai_adapter.cpp` derives from that
  // base, so the same code path covers all of them. Output equivalence with
  // upstream is verified against the per-model fixtures in upstream's
  // `nebula_hesai/test_resources/decoder_ground_truth/` rather than via a
  // device-specific sample bag.
  switch (model) {
    case nebula::drivers::SensorModel::HESAI_PANDAR40P:
    case nebula::drivers::SensorModel::HESAI_PANDAR64:
    case nebula::drivers::SensorModel::HESAI_PANDARQT64:
    case nebula::drivers::SensorModel::HESAI_PANDARQT128:
    case nebula::drivers::SensorModel::HESAI_PANDARXT16:
    case nebula::drivers::SensorModel::HESAI_PANDARXT32:
    case nebula::drivers::SensorModel::HESAI_PANDARXT32M:
    case nebula::drivers::SensorModel::HESAI_PANDARAT128:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E4X:
      return true;
    default:
      return false;
  }
}

AcceleratedHesaiDriver::AcceleratedHesaiDriver(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & calibration,
  const std::shared_ptr<nebula::drivers::loggers::Logger> & logger,
  nebula::drivers::HesaiScanDecoder::pointcloud_callback_t pointcloud_cb)
: logger_(logger)
{
  switch (sensor_configuration->sensor_model) {
    case nebula::drivers::SensorModel::HESAI_PANDAR40P:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::Pandar40>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDAR64:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::Pandar64>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDARQT64:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::PandarQT64>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDARQT128:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::PandarQT128>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDARXT16:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::PandarXT16>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDARXT32:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::PandarXT32>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDARXT32M:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::PandarXT32M>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDARAT128:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::PandarAT128>(
        sensor_configuration, calibration, logger_);
      break;
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E4X:
      scan_decoder_ = make_accelerated_decoder<nebula::drivers::Pandar128E4X>(
        sensor_configuration, calibration, logger_);
      break;
    default:
      throw std::runtime_error("AcceleratedHesaiDriver: unsupported sensor model");
  }
  scan_decoder_->set_pointcloud_callback(std::move(pointcloud_cb));
  driver_status_ = nebula::Status::OK;
}

void AcceleratedHesaiDriver::parse_cloud_packet(const std::vector<std::uint8_t> & packet)
{
  if (driver_status_ != nebula::Status::OK || !scan_decoder_) {
    return;
  }
  scan_decoder_->unpack(packet);
}

}  // namespace nebuladec::adapters
