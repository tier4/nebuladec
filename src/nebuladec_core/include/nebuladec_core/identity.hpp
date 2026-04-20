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

#ifndef NEBULADEC_CORE__IDENTITY_HPP_
#define NEBULADEC_CORE__IDENTITY_HPP_

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_seyond_common/seyond_common.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace nebuladec
{

enum class Vendor : std::uint8_t {
  UNKNOWN = 0,
  HESAI,
  VELODYNE,
  ROBOSENSE,
  SEYOND,
};

struct Identity
{
  Vendor vendor{Vendor::UNKNOWN};
  nebula::drivers::SensorModel model{nebula::drivers::SensorModel::UNKNOWN};
  nebula::drivers::ReturnMode return_mode{nebula::drivers::ReturnMode::UNKNOWN};
  /// Seyond has its own SensorModel enum (separate from the main one).
  /// Populated only when vendor == SEYOND.
  std::optional<nebula::drivers::SeyondSensorModel> seyond_model;
  float confidence{0.0F};
};

std::string to_string(Vendor vendor);

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__IDENTITY_HPP_
