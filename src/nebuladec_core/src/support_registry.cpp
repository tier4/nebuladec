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

#include "nebuladec_core/support_registry.hpp"

#include "nebuladec_core/packet_sniffer.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>

namespace nebuladec
{

namespace
{

using nebula::drivers::SensorModel;

std::string format_model(const Identity & id)
{
  std::ostringstream os;
  if (id.model != SensorModel::UNKNOWN) {
    os << id.model;
  } else {
    os << "unknown";
  }
  return os.str();
}

}  // namespace

const SupportRegistry & SupportRegistry::instance()
{
  // Meyers singleton: constructed on first use, thread-safe under C++11,
  // zero-overhead after that.
  static const SupportRegistry registry;
  return registry;
}

SupportRegistry::SupportRegistry()
{
  // The lists below mirror what `make_adapter` (nebuladec_adapters) and
  // `PacketSniffer::identify` (nebuladec_core) currently agree on. New
  // models are added here and cross-checked by the adapters-level unit
  // tests before they are wired through.
  vendors_.push_back(
    {Vendor::HESAI,
     {
       SensorModel::HESAI_PANDAR40P,
       SensorModel::HESAI_PANDAR64,
       SensorModel::HESAI_PANDARQT64,
       SensorModel::HESAI_PANDARXT16,
       SensorModel::HESAI_PANDARXT32,
       SensorModel::HESAI_PANDARXT32M,
       SensorModel::HESAI_PANDARQT128,
       SensorModel::HESAI_PANDARAT128,
       SensorModel::HESAI_PANDAR128_E4X,
     }});
  vendors_.push_back(
    {Vendor::VELODYNE,
     {
       SensorModel::VELODYNE_HDL32,
       SensorModel::VELODYNE_VLP16,
       SensorModel::VELODYNE_VLP32,
       SensorModel::VELODYNE_VLS128,
     }});

  // Derive the supported-vendor quick list.
  supported_vendor_list_.reserve(vendors_.size());
  for (const auto & v : vendors_) {
    supported_vendor_list_.push_back(v.vendor);
  }
}

bool SupportRegistry::is_vendor_supported(Vendor vendor) const
{
  return std::find(supported_vendor_list_.begin(), supported_vendor_list_.end(), vendor) !=
         supported_vendor_list_.end();
}

bool SupportRegistry::is_model_supported(const Identity & identity) const
{
  for (const auto & vs : vendors_) {
    if (vs.vendor != identity.vendor) {
      continue;
    }
    return std::find(vs.sensor_models.begin(), vs.sensor_models.end(), identity.model) !=
           vs.sensor_models.end();
  }
  return false;
}

SupportDecision SupportRegistry::check(const std::optional<Identity> & identity) const
{
  if (!identity) {
    return {SupportLevel::VendorUnknown, "sniffer did not identify any vendor"};
  }
  if (identity->vendor == Vendor::UNKNOWN) {
    return {SupportLevel::VendorUnknown, "vendor is unknown"};
  }
  if (!is_vendor_supported(identity->vendor)) {
    std::string reason = "vendor '";
    reason += to_string(identity->vendor);
    reason += "' has no PointCloud2 adapter";
    return {SupportLevel::VendorNotSupported, std::move(reason)};
  }
  if (!is_model_supported(*identity)) {
    std::string reason = "model '" + format_model(*identity) + "' is not in the supported set for ";
    reason += to_string(identity->vendor);
    return {SupportLevel::ModelNotSupported, std::move(reason)};
  }
  return {SupportLevel::Supported, ""};
}

}  // namespace nebuladec
