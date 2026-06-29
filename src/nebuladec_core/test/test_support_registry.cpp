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

#include <gtest/gtest.h>

#include <optional>

namespace nebuladec
{
namespace
{

using nebula::drivers::SensorModel;

const SupportRegistry & registry()
{
  return SupportRegistry::instance();
}

// --------------------------------------------------------------------------
// is_vendor_supported
// --------------------------------------------------------------------------

TEST(SupportRegistry, DecodedLidarVendorsAreSupported)
{
  EXPECT_TRUE(registry().is_vendor_supported(Vendor::HESAI));
  EXPECT_TRUE(registry().is_vendor_supported(Vendor::VELODYNE));
}

TEST(SupportRegistry, RobosenseIsIdentifiedButNotDecoded)
{
  // Robosense LiDAR is sniffed for vendor/model but nebuladec_adapters
  // does not provide a PointCloud2 adapter for it.
  EXPECT_FALSE(registry().is_vendor_supported(Vendor::ROBOSENSE));
}

TEST(SupportRegistry, ContinentalRadarIsNotSupported)
{
  EXPECT_FALSE(registry().is_vendor_supported(Vendor::CONTINENTAL));
}

TEST(SupportRegistry, UnknownVendorIsNotSupported)
{
  EXPECT_FALSE(registry().is_vendor_supported(Vendor::UNKNOWN));
}

// --------------------------------------------------------------------------
// is_model_supported
// --------------------------------------------------------------------------

TEST(SupportRegistry, HesaiKnownModelIsSupported)
{
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = SensorModel::HESAI_PANDARQT128;
  EXPECT_TRUE(registry().is_model_supported(id));
}

TEST(SupportRegistry, HesaiUnknownModelIsNotSupported)
{
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = SensorModel::UNKNOWN;
  EXPECT_FALSE(registry().is_model_supported(id));
}

// --------------------------------------------------------------------------
// check()
// --------------------------------------------------------------------------

TEST(SupportRegistryCheck, NulloptIdentityIsVendorUnknown)
{
  const auto d = registry().check(std::nullopt);
  EXPECT_EQ(d.level, SupportLevel::VendorUnknown);
  EXPECT_FALSE(d.reason.empty());
}

TEST(SupportRegistryCheck, UnknownVendorIsVendorUnknown)
{
  Identity id;
  id.vendor = Vendor::UNKNOWN;
  const auto d = registry().check(id);
  EXPECT_EQ(d.level, SupportLevel::VendorUnknown);
}

TEST(SupportRegistryCheck, ContinentalIsVendorNotSupported)
{
  Identity id;
  id.vendor = Vendor::CONTINENTAL;
  id.model = SensorModel::CONTINENTAL_ARS548;
  const auto d = registry().check(id);
  EXPECT_EQ(d.level, SupportLevel::VendorNotSupported);
}

TEST(SupportRegistryCheck, LidarVendorWithUnknownModelIsModelNotSupported)
{
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = SensorModel::UNKNOWN;
  const auto d = registry().check(id);
  EXPECT_EQ(d.level, SupportLevel::ModelNotSupported);
}

TEST(SupportRegistryCheck, FullySupportedPairReturnsSupported)
{
  Identity id;
  id.vendor = Vendor::VELODYNE;
  id.model = SensorModel::VELODYNE_VLP16;
  const auto d = registry().check(id);
  EXPECT_EQ(d.level, SupportLevel::Supported);
  EXPECT_TRUE(d.reason.empty());
}

// --------------------------------------------------------------------------
// supported_vendors
// --------------------------------------------------------------------------

TEST(SupportRegistry, SupportedVendorListCoversDecodedLidarVendors)
{
  const auto & list = registry().supported_vendors();
  EXPECT_EQ(list.size(), 2U);
  // CONTINENTAL, ROBOSENSE, and UNKNOWN must not appear (identified
  // vendors that lack a PointCloud2 adapter are excluded).
  for (auto v : list) {
    EXPECT_NE(v, Vendor::CONTINENTAL);
    EXPECT_NE(v, Vendor::ROBOSENSE);
    EXPECT_NE(v, Vendor::UNKNOWN);
  }
}

}  // namespace
}  // namespace nebuladec
