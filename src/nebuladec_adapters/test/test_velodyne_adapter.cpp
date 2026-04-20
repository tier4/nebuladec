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

#include "nebuladec_adapters/decoder.hpp"
#include "nebuladec_adapters/velodyne_adapter.hpp"

#include <nebuladec_core/identity.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{
namespace
{

Identity make_velodyne_identity(nebula::drivers::SensorModel model)
{
  Identity id;
  id.vendor = Vendor::VELODYNE;
  id.model = model;
  id.return_mode = nebula::drivers::ReturnMode::SINGLE_STRONGEST;
  id.confidence = 0.95F;
  return id;
}

}  // namespace

TEST(VelodyneAdapter, ReadyForVLP16)
{
  adapters::VelodyneAdapter adapter(
    make_velodyne_identity(nebula::drivers::SensorModel::VELODYNE_VLP16));
  EXPECT_TRUE(adapter.is_ready());
  EXPECT_EQ(adapter.identity().model, nebula::drivers::SensorModel::VELODYNE_VLP16);
}

TEST(VelodyneAdapter, ReadyForVLS128)
{
  adapters::VelodyneAdapter adapter(
    make_velodyne_identity(nebula::drivers::SensorModel::VELODYNE_VLS128));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(VelodyneAdapter, NotReadyForUnknownModel)
{
  Identity id;
  id.vendor = Vendor::VELODYNE;
  id.model = nebula::drivers::SensorModel::UNKNOWN;
  adapters::VelodyneAdapter adapter(id);
  EXPECT_FALSE(adapter.is_ready());
}

TEST(VelodyneAdapter, EmptyPacketDoesNotProduceCloud)
{
  adapters::VelodyneAdapter adapter(
    make_velodyne_identity(nebula::drivers::SensorModel::VELODYNE_VLP16));
  ASSERT_TRUE(adapter.is_ready());
  std::vector<std::uint8_t> empty;
  EXPECT_FALSE(adapter.feed(empty, 0.0).has_value());
}

TEST(MakeAdapter, VelodyneReturnsAdapterForKnownModel)
{
  auto adapter = make_adapter(make_velodyne_identity(nebula::drivers::SensorModel::VELODYNE_VLP32));
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(adapter->identity().vendor, Vendor::VELODYNE);
}

TEST(MakeAdapter, VelodyneReturnsNullForUnknownModel)
{
  Identity id;
  id.vendor = Vendor::VELODYNE;
  id.model = nebula::drivers::SensorModel::UNKNOWN;
  EXPECT_EQ(make_adapter(id), nullptr);
}

}  // namespace nebuladec
