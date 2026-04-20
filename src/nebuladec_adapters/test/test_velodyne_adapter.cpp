// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

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
