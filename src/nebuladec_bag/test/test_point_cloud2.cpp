// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_bag/point_cloud2.hpp"

#include <nebula_core_common/point_types.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

namespace nebuladec::bag
{

TEST(PointCloud2, EmptyCloudBecomesEmptyMsg)
{
  nebula::drivers::NebulaPointCloud cloud;
  const auto msg = to_point_cloud2(cloud, rclcpp::Time(0, 0), "lidar");
  EXPECT_EQ(msg.width, 0U);
  EXPECT_EQ(msg.height, 1U);
  EXPECT_TRUE(msg.data.empty());
  EXPECT_EQ(msg.header.frame_id, "lidar");
  EXPECT_EQ(msg.fields.size(), nebula::drivers::NebulaPoint::fields().size());
}

TEST(PointCloud2, PointsRoundTripThroughRawBuffer)
{
  nebula::drivers::NebulaPointCloud cloud;
  cloud.push_back(
    nebula::drivers::NebulaPoint{1.0F, 2.0F, 3.0F, 128, 1, 7, 0.5F, -0.25F, 12.0F, 42U});
  cloud.push_back(
    nebula::drivers::NebulaPoint{-4.0F, 5.0F, -6.0F, 64, 2, 9, 1.0F, 0.5F, 20.0F, 99U});

  const auto msg = to_point_cloud2(cloud, rclcpp::Time(1, 2), "lidar");
  EXPECT_EQ(msg.width, 2U);
  EXPECT_EQ(msg.point_step, sizeof(nebula::drivers::NebulaPoint));
  EXPECT_EQ(msg.data.size(), cloud.size() * sizeof(nebula::drivers::NebulaPoint));

  nebula::drivers::NebulaPoint decoded[2];
  std::memcpy(decoded, msg.data.data(), msg.data.size());
  EXPECT_FLOAT_EQ(decoded[0].x, 1.0F);
  EXPECT_FLOAT_EQ(decoded[1].z, -6.0F);
  EXPECT_EQ(decoded[0].channel, 7);
  EXPECT_EQ(decoded[1].time_stamp, 99U);
}

TEST(PointCloud2, FieldsMatchNebulaPointDescriptors)
{
  nebula::drivers::NebulaPointCloud cloud;
  const auto msg = to_point_cloud2(cloud, rclcpp::Time(0, 0), "lidar");
  const auto expected = nebula::drivers::NebulaPoint::fields();
  ASSERT_EQ(msg.fields.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(msg.fields[i].name, expected[i].name);
    EXPECT_EQ(msg.fields[i].offset, expected[i].offset);
    EXPECT_EQ(msg.fields[i].count, expected[i].count);
  }
}

}  // namespace nebuladec::bag
