// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef NEBULADEC_BAG__POINT_CLOUD2_HPP_
#define NEBULADEC_BAG__POINT_CLOUD2_HPP_

#include <nebula_core_common/point_types.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace nebuladec::bag
{

/// Convert a NebulaPointCloud into a sensor_msgs/PointCloud2. Uses the
/// static `fields()` metadata on NebulaPoint to populate the field
/// descriptors so the output retains every Nebula attribute (intensity,
/// return_type, channel, azimuth, elevation, distance, time_stamp).
sensor_msgs::msg::PointCloud2 to_point_cloud2(
  const nebula::drivers::NebulaPointCloud & cloud, const rclcpp::Time & stamp,
  const std::string & frame_id);

}  // namespace nebuladec::bag

#endif  // NEBULADEC_BAG__POINT_CLOUD2_HPP_
