// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_bag/point_cloud2.hpp"

#include <nebula_core_common/point_cloud.hpp>
#include <nebula_core_common/point_types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <cstring>
#include <string>

namespace nebuladec::bag
{

namespace
{

std::uint8_t to_ros_datatype(nebula::drivers::PointField::DataType t)
{
  using DT = nebula::drivers::PointField::DataType;
  using sensor_msgs::msg::PointField;
  switch (t) {
    case DT::Int8:
      return PointField::INT8;
    case DT::UInt8:
      return PointField::UINT8;
    case DT::Int16:
      return PointField::INT16;
    case DT::UInt16:
      return PointField::UINT16;
    case DT::Int32:
      return PointField::INT32;
    case DT::UInt32:
      return PointField::UINT32;
    case DT::Float32:
      return PointField::FLOAT32;
    case DT::Float64:
      return PointField::FLOAT64;
  }
  return PointField::FLOAT32;
}

}  // namespace

sensor_msgs::msg::PointCloud2 to_point_cloud2(
  const nebula::drivers::NebulaPointCloud & cloud, const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.is_bigendian = false;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(cloud.size());

  const auto fields = nebula::drivers::NebulaPoint::fields();
  msg.fields.reserve(fields.size());
  for (const auto & f : fields) {
    sensor_msgs::msg::PointField pf;
    pf.name = f.name;
    pf.offset = f.offset;
    pf.datatype = to_ros_datatype(f.datatype);
    pf.count = f.count;
    msg.fields.push_back(pf);
  }

  msg.point_step = sizeof(nebula::drivers::NebulaPoint);
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);
  if (!cloud.empty()) {
    std::memcpy(msg.data.data(), cloud.data(), msg.row_step);
  }
  msg.is_dense = true;
  return msg;
}

}  // namespace nebuladec::bag
