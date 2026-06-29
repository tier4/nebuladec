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

#include "packet_source.hpp"

#include <rclcpp/serialization.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <robosense_msgs/msg/robosense_scan.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebuladec::bag
{

namespace
{

constexpr auto k_nebula_packets = "nebula_msgs/msg/NebulaPackets";
constexpr auto k_pandar_scan = "pandar_msgs/msg/PandarScan";
constexpr auto k_velodyne_scan = "velodyne_msgs/msg/VelodyneScan";
constexpr auto k_robosense_scan = "robosense_msgs/msg/RobosenseScan";

std::int64_t nsec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * 1'000'000'000LL + static_cast<std::int64_t>(t.nanosec);
}

template <class Msg>
Msg deserialize(const rclcpp::SerializedMessage & serialized)
{
  Msg out;
  rclcpp::Serialization<Msg> serializer;
  serializer.deserialize_message(&serialized, &out);
  return out;
}

class NebulaPacketsSource : public PacketSource
{
public:
  std::vector<PacketBytes> extract(const rclcpp::SerializedMessage & msg) override
  {
    const auto parsed = deserialize<nebula_msgs::msg::NebulaPackets>(msg);
    std::vector<PacketBytes> out;
    out.reserve(parsed.packets.size());
    for (const auto & p : parsed.packets) {
      PacketBytes bytes;
      bytes.data.assign(p.data.begin(), p.data.end());
      bytes.stamp_ns = nsec(p.stamp);
      out.push_back(std::move(bytes));
    }
    return out;
  }
};

class PandarScanSource : public PacketSource
{
public:
  std::vector<PacketBytes> extract(const rclcpp::SerializedMessage & msg) override
  {
    const auto parsed = deserialize<pandar_msgs::msg::PandarScan>(msg);
    std::vector<PacketBytes> out;
    out.reserve(parsed.packets.size());
    for (const auto & p : parsed.packets) {
      PacketBytes bytes;
      const auto valid = std::min<std::uint32_t>(p.size, static_cast<std::uint32_t>(p.data.size()));
      bytes.data.assign(p.data.begin(), p.data.begin() + valid);
      bytes.stamp_ns = nsec(p.stamp);
      out.push_back(std::move(bytes));
    }
    return out;
  }
};

class VelodyneScanSource : public PacketSource
{
public:
  std::vector<PacketBytes> extract(const rclcpp::SerializedMessage & msg) override
  {
    const auto parsed = deserialize<velodyne_msgs::msg::VelodyneScan>(msg);
    std::vector<PacketBytes> out;
    out.reserve(parsed.packets.size());
    for (const auto & p : parsed.packets) {
      PacketBytes bytes;
      bytes.data.assign(p.data.begin(), p.data.end());
      bytes.stamp_ns = nsec(p.stamp);
      out.push_back(std::move(bytes));
    }
    return out;
  }
};

class RobosenseScanSource : public PacketSource
{
public:
  std::vector<PacketBytes> extract(const rclcpp::SerializedMessage & msg) override
  {
    const auto parsed = deserialize<robosense_msgs::msg::RobosenseScan>(msg);
    std::vector<PacketBytes> out;
    out.reserve(parsed.packets.size());
    for (const auto & p : parsed.packets) {
      PacketBytes bytes;
      bytes.data.assign(p.data.begin(), p.data.end());
      bytes.stamp_ns = nsec(p.stamp);
      out.push_back(std::move(bytes));
    }
    return out;
  }
};

}  // namespace

std::unique_ptr<PacketSource> make_packet_source(const std::string & type_name)
{
  if (type_name == k_nebula_packets) {
    return std::make_unique<NebulaPacketsSource>();
  }
  if (type_name == k_pandar_scan) {
    return std::make_unique<PandarScanSource>();
  }
  if (type_name == k_velodyne_scan) {
    return std::make_unique<VelodyneScanSource>();
  }
  if (type_name == k_robosense_scan) {
    return std::make_unique<RobosenseScanSource>();
  }
  return nullptr;
}

bool is_packet_type(const std::string & type_name)
{
  return type_name == k_nebula_packets || type_name == k_pandar_scan ||
         type_name == k_velodyne_scan || type_name == k_robosense_scan;
}

Vendor vendor_from_message_type(const std::string & type_name)
{
  if (type_name == k_pandar_scan) {
    return Vendor::HESAI;
  }
  if (type_name == k_velodyne_scan) {
    return Vendor::VELODYNE;
  }
  if (type_name == k_robosense_scan) {
    return Vendor::ROBOSENSE;
  }
  // nebula_msgs/NebulaPackets is a generic container (e.g. Continental
  // radar): only a packet-level sniff can decide the vendor.
  return Vendor::UNKNOWN;
}

}  // namespace nebuladec::bag
