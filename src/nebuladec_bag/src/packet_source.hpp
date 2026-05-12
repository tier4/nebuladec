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

#ifndef PACKET_SOURCE_HPP_
#define PACKET_SOURCE_HPP_

#include <nebuladec_core/identity.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nebuladec::bag
{

/// @brief Byte payload + ns timestamp extracted from a single packet.
struct PacketBytes
{
  std::vector<std::uint8_t> data;
  std::int64_t stamp_ns{0};
};

/// @brief Turns a serialized bag message of a specific vendor type into
/// zero or more raw-packet byte vectors.
///
/// Each Nebula-facing ROS message type (NebulaPackets, PandarScan,
/// VelodyneScan, RobosenseScan) wraps a vector of per-UDP-packet byte
/// payloads; this interface hides the deserialisation differences from
/// the rest of the bag layer. RobosenseScan is read for vendor / sensor
/// model identification only — nebuladec_adapters does not produce a
/// PointCloud2 stream for Robosense data.
class PacketSource
{
public:
  virtual ~PacketSource() = default;
  virtual std::vector<PacketBytes> extract(const rclcpp::SerializedMessage & msg) = 0;
};

/// @brief Return non-null when `type_name` is a packet-stream type we
/// know how to read.
std::unique_ptr<PacketSource> make_packet_source(const std::string & type_name);

/// @brief Is this ROS 2 message type one of the data-packet streams?
bool is_packet_type(const std::string & type_name);

/// @brief Infer the sensor vendor from a packet-stream message type.
///
/// The vendor of `pandar_msgs/PandarScan`, `velodyne_msgs/VelodyneScan`
/// and `robosense_msgs/RobosenseScan` is known from the message type
/// alone. `nebula_msgs/NebulaPackets` is a generic container used by
/// Seyond LiDAR AND Continental radar, so it returns `Vendor::UNKNOWN`
/// and the caller must disambiguate by sniffing the packet payload.
/// Any other / non-packet type returns `Vendor::UNKNOWN`.
Vendor vendor_from_message_type(const std::string & type_name);

}  // namespace nebuladec::bag

#endif  // PACKET_SOURCE_HPP_
