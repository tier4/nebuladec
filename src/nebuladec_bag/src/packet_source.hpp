// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PACKET_SOURCE_HPP_
#define PACKET_SOURCE_HPP_

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
/// the rest of the bag layer.
class PacketSource
{
public:
  virtual ~PacketSource() = default;
  virtual std::vector<PacketBytes> extract(const rclcpp::SerializedMessage & msg) = 0;
};

class InfoSource
{
public:
  virtual ~InfoSource() = default;
  virtual std::vector<std::uint8_t> extract(const rclcpp::SerializedMessage & msg) = 0;
};

/// @brief Return non-null when `type_name` is a packet-stream type we
/// know how to decode.
std::unique_ptr<PacketSource> make_packet_source(const std::string & type_name);

/// @brief Return non-null for info-packet type names (currently only the
/// Robosense DIFOP message).
std::unique_ptr<InfoSource> make_info_source(const std::string & type_name);

/// @brief Is this ROS 2 message type one of the data-packet streams?
bool is_packet_type(const std::string & type_name);

/// @brief Is this ROS 2 message type the Robosense info-packet stream?
bool is_info_type(const std::string & type_name);

}  // namespace nebuladec::bag

#endif  // PACKET_SOURCE_HPP_
