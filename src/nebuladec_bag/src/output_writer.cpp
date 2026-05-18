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

#include "output_writer.hpp"

#include "nebuladec_bag/point_cloud2.hpp"

#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <cstddef>
#include <memory>
#include <string>

namespace nebuladec::bag
{

// -- Rosbag2OutputWriter ----------------------------------------------------

Rosbag2OutputWriter::Rosbag2OutputWriter(rosbag2_cpp::Writer & writer) : writer_(writer)
{
}

void Rosbag2OutputWriter::create_topic(
  std::string_view topic, std::string_view type, std::string_view serialization_format)
{
  rosbag2_storage::TopicMetadata meta;
  meta.name = std::string{topic};
  meta.type = std::string{type};
  meta.serialization_format = std::string{serialization_format};
  writer_.create_topic(meta);
}

void Rosbag2OutputWriter::write_cloud(
  std::string_view out_topic, std::string_view frame_id,
  const nebula::drivers::NebulaPointCloud & cloud, std::int64_t stamp_ns)
{
  const auto pc_msg = to_point_cloud2(cloud, rclcpp::Time(stamp_ns), std::string{frame_id});
  // rosbag2_cpp::Writer::write performs CDR serialization internally
  // for typed messages; we mirror the pre-refactor call so behaviour
  // stays byte-equivalent.
  writer_.write(pc_msg, std::string{out_topic}, rclcpp::Time(stamp_ns));
}

void Rosbag2OutputWriter::write_passthrough(
  const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg,
  std::int64_t /*log_time_ns*/)
{
  // rosbag2_cpp::Writer::write takes a non-const shared_ptr; the writer
  // does not modify the payload bytes (only internal counters), so
  // const_cast is safe here. log_time comes from the message itself.
  writer_.write(std::const_pointer_cast<rosbag2_storage::SerializedBagMessage>(msg));
}

// -- McapDefinitionOutputWriter ---------------------------------------------

McapDefinitionOutputWriter::McapDefinitionOutputWriter(McapDefinitionWriter & writer)
: writer_(writer)
{
}

// `std::string_view` is passed by value across this adapter -- it
// matches the underlying `McapDefinitionWriter` string-view APIs and
// the type is a cheap {pointer,size} value. cppcheck's heuristic
// flagging pass-through methods is suppressed file-wide in
// `.cppcheck_suppressions` for this file.
void McapDefinitionOutputWriter::create_topic(
  std::string_view topic, std::string_view type, std::string_view serialization_format)
{
  writer_.create_topic(topic, type, serialization_format);
}

void McapDefinitionOutputWriter::write_cloud(
  std::string_view out_topic, std::string_view frame_id,
  const nebula::drivers::NebulaPointCloud & cloud, std::int64_t stamp_ns)
{
  const auto pc_msg = to_point_cloud2(cloud, rclcpp::Time(stamp_ns), std::string{frame_id});
  // McapDefinitionWriter takes raw bytes, so serialize manually.
  rclcpp::SerializedMessage serialized;
  pc2_serializer_.serialize_message(&pc_msg, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  writer_.write_serialized(
    out_topic, reinterpret_cast<const std::byte *>(raw.buffer),  // NOLINT
    raw.buffer_length, stamp_ns, stamp_ns);
}

void McapDefinitionOutputWriter::write_passthrough(
  const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg,
  std::int64_t log_time_ns)
{
  // SerializedBagMessage owns a shared rcl buffer (contiguous heap
  // allocation). Pass it through opaque -- no copy.
  const auto & raw = msg->serialized_data;
  writer_.write_serialized(
    msg->topic_name, reinterpret_cast<const std::byte *>(raw->buffer),  // NOLINT
    raw->buffer_length, log_time_ns, log_time_ns);
}

}  // namespace nebuladec::bag
