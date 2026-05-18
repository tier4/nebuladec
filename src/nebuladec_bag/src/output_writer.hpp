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

#ifndef OUTPUT_WRITER_HPP_
#define OUTPUT_WRITER_HPP_

#include "mcap_definition_writer.hpp"

#include <nebula_core_common/point_types.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace nebuladec::bag
{

/// @brief Abstract output sink used by the convert pipeline.
///
/// Wraps the two underlying writers that nebuladec_bag uses today
/// (`rosbag2_cpp::Writer` for the generic path, `McapDefinitionWriter`
/// for the schema-forwarding fast path) behind a single interface so
/// the read loop does not have to know which one is active.
///
/// All implementations are designed to be driven from a single thread
/// for the duration of a `convert()` call.
class OutputWriter
{
public:
  virtual ~OutputWriter() = default;

  OutputWriter() = default;
  OutputWriter(const OutputWriter &) = delete;
  OutputWriter & operator=(const OutputWriter &) = delete;
  OutputWriter(OutputWriter &&) = delete;
  OutputWriter & operator=(OutputWriter &&) = delete;

  virtual void create_topic(
    std::string_view topic, std::string_view type, std::string_view serialization_format) = 0;

  virtual void write_cloud(
    std::string_view out_topic, std::string_view frame_id,
    const nebula::drivers::NebulaPointCloud & cloud, std::int64_t stamp_ns) = 0;

  /// `log_time_ns` is passed explicitly so the caller can apply the
  /// SFINAE-resolved `bag_message_log_time_ns` helper once and feed
  /// the result here — implementations differ in whether they use the
  /// value (the MCAP-definition path requires it; the rosbag2 path
  /// reads log_time from the message internally and ignores the
  /// parameter).
  virtual void write_passthrough(
    const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg,
    std::int64_t log_time_ns) = 0;
};

/// `rosbag2_cpp::Writer`-backed implementation. Used for every output
/// except the MCAP schema-forwarding fast path.
class Rosbag2OutputWriter final : public OutputWriter
{
public:
  explicit Rosbag2OutputWriter(rosbag2_cpp::Writer & writer);

  void create_topic(
    std::string_view topic, std::string_view type, std::string_view serialization_format) override;
  void write_cloud(
    std::string_view out_topic, std::string_view frame_id,
    const nebula::drivers::NebulaPointCloud & cloud, std::int64_t stamp_ns) override;
  void write_passthrough(
    const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg,
    std::int64_t log_time_ns) override;

private:
  rosbag2_cpp::Writer & writer_;
};

/// `McapDefinitionWriter`-backed implementation. Used on the MCAP
/// schema-forwarding fast path.
class McapDefinitionOutputWriter final : public OutputWriter
{
public:
  explicit McapDefinitionOutputWriter(McapDefinitionWriter & writer);

  void create_topic(
    std::string_view topic, std::string_view type, std::string_view serialization_format) override;
  void write_cloud(
    std::string_view out_topic, std::string_view frame_id,
    const nebula::drivers::NebulaPointCloud & cloud, std::int64_t stamp_ns) override;
  void write_passthrough(
    const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> & msg,
    std::int64_t log_time_ns) override;

private:
  McapDefinitionWriter & writer_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer_;
};

}  // namespace nebuladec::bag

#endif  // OUTPUT_WRITER_HPP_
