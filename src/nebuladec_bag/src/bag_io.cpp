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

#include "nebuladec_bag/bag_io.hpp"

#include "nebuladec_bag/point_cloud2.hpp"
#include "packet_source.hpp"

#include <nebuladec_adapters/decoder.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebuladec::bag
{

namespace
{

namespace fs = std::filesystem;

std::string storage_id_from_extension(const fs::path & file)
{
  const auto ext = file.extension().string();
  if (ext == ".mcap") {
    return "mcap";
  }
  if (ext == ".db3") {
    return "sqlite3";
  }
  return "";
}

std::string storage_id_from_metadata(const fs::path & metadata_file)
{
  try {
    const auto root = YAML::LoadFile(metadata_file.string());
    const auto info = root["rosbag2_bagfile_information"];
    if (!info) {
      return "";
    }
    if (const auto sid = info["storage_identifier"]; sid) {
      return sid.as<std::string>();
    }
  } catch (const YAML::Exception &) {
    return "";
  }
  return "";
}

rosbag2_storage::StorageOptions to_storage_options(const InputSpec & spec)
{
  rosbag2_storage::StorageOptions opts;
  opts.uri = spec.uri;
  opts.storage_id = spec.storage_id;
  return opts;
}

}  // namespace

InputSpec detect_input(const std::string & path)
{
  const fs::path p(path);
  if (!fs::exists(p)) {
    throw std::invalid_argument("path does not exist: " + path);
  }

  if (fs::is_directory(p)) {
    const auto metadata = p / "metadata.yaml";
    if (!fs::exists(metadata)) {
      throw std::invalid_argument("directory has no metadata.yaml: " + path);
    }
    auto sid = storage_id_from_metadata(metadata);
    if (sid.empty()) {
      throw std::invalid_argument("metadata.yaml missing storage_identifier: " + path);
    }
    return InputSpec{p.lexically_normal().string(), std::move(sid), true};
  }

  if (fs::is_regular_file(p)) {
    const auto sid = storage_id_from_extension(p);
    if (sid.empty()) {
      throw std::invalid_argument(
        "unknown storage extension (expected .mcap or .db3): " + p.string());
    }
    return InputSpec{p.string(), sid, false};
  }

  throw std::invalid_argument("path is neither a file nor a directory: " + path);
}

namespace
{

struct TopicSelection
{
  std::string packets_topic;
  std::string packets_type;
  std::string info_topic;
  std::string info_type;
};

TopicSelection select_topics(
  const rosbag2_cpp::Reader & reader, const std::optional<std::string> & packets_override,
  const std::optional<std::string> & info_override)
{
  TopicSelection sel;
  for (const auto & info : reader.get_metadata().topics_with_message_count) {
    const auto & meta = info.topic_metadata;
    const auto & name = meta.name;
    const auto & type = meta.type;

    if (packets_override && name == *packets_override) {
      sel.packets_topic = name;
      sel.packets_type = type;
    } else if (!packets_override && sel.packets_topic.empty() && is_packet_type(type)) {
      sel.packets_topic = name;
      sel.packets_type = type;
    }

    if (info_override && name == *info_override) {
      sel.info_topic = name;
      sel.info_type = type;
    } else if (!info_override && sel.info_topic.empty() && is_info_type(type)) {
      sel.info_topic = name;
      sel.info_type = type;
    }
  }

  if (sel.packets_topic.empty()) {
    throw std::runtime_error("no Nebula packet topic found in bag");
  }
  return sel;
}

struct PipelineState
{
  Decoder decoder;
  std::unique_ptr<PacketSource> packet_source;
  std::unique_ptr<InfoSource> info_source;
  std::size_t data_packets{0};
  std::size_t info_packets{0};
  std::size_t clouds_produced{0};
};

void process_bag(
  rosbag2_cpp::Reader & reader, const TopicSelection & sel, PipelineState & state,
  const std::function<void(nebula::drivers::NebulaPointCloudPtr, std::int64_t)> & cloud_sink)
{
  state.packet_source = make_packet_source(sel.packets_type);
  if (!sel.info_type.empty()) {
    state.info_source = make_info_source(sel.info_type);
  }

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    const auto & topic = bag_msg->topic_name;

    if (topic == sel.packets_topic && state.packet_source) {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto packets = state.packet_source->extract(serialized);
      for (auto & pkt : packets) {
        ++state.data_packets;
        const double stamp_sec = static_cast<double>(pkt.stamp_ns) / 1e9;
        if (auto cloud = state.decoder.feed(pkt.data, stamp_sec); cloud && *cloud) {
          ++state.clouds_produced;
          cloud_sink(*cloud, pkt.stamp_ns);
        }
      }
    } else if (topic == sel.info_topic && state.info_source) {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto info_bytes = state.info_source->extract(serialized);
      if (!info_bytes.empty()) {
        ++state.info_packets;
        state.decoder.feed_info(info_bytes);
      }
    }
  }
}

}  // namespace

InspectSummary inspect(const std::string & input_path)
{
  const auto spec = detect_input(input_path);

  rosbag2_cpp::Reader reader;
  reader.open(to_storage_options(spec));

  const auto sel = select_topics(reader, std::nullopt, std::nullopt);
  PipelineState state;
  process_bag(reader, sel, state, [](auto, auto) {});

  InspectSummary summary;
  summary.identity = state.decoder.identity();
  summary.data_packets = state.data_packets;
  summary.info_packets = state.info_packets;
  summary.clouds_produced = state.clouds_produced;
  summary.packets_topic = sel.packets_topic;
  summary.info_topic = sel.info_topic;
  return summary;
}

ConvertResult convert(const ConvertOptions & options)
{
  const auto in_spec = detect_input(options.input_path);

  rosbag2_cpp::Reader reader;
  reader.open(to_storage_options(in_spec));

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions out_opts;
  out_opts.uri = options.output_path;
  out_opts.storage_id = in_spec.storage_id;  // mirror input plugin
  writer.open(out_opts);

  rosbag2_storage::TopicMetadata topic_meta;
  topic_meta.name = options.output_topic;
  topic_meta.type = "sensor_msgs/msg/PointCloud2";
  topic_meta.serialization_format = "cdr";
  writer.create_topic(topic_meta);

  const auto sel = select_topics(reader, options.packets_topic, options.info_topic);
  PipelineState state;

  auto sink = [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t stamp_ns) {
    if (!cloud || cloud->empty()) {
      return;
    }
    const auto pc_msg = to_point_cloud2(*cloud, rclcpp::Time(stamp_ns), options.frame_id);
    writer.write(pc_msg, options.output_topic, rclcpp::Time(stamp_ns));
  };

  process_bag(reader, sel, state, sink);

  ConvertResult result;
  result.identity = state.decoder.identity();
  result.data_packets = state.data_packets;
  result.info_packets = state.info_packets;
  result.clouds_written = state.clouds_produced;
  result.packets_topic = sel.packets_topic;
  result.info_topic = sel.info_topic;
  return result;
}

}  // namespace nebuladec::bag
