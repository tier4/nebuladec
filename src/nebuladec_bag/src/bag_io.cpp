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
#include <unordered_map>
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

struct PacketTopicSpec
{
  std::string topic;
  std::string type;
};

struct InfoTopicSpec
{
  std::string topic;
  std::string type;
};

struct DiscoveredTopics
{
  std::vector<PacketTopicSpec> packet_topics;
  std::vector<InfoTopicSpec> info_topics;
};

DiscoveredTopics discover_topics(const rosbag2_cpp::Reader & reader)
{
  DiscoveredTopics out;
  for (const auto & info : reader.get_metadata().topics_with_message_count) {
    const auto & meta = info.topic_metadata;
    if (is_packet_type(meta.type)) {
      out.packet_topics.push_back({meta.name, meta.type});
    } else if (is_info_type(meta.type)) {
      out.info_topics.push_back({meta.name, meta.type});
    }
  }
  return out;
}

/// Auto-pair a Robosense packet topic with its DIFOP info topic, using
/// only structural signals (no topic-name heuristics):
///   * exactly one info topic in the bag -> pair with every Robosense
///     packet topic, on the assumption that a single bag carries one
///     Robosense calibration;
///   * zero or multiple info topics -> return empty; the caller must
///     pass an explicit override (e.g. --info-topic) to disambiguate.
/// Model identification NEVER depends on this pairing; the sniffer
/// resolves vendor and model from packet bytes alone.
std::string unique_info_topic(const std::vector<InfoTopicSpec> & infos)
{
  return infos.size() == 1 ? infos.front().topic : "";
}

struct TopicState
{
  std::string topic;
  std::string type;
  Vendor vendor_hint{Vendor::UNKNOWN};
  std::unique_ptr<PacketSource> packet_source;
  Decoder decoder;
  std::size_t data_packets{0};
  std::size_t info_packets{0};
  std::size_t clouds_produced{0};
  /// Tracks the last-seen identity so callers can inspect Continental
  /// radar topics even though make_adapter returns nullptr for them.
  /// Mirrors `decoder.identity()` for LiDAR vendors and carries the
  /// sniffed CONTINENTAL identity for radar.
  std::optional<Identity> sniffed_identity;
  PacketSniffer sniffer;
};

struct InfoState
{
  std::string topic;
  std::string type;
  std::unique_ptr<InfoSource> info_source;
  /// Packet topics that should receive feed_info() calls for this info
  /// topic. Set during inspect() / convert() setup and only populated
  /// when the bag has exactly one info topic.
  std::vector<std::string> target_packet_topics;
};

using TopicStateMap = std::unordered_map<std::string, TopicState>;
using InfoStateMap = std::unordered_map<std::string, InfoState>;

void feed_packet(
  TopicState & state, const PacketBytes & pkt,
  const std::function<void(nebula::drivers::NebulaPointCloudPtr, std::int64_t)> & cloud_sink)
{
  ++state.data_packets;

  // Keep sniffing for model information until we have a concrete
  // identity. Once resolved, the decoder short-circuits.
  if (
    !state.sniffed_identity ||
    state.sniffed_identity->model == nebula::drivers::SensorModel::UNKNOWN) {
    if (auto id = state.sniffer.identify(pkt.data, state.vendor_hint); id) {
      // Accept the newly-sniffed identity only if it strictly improves
      // on what we already have (unknown -> resolved model). Prevents
      // later garbage packets from wiping out a confirmed match.
      const bool first = !state.sniffed_identity.has_value();
      const bool resolved_model =
        id->model != nebula::drivers::SensorModel::UNKNOWN || id->seyond_model.has_value();
      if (
        first || (resolved_model &&
                  state.sniffed_identity->model == nebula::drivers::SensorModel::UNKNOWN)) {
        state.sniffed_identity = id;
      }
    }
  }

  const double stamp_sec = static_cast<double>(pkt.stamp_ns) / 1e9;
  auto cloud = state.decoder.feed(pkt.data, stamp_sec);
  if (cloud && *cloud) {
    ++state.clouds_produced;
    if (cloud_sink) {
      cloud_sink(*cloud, pkt.stamp_ns);
    }
  }
  // Prefer the decoder's resolved identity once it exists: it reflects
  // the adapter's own view (useful when, e.g., a Robosense DIFOP packet
  // pins down the model).
  if (auto decoder_id = state.decoder.identity(); decoder_id) {
    state.sniffed_identity = decoder_id;
  }
}

void feed_info_to_targets(
  const InfoState & info_state, const std::vector<std::uint8_t> & info_bytes,
  TopicStateMap & topics)
{
  for (const auto & target : info_state.target_packet_topics) {
    auto it = topics.find(target);
    if (it == topics.end()) {
      continue;
    }
    ++it->second.info_packets;
    it->second.decoder.feed_info(info_bytes);
  }
}

}  // namespace

InspectSummary inspect(const std::string & input_path)
{
  const auto spec = detect_input(input_path);

  rosbag2_cpp::Reader reader;
  reader.open(to_storage_options(spec));

  const auto discovered = discover_topics(reader);
  if (discovered.packet_topics.empty()) {
    return {};
  }

  TopicStateMap topic_states;
  topic_states.reserve(discovered.packet_topics.size());
  for (const auto & pt : discovered.packet_topics) {
    TopicState state;
    state.topic = pt.topic;
    state.type = pt.type;
    state.vendor_hint = vendor_from_message_type(pt.type);
    state.packet_source = make_packet_source(pt.type);
    state.decoder.set_vendor_hint(state.vendor_hint);
    topic_states.emplace(pt.topic, std::move(state));
  }

  InfoStateMap info_states;
  info_states.reserve(discovered.info_topics.size());
  for (const auto & it : discovered.info_topics) {
    InfoState info;
    info.topic = it.topic;
    info.type = it.type;
    info.info_source = make_info_source(it.type);
    info_states.emplace(it.topic, std::move(info));
  }
  // When the bag contains exactly one info topic, route it to every
  // Robosense packet topic. With zero or multiple info topics we cannot
  // decide ownership without a topic-name heuristic, so we stay silent
  // and leave pairing to an explicit --info-topic override. Model
  // identification does not depend on this step.
  const auto global_info_topic = unique_info_topic(discovered.info_topics);
  if (!global_info_topic.empty()) {
    auto info_it = info_states.find(global_info_topic);
    if (info_it != info_states.end()) {
      for (auto & [topic_name, state] : topic_states) {
        if (state.vendor_hint == Vendor::ROBOSENSE) {
          info_it->second.target_packet_topics.push_back(topic_name);
        }
      }
    }
  }

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (auto it = topic_states.find(bag_msg->topic_name); it != topic_states.end()) {
      if (!it->second.packet_source) {
        continue;
      }
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto packets = it->second.packet_source->extract(serialized);
      for (auto & pkt : packets) {
        feed_packet(it->second, pkt, nullptr);
      }
      continue;
    }
    if (auto it = info_states.find(bag_msg->topic_name); it != info_states.end()) {
      if (!it->second.info_source || it->second.target_packet_topics.empty()) {
        continue;
      }
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto info_bytes = it->second.info_source->extract(serialized);
      if (!info_bytes.empty()) {
        feed_info_to_targets(it->second, info_bytes, topic_states);
      }
    }
  }

  InspectSummary summary;
  summary.topics.reserve(discovered.packet_topics.size());
  // Preserve bag metadata order so repeated inspect() calls are stable.
  for (const auto & pt : discovered.packet_topics) {
    auto it = topic_states.find(pt.topic);
    if (it == topic_states.end()) {
      continue;
    }
    auto & state = it->second;

    TopicInspectResult result;
    result.topic = state.topic;
    result.message_type = state.type;
    result.vendor_by_message_type = state.vendor_hint;
    result.identity = state.sniffed_identity;
    result.data_packets = state.data_packets;
    result.info_packets = state.info_packets;
    result.clouds_produced = state.clouds_produced;
    if (state.vendor_hint == Vendor::ROBOSENSE) {
      result.info_topic = global_info_topic;
    }
    summary.topics.push_back(std::move(result));
  }
  return summary;
}

namespace
{

/// Resolve which packet topic `convert()` should operate on. When the
/// caller supplies an override, honor it verbatim. Otherwise require a
/// single LiDAR-capable topic — if more than one exists, fail with a
/// message pointing the user to --packets-topic. Radar topics are
/// excluded from auto-selection since `convert()` writes PointCloud2.
PacketTopicSpec choose_convert_packet_topic(
  const DiscoveredTopics & discovered, const std::optional<std::string> & override_name)
{
  if (override_name) {
    for (const auto & pt : discovered.packet_topics) {
      if (pt.topic == *override_name) {
        return pt;
      }
    }
    throw std::runtime_error("packets topic '" + *override_name + "' not found in bag");
  }

  std::vector<PacketTopicSpec> lidar_candidates;
  for (const auto & pt : discovered.packet_topics) {
    const auto vendor_hint = vendor_from_message_type(pt.type);
    // NebulaPackets (vendor_hint == UNKNOWN) is ambiguous — it could be
    // radar. Auto-select only when it is the sole candidate.
    if (vendor_hint != Vendor::UNKNOWN) {
      lidar_candidates.push_back(pt);
    }
  }
  if (lidar_candidates.empty()) {
    // Fall back to NebulaPackets topics; sniffer in the pipeline will
    // sort Seyond from radar.
    for (const auto & pt : discovered.packet_topics) {
      if (vendor_from_message_type(pt.type) == Vendor::UNKNOWN) {
        lidar_candidates.push_back(pt);
      }
    }
  }
  if (lidar_candidates.empty()) {
    throw std::runtime_error("no Nebula packet topic found in bag");
  }
  if (lidar_candidates.size() > 1) {
    std::string msg = "multiple packet topics present; pass --packets-topic to pick one:";
    for (const auto & c : lidar_candidates) {
      msg += "\n  " + c.topic + " (" + c.type + ")";
    }
    throw std::runtime_error(msg);
  }
  return lidar_candidates.front();
}

}  // namespace

ConvertResult convert(const ConvertOptions & options)
{
  const auto in_spec = detect_input(options.input_path);

  rosbag2_cpp::Reader reader;
  reader.open(to_storage_options(in_spec));

  const auto discovered = discover_topics(reader);
  const auto packet_spec = choose_convert_packet_topic(discovered, options.packets_topic);
  const auto vendor_hint = vendor_from_message_type(packet_spec.type);

  // Info topic: honor an explicit override; otherwise pair by name prefix
  // for Robosense streams.
  std::string info_topic_name;
  if (options.info_topic) {
    for (const auto & it : discovered.info_topics) {
      if (it.topic == *options.info_topic) {
        info_topic_name = it.topic;
        break;
      }
    }
    if (info_topic_name.empty()) {
      throw std::runtime_error("info topic '" + *options.info_topic + "' not found in bag");
    }
  } else if (vendor_hint == Vendor::ROBOSENSE) {
    // Auto-pair only when the bag has exactly one info topic. Multiple
    // info topics require --info-topic so pairing never relies on
    // topic-name matching.
    info_topic_name = unique_info_topic(discovered.info_topics);
  }

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

  TopicState state;
  state.topic = packet_spec.topic;
  state.type = packet_spec.type;
  state.vendor_hint = vendor_hint;
  state.packet_source = make_packet_source(packet_spec.type);
  state.decoder.set_vendor_hint(vendor_hint);

  std::unique_ptr<InfoSource> info_source;
  if (!info_topic_name.empty()) {
    for (const auto & it : discovered.info_topics) {
      if (it.topic == info_topic_name) {
        info_source = make_info_source(it.type);
        break;
      }
    }
  }

  auto sink = [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t stamp_ns) {
    if (!cloud || cloud->empty()) {
      return;
    }
    const auto pc_msg = to_point_cloud2(*cloud, rclcpp::Time(stamp_ns), options.frame_id);
    writer.write(pc_msg, options.output_topic, rclcpp::Time(stamp_ns));
  };

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->topic_name == packet_spec.topic && state.packet_source) {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto packets = state.packet_source->extract(serialized);
      for (auto & pkt : packets) {
        feed_packet(state, pkt, sink);
      }
    } else if (bag_msg->topic_name == info_topic_name && info_source) {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto info_bytes = info_source->extract(serialized);
      if (!info_bytes.empty()) {
        ++state.info_packets;
        state.decoder.feed_info(info_bytes);
      }
    }
  }

  ConvertResult result;
  result.identity = state.sniffed_identity ? state.sniffed_identity : state.decoder.identity();
  result.data_packets = state.data_packets;
  result.info_packets = state.info_packets;
  result.clouds_written = state.clouds_produced;
  result.packets_topic = packet_spec.topic;
  result.info_topic = info_topic_name;
  return result;
}

}  // namespace nebuladec::bag
