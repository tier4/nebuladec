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
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

/// Build `TopicInspectResult` entries from a fully-populated
/// `TopicStateMap`. Shared between the rosbag2 and direct-SQLite inspect
/// paths so the summary format stays identical. Topics absent from
/// `topics_with_messages` carry zero ROS messages in the bag and are
/// dropped: inspect() has no data to sniff for them, so reporting an
/// entry would only add noise.
InspectSummary build_summary(
  const std::vector<PacketTopicSpec> & packet_order, const TopicStateMap & topic_states,
  const std::unordered_set<std::string> & topics_with_messages)
{
  InspectSummary summary;
  summary.topics.reserve(packet_order.size());
  // Preserve bag metadata order so repeated inspect() calls are stable.
  for (const auto & pt : packet_order) {
    if (!topics_with_messages.count(pt.topic)) {
      continue;
    }
    auto it = topic_states.find(pt.topic);
    if (it == topic_states.end()) {
      continue;
    }
    const auto & state = it->second;

    TopicInspectResult result;
    result.topic = state.topic;
    result.identity = state.sniffed_identity;
    summary.topics.push_back(std::move(result));
  }
  return summary;
}

/// Cleanup wrapper so early returns always close the DB / finalize the
/// statement. Keeps the inspect_sqlite3_file body linear.
struct SqliteStmtGuard
{
  sqlite3_stmt * stmt{nullptr};
  ~SqliteStmtGuard()
  {
    if (stmt) {
      sqlite3_finalize(stmt);
    }
  }
};

struct SqliteGuard
{
  sqlite3 * db{nullptr};
  ~SqliteGuard()
  {
    if (db) {
      sqlite3_close(db);
    }
  }
};

/// Inspect a bare `.db3` file without going through `rosbag2_cpp::Reader`.
///
/// `rosbag2_cpp::Reader::open()` reconstructs `BagMetadata` (COUNT / MIN
/// / MAX over `messages`) whenever `metadata.yaml` is absent, which on a
/// 17GB / 941k-row bag takes ~16s. inspect() does not consume any of that
/// metadata, so we bypass the reader and talk to libsqlite3 directly:
///
///   1) `SELECT id, name, type FROM topics`                          (~1ms)
///   2) `SELECT data, topic_id FROM messages WHERE id IN (
///         SELECT MIN(id) FROM messages WHERE topic_id IN (…)
///         GROUP BY topic_id)`                                        (~0.1s)
///
/// Step 2 returns exactly one row per non-empty target topic (empty
/// topics drop out of the GROUP BY), so the cost scales with the size of
/// the `messages` table but is a single aggregate scan instead of the
/// full JOIN + per-topic COUNT/MIN/MAX rosbag2 does on open.
InspectSummary inspect_sqlite3_file(const InputSpec & spec)
{
  SqliteGuard db_guard;
  if (sqlite3_open_v2(spec.uri.c_str(), &db_guard.db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    std::string msg = "failed to open sqlite3 bag: ";
    msg += spec.uri;
    if (db_guard.db) {
      msg += " (";
      msg += sqlite3_errmsg(db_guard.db);
      msg += ")";
    }
    throw std::runtime_error(msg);
  }

  struct TopicRow
  {
    int id;
    std::string name;
    std::string type;
  };
  std::vector<TopicRow> topics_rows;
  {
    SqliteStmtGuard sg;
    if (
      sqlite3_prepare_v2(db_guard.db, "SELECT id, name, type FROM topics", -1, &sg.stmt, nullptr) !=
      SQLITE_OK) {
      throw std::runtime_error(
        std::string{"failed to query topics: "} + sqlite3_errmsg(db_guard.db));
    }
    while (sqlite3_step(sg.stmt) == SQLITE_ROW) {
      TopicRow row;
      row.id = sqlite3_column_int(sg.stmt, 0);
      row.name = reinterpret_cast<const char *>(sqlite3_column_text(sg.stmt, 1));
      row.type = reinterpret_cast<const char *>(sqlite3_column_text(sg.stmt, 2));
      topics_rows.push_back(std::move(row));
    }
  }

  // Partition into packet / info topics, preserving bag order for
  // reporting stability.
  std::vector<PacketTopicSpec> packet_order;
  std::unordered_map<int, std::string> topic_id_to_name;
  TopicStateMap topic_states;
  std::unordered_map<std::string, int> packet_topic_id;
  InfoStateMap info_states;
  std::unordered_map<std::string, int> info_topic_id;
  std::vector<InfoTopicSpec> info_specs;
  for (const auto & r : topics_rows) {
    if (is_packet_type(r.type)) {
      packet_order.push_back({r.name, r.type});
      TopicState state;
      state.topic = r.name;
      state.type = r.type;
      state.vendor_hint = vendor_from_message_type(r.type);
      state.packet_source = make_packet_source(r.type);
      state.decoder.set_vendor_hint(state.vendor_hint);
      topic_states.emplace(r.name, std::move(state));
      packet_topic_id.emplace(r.name, r.id);
      topic_id_to_name.emplace(r.id, r.name);
    } else if (is_info_type(r.type)) {
      info_specs.push_back({r.name, r.type});
      InfoState info;
      info.topic = r.name;
      info.type = r.type;
      info.info_source = make_info_source(r.type);
      info_states.emplace(r.name, std::move(info));
      info_topic_id.emplace(r.name, r.id);
      topic_id_to_name.emplace(r.id, r.name);
    }
  }

  if (packet_order.empty()) {
    return {};
  }

  const auto global_info_topic = unique_info_topic(info_specs);
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

  // Build the bind list for the aggregate query: every packet topic id
  // plus the unique info topic id (if any). Empty topics cannot satisfy
  // GROUP BY so they naturally drop out -- that's exactly how we derive
  // `has_messages` without a second scan.
  std::vector<int> target_ids;
  target_ids.reserve(packet_topic_id.size() + (global_info_topic.empty() ? 0 : 1));
  for (const auto & [_, id] : packet_topic_id) {
    target_ids.push_back(id);
  }
  if (!global_info_topic.empty()) {
    if (auto it = info_topic_id.find(global_info_topic); it != info_topic_id.end()) {
      target_ids.push_back(it->second);
    }
  }

  // One row per non-empty target topic: the first inserted message's
  // bytes. MIN(id) is used (not MIN(timestamp)) so the aggregate can
  // resolve directly off the integer primary key without an extra
  // lookup. rosbag2 writes in timestamp order so the two coincide in
  // practice, and for vendor sniffing any early packet is sufficient.
  std::unordered_set<std::string> topics_with_messages;
  {
    std::string q =
      "SELECT data, topic_id FROM messages WHERE id IN ("
      "SELECT MIN(id) FROM messages WHERE topic_id IN (";
    for (std::size_t i = 0; i < target_ids.size(); ++i) {
      q += (i == 0) ? "?" : ",?";
    }
    q += ") GROUP BY topic_id)";

    SqliteStmtGuard sg;
    if (sqlite3_prepare_v2(db_guard.db, q.c_str(), -1, &sg.stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error(
        std::string{"failed to prepare first-message query: "} + sqlite3_errmsg(db_guard.db));
    }
    for (std::size_t i = 0; i < target_ids.size(); ++i) {
      sqlite3_bind_int(sg.stmt, static_cast<int>(i + 1), target_ids[i]);
    }

    while (sqlite3_step(sg.stmt) == SQLITE_ROW) {
      const void * blob = sqlite3_column_blob(sg.stmt, 0);
      const int blob_size = sqlite3_column_bytes(sg.stmt, 0);
      const int topic_id = sqlite3_column_int(sg.stmt, 1);
      auto name_it = topic_id_to_name.find(topic_id);
      if (name_it == topic_id_to_name.end()) {
        continue;
      }
      topics_with_messages.insert(name_it->second);

      // Wrap the raw CDR-serialized bytes in a rclcpp::SerializedMessage
      // without copying them a second time downstream -- packet_source /
      // info_source only read from the buffer.
      rclcpp::SerializedMessage serialized(static_cast<std::size_t>(blob_size));
      auto & raw = serialized.get_rcl_serialized_message();
      std::memcpy(raw.buffer, blob, static_cast<std::size_t>(blob_size));
      raw.buffer_length = static_cast<std::size_t>(blob_size);

      if (auto ts_it = topic_states.find(name_it->second); ts_it != topic_states.end()) {
        if (!ts_it->second.packet_source) {
          continue;
        }
        auto packets = ts_it->second.packet_source->extract(serialized);
        for (const auto & pkt : packets) {
          feed_packet(ts_it->second, pkt, nullptr);
        }
      } else if (auto is_it = info_states.find(name_it->second); is_it != info_states.end()) {
        if (!is_it->second.info_source || is_it->second.target_packet_topics.empty()) {
          continue;
        }
        auto info_bytes = is_it->second.info_source->extract(serialized);
        if (!info_bytes.empty()) {
          feed_info_to_targets(is_it->second, info_bytes, topic_states);
        }
      }
    }
  }

  // DIFOP may have resolved the model after the initial data-packet
  // sniff ran. Refresh each sniffed_identity from the decoder so the
  // summary reflects the latest known identity without feeding more
  // packets.
  for (auto & [_, state] : topic_states) {
    if (auto decoder_id = state.decoder.identity(); decoder_id) {
      state.sniffed_identity = decoder_id;
    }
  }

  return build_summary(packet_order, topic_states, topics_with_messages);
}

}  // namespace

InspectSummary inspect(const std::string & input_path)
{
  const auto spec = detect_input(input_path);

  // Why only SQLite3 bare files are routed to a low-level path:
  //
  // * SQLite3 (bare .db3 file) -- no metadata.yaml nearby, so
  //   rosbag2_cpp::Reader::open() reconstructs BagMetadata with an
  //   expensive `JOIN + GROUP BY topics.name` over the full messages
  //   table (COUNT/MIN/MAX per topic). Measured 16s on a 17GB / 941k-row
  //   bag. inspect() consumes none of that metadata, so we bypass the
  //   reader.
  // * SQLite3 (directory + metadata.yaml) -- rosbag2 reads the yaml and
  //   skips the scan; no benefit from going low-level.
  // * MCAP (bare file or directory) -- the MCAP container already stores
  //   per-channel message counts, start/end timestamps and a chunk
  //   index in its Summary section, so rosbag2_storage_mcap's
  //   get_metadata() is an O(1) read of that summary regardless of
  //   whether metadata.yaml is present. No "slow scan" exists to
  //   bypass.
  //
  // If MCAP open ever shows up as a bottleneck, revisit -- but measure
  // first; the format's self-indexing design makes it unlikely.
  if (!spec.is_directory && spec.storage_id == "sqlite3") {
    return inspect_sqlite3_file(spec);
  }

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

  // Fast path: read only the first message per packet topic (plus the
  // first unique-paired info topic message, if any) — just enough to
  // sniff vendor and model. Stop as soon as every relevant topic has
  // been touched once so large bags return quickly.
  const bool needs_info = !global_info_topic.empty();

  // 1) Restrict the storage reader to the topics we actually inspect.
  // Skipping unrelated topics (e.g. /tf, /imu, camera streams) at the
  // storage layer is far cheaper than reading+discarding their bytes.
  rosbag2_storage::StorageFilter filter;
  filter.topics.reserve(discovered.packet_topics.size() + (needs_info ? 1 : 0));
  for (const auto & pt : discovered.packet_topics) {
    filter.topics.push_back(pt.topic);
  }
  if (needs_info) {
    filter.topics.push_back(global_info_topic);
  }
  reader.set_filter(filter);

  // 2) Build a name -> bag-wide message count map from metadata. This
  // has two jobs: (a) pre-mark declared-but-silent topics as "done" so
  // the main loop does not walk the entire bag waiting for them;
  // (b) feed the per-topic `has_messages` flag in the summary with a
  // signal that comes straight from the metadata -- using the
  // loop-termination set here would make silent topics look like
  // "we read a message" and surface them as <unknown> instead of
  // <no packet messages>.
  std::unordered_map<std::string, std::size_t> message_counts;
  for (const auto & info : reader.get_metadata().topics_with_message_count) {
    message_counts.emplace(info.topic_metadata.name, info.message_count);
  }
  std::unordered_set<std::string> packet_done;
  std::unordered_set<std::string> info_done_set;
  for (const auto & [name, count] : message_counts) {
    if (count != 0) {
      continue;
    }
    if (topic_states.count(name)) {
      packet_done.insert(name);
    } else if (needs_info && name == global_info_topic) {
      info_done_set.insert(name);
    }
  }

  while (reader.has_next()) {
    const bool all_packets_done = packet_done.size() == topic_states.size();
    const bool info_done = !needs_info || info_done_set.count(global_info_topic) > 0;
    if (all_packets_done && info_done) {
      break;
    }

    auto bag_msg = reader.read_next();
    if (auto it = topic_states.find(bag_msg->topic_name); it != topic_states.end()) {
      if (packet_done.count(bag_msg->topic_name)) {
        continue;
      }
      packet_done.insert(bag_msg->topic_name);
      if (!it->second.packet_source) {
        continue;
      }
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto packets = it->second.packet_source->extract(serialized);
      for (const auto & pkt : packets) {
        feed_packet(it->second, pkt, nullptr);
      }
      continue;
    }
    if (needs_info) {
      if (auto it = info_states.find(bag_msg->topic_name); it != info_states.end()) {
        if (info_done_set.count(bag_msg->topic_name)) {
          continue;
        }
        info_done_set.insert(bag_msg->topic_name);
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
  }

  // DIFOP may have resolved the model after the initial data-packet
  // sniff ran. Refresh each sniffed_identity from the decoder so the
  // summary reflects the latest known identity without feeding more
  // packets.
  for (auto & [_, state] : topic_states) {
    if (auto decoder_id = state.decoder.identity(); decoder_id) {
      state.sniffed_identity = decoder_id;
    }
  }

  std::unordered_set<std::string> topics_with_messages;
  for (const auto & [name, count] : message_counts) {
    if (count > 0 && topic_states.count(name)) {
      topics_with_messages.insert(name);
    }
  }
  return build_summary(discovered.packet_topics, topic_states, topics_with_messages);
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

  std::int64_t last_packet_stamp_ns = 0;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->topic_name == packet_spec.topic && state.packet_source) {
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto packets = state.packet_source->extract(serialized);
      for (const auto & pkt : packets) {
        last_packet_stamp_ns = pkt.stamp_ns;
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

  // Mechanical-LiDAR decoders hold the final scan until the next packet
  // crosses the cut angle. At end-of-bag there is no next packet, so the
  // last scan would be lost. Ask the decoder to flush and write it with
  // the last-seen packet timestamp.
  if (auto trailing = state.decoder.flush(); trailing && *trailing) {
    ++state.clouds_produced;
    sink(*trailing, last_packet_stamp_ns);
  }

  ConvertResult result;
  result.identity = state.sniffed_identity ? state.sniffed_identity : state.decoder.identity();
  result.data_packets = state.data_packets;
  result.info_packets = state.info_packets;
  result.clouds_written = state.clouds_produced;
  return result;
}

}  // namespace nebuladec::bag
