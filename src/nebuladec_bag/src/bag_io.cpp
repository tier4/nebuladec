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
#include <nebuladec_core/support_registry.hpp>
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

/// Inverse of `storage_id_from_extension` for the write path.
std::string storage_ext_for_id(const std::string & storage_id)
{
  if (storage_id == "mcap") {
    return ".mcap";
  }
  if (storage_id == "sqlite3") {
    return ".db3";
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

struct DiscoveredTopics
{
  std::vector<PacketTopicSpec> packet_topics;
};

DiscoveredTopics discover_topics(const rosbag2_cpp::Reader & reader)
{
  DiscoveredTopics out;
  for (const auto & info : reader.get_metadata().topics_with_message_count) {
    const auto & meta = info.topic_metadata;
    if (is_packet_type(meta.type)) {
      out.packet_topics.push_back({meta.name, meta.type});
    }
  }
  return out;
}

struct TopicState
{
  std::string topic;
  std::string type;
  Vendor vendor_hint{Vendor::UNKNOWN};
  std::unique_ptr<PacketSource> packet_source;
  Decoder decoder;
  std::size_t data_packets{0};
  std::size_t clouds_produced{0};
  /// Tracks the last-seen identity so callers can inspect Continental
  /// radar / Robosense topics even though make_adapter returns nullptr
  /// for them. Mirrors `decoder.identity()` for decoded LiDAR vendors
  /// and carries the sniffed identity for identification-only vendors.
  std::optional<Identity> sniffed_identity;
  PacketSniffer sniffer;
};

using TopicStateMap = std::unordered_map<std::string, TopicState>;

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
  // Prefer the decoder's resolved identity once it exists.
  if (auto decoder_id = state.decoder.identity(); decoder_id) {
    state.sniffed_identity = decoder_id;
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
  // Zero-message topics are kept (with identity unset + has_messages
  // false) so `plan_convert()` can emit "skipped: no messages" for them
  // without having to re-walk the bag metadata.
  for (const auto & pt : packet_order) {
    TopicInspectResult result;
    result.topic = pt.topic;
    result.has_messages = topics_with_messages.count(pt.topic) > 0;
    if (auto it = topic_states.find(pt.topic); it != topic_states.end()) {
      result.identity = it->second.sniffed_identity;
    }
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

  // Filter to packet topics, preserving bag order for reporting stability.
  std::vector<PacketTopicSpec> packet_order;
  std::unordered_map<int, std::string> topic_id_to_name;
  TopicStateMap topic_states;
  std::unordered_map<std::string, int> packet_topic_id;
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
    }
  }

  if (packet_order.empty()) {
    return {};
  }

  // Build the bind list for the aggregate query: every packet topic id.
  // Empty topics cannot satisfy GROUP BY so they naturally drop out --
  // that's exactly how we derive `has_messages` without a second scan.
  std::vector<int> target_ids;
  target_ids.reserve(packet_topic_id.size());
  for (const auto & entry : packet_topic_id) {
    target_ids.push_back(entry.second);
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
      // without copying them a second time downstream -- packet_source
      // only reads from the buffer.
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
      }
    }
  }

  // Refresh each sniffed_identity from the decoder so the summary
  // reflects the latest known identity without feeding more packets.
  for (auto & entry : topic_states) {
    auto & state = entry.second;
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

  // Fast path: read only the first message per packet topic — just
  // enough to sniff vendor and model. Stop as soon as every relevant
  // topic has been touched once so large bags return quickly.

  // 1) Restrict the storage reader to the topics we actually inspect.
  // Skipping unrelated topics (e.g. /tf, /imu, camera streams) at the
  // storage layer is far cheaper than reading+discarding their bytes.
  rosbag2_storage::StorageFilter filter;
  filter.topics.reserve(discovered.packet_topics.size());
  for (const auto & pt : discovered.packet_topics) {
    filter.topics.push_back(pt.topic);
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
  for (const auto & [name, count] : message_counts) {
    if (count != 0) {
      continue;
    }
    if (topic_states.count(name)) {
      packet_done.insert(name);
    }
  }

  while (reader.has_next()) {
    if (packet_done.size() == topic_states.size()) {
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
    }
  }

  // Refresh each sniffed_identity from the decoder so the summary
  // reflects the latest known identity without feeding more packets.
  for (auto & entry : topic_states) {
    auto & state = entry.second;
    if (auto decoder_id = state.decoder.identity(); decoder_id) {
      state.sniffed_identity = decoder_id;
    }
  }

  std::unordered_set<std::string> topics_with_messages;
  for (const auto & [name, count] : message_counts) {
    if (count > 0) {
      topics_with_messages.insert(name);
    }
  }
  return build_summary(discovered.packet_topics, topic_states, topics_with_messages);
}

namespace
{

struct ResolvedRule
{
  std::string in_topic;
  std::string type;
  MappingMatch match;
};

/// Resolve every discovered packet topic against `mapping`. Packet
/// topics that match no rule are returned in `skipped` so callers can
/// report them. Throws std::runtime_error on duplicate matches (forwarded
/// from TopicMapping::resolve, annotated with the offending topic).
struct ResolvePartition
{
  std::vector<ResolvedRule> resolved;
  std::vector<std::string> skipped;
};

ResolvePartition resolve_discovered_topics(
  const std::vector<PacketTopicSpec> & packet_topics, const TopicMapping & mapping)
{
  ResolvePartition out;
  out.resolved.reserve(packet_topics.size());
  for (const auto & pt : packet_topics) {
    auto match = mapping.resolve(pt.topic);  // may throw on duplicate match
    if (!match) {
      out.skipped.push_back(pt.topic);
      continue;
    }
    ResolvedRule r;
    r.in_topic = pt.topic;
    r.type = pt.type;
    r.match = std::move(*match);
    out.resolved.push_back(std::move(r));
  }
  return out;
}

}  // namespace

std::vector<ConvertPlanEntry> plan_convert(
  const std::string & input_path, const TopicMapping & mapping)
{
  // Reuse inspect() so dry-run sees the same sniffed identities as the
  // full convert() flow. inspect() reports every packet topic (including
  // zero-message ones) present in the bag.
  const auto summary = inspect(input_path);

  std::vector<ConvertPlanEntry> entries;
  entries.reserve(summary.topics.size());
  for (const auto & t : summary.topics) {
    ConvertPlanEntry entry;
    entry.in_topic = t.topic;
    entry.identity = t.identity;

    // Order matters: data-level reasons ("no messages", "unsupported
    // vendor/model") short-circuit before mapping resolution because a
    // topic with no data or no decoder support cannot produce clouds
    // even if it matches a rule. All vendor/model support questions go
    // through `SupportRegistry` so this file never has to know which
    // specific vendors or models are decodable.
    if (!t.has_messages) {
      entry.status = "skipped";
      entry.message = "no messages";
      entries.push_back(std::move(entry));
      continue;
    }
    const auto & registry = SupportRegistry::instance();
    const auto support = registry.check(t.identity);
    if (support.level == SupportLevel::VendorUnknown) {
      entry.status = "skipped";
      entry.message = "unknown vendor";
      entries.push_back(std::move(entry));
      continue;
    }
    if (support.level == SupportLevel::VendorNotSupported) {
      entry.status = "skipped";
      entry.message = "vendor not supported";
      entries.push_back(std::move(entry));
      continue;
    }
    if (support.level == SupportLevel::ModelNotSupported) {
      entry.status = "skipped";
      entry.message = "model not supported";
      entries.push_back(std::move(entry));
      continue;
    }

    try {
      auto match = mapping.resolve(t.topic);
      if (!match) {
        entry.status = "skipped";
        entry.message = "no matching rule";
      } else {
        entry.out_topic = match->out_topic;
        entry.frame_id = match->frame_id;
        entry.status = "ok";
      }
    } catch (const std::runtime_error &) {
      // TopicMapping::resolve only throws for duplicate-rule match. Keep
      // the dry-run cell short; the full library message is only useful
      // when convert() itself surfaces the exception.
      entry.status = "error";
      entry.message = "multiple rules match";
    }
    entries.push_back(std::move(entry));
  }
  return entries;
}

ConvertResult convert(const ConvertOptions & options)
{
  const auto in_spec = detect_input(options.input_path);

  // Phase 1: quick inspect to learn each packet topic's sniffed identity
  // before we open the main reader. Decoding vs passthrough hinges on
  // `SupportRegistry::check(identity)`: a topic that matches a mapping
  // rule but whose vendor/model is not supported is demoted to
  // passthrough instead of producing an empty PointCloud2 topic.
  const auto summary = inspect(options.input_path);
  std::unordered_map<std::string, std::optional<Identity>> identity_by_topic;
  for (const auto & t : summary.topics) {
    identity_by_topic.emplace(t.topic, t.identity);
  }

  rosbag2_cpp::Reader reader;
  reader.open(to_storage_options(in_spec));

  const auto discovered = discover_topics(reader);
  auto partition = resolve_discovered_topics(discovered.packet_topics, options.mapping);

  // Filter mapping-matched topics by the SupportRegistry. Topics the
  // registry rejects are demoted to passthrough so their raw packets
  // still land in the output bag (Continental radar, Robosense,
  // unidentified streams, etc.).
  const auto & registry = SupportRegistry::instance();
  std::vector<ResolvedRule> decoded_rules;
  decoded_rules.reserve(partition.resolved.size());
  std::vector<std::string> demoted_topics;
  for (auto & r : partition.resolved) {
    std::optional<Identity> id;
    if (auto it = identity_by_topic.find(r.in_topic); it != identity_by_topic.end()) {
      id = it->second;
    }
    if (registry.check(id).level == SupportLevel::Supported) {
      decoded_rules.push_back(std::move(r));
    } else {
      demoted_topics.push_back(r.in_topic);
    }
  }

  std::unordered_set<std::string> decoded_topic_names;
  decoded_topic_names.reserve(decoded_rules.size());
  for (const auto & r : decoded_rules) {
    decoded_topic_names.insert(r.in_topic);
  }

  // Snapshot every topic in the bag with its original metadata so we
  // can recreate it verbatim on the output side (everything that is
  // not a decoded packet topic gets copied through unchanged).
  std::vector<rosbag2_storage::TopicMetadata> all_topic_metadata;
  all_topic_metadata.reserve(reader.get_metadata().topics_with_message_count.size());
  for (const auto & info : reader.get_metadata().topics_with_message_count) {
    all_topic_metadata.push_back(info.topic_metadata);
  }

  // Mirror the input's file-vs-directory layout on the output side.
  // rosbag2_cpp::Writer always materialises a directory (metadata.yaml +
  // <base>_<chunk>.ext), so to emit a bare file for bare-file input we
  // let the writer create a sibling scratch directory, then rename the
  // single storage file to the user's requested path once the writer has
  // closed.
  const fs::path final_output_path{options.output_path};
  const bool collapse_to_file = !in_spec.is_directory;

  if (fs::exists(final_output_path)) {
    throw std::runtime_error(
      "output path already exists: " + final_output_path.string() + " (refusing to overwrite)");
  }

  const fs::path writer_uri = [&]() -> fs::path {
    if (!collapse_to_file) {
      return final_output_path;
    }
    fs::path parent = final_output_path.parent_path();
    if (parent.empty()) {
      parent = ".";
    }
    fs::path scratch = parent / (std::string{"."} + final_output_path.filename().string() +
                                 ".nebuladec_writer_scratch");
    // Defensive cleanup for leftovers from a prior failed run; Writer::
    // open() fails if the uri already exists.
    fs::remove_all(scratch);
    return scratch;
  }();

  // Only `states` needs to outlive the writer scope -- ConvertResult
  // assembly below reads per-topic decoder stats from it. Every other
  // bookkeeping map lives inside the writer scope so cppcheck's
  // variableScope check stays happy.
  TopicStateMap states;
  std::vector<std::string> passthrough_topics;  // declared for result; filled below.

  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions out_opts;
    out_opts.uri = writer_uri.string();
    out_opts.storage_id = in_spec.storage_id;  // mirror input plugin
    writer.open(out_opts);

    std::unordered_map<std::string, std::string> out_topic_by_in;
    std::unordered_map<std::string, std::string> frame_id_by_in;
    std::unordered_map<std::string, std::int64_t> last_stamp_by_in;
    std::unordered_set<std::string> created_out_topics;

    // 1) Create PointCloud2 output topics for every decoded rule.
    for (const auto & r : decoded_rules) {
      const auto vendor_hint = vendor_from_message_type(r.type);

      // Guard against two rules resolving to the same output topic with
      // different frame_ids -- rosbag2 allows two create_topic calls only
      // when name+type+serialization match exactly, and the downstream
      // writer.write() dispatches on name alone. Merging silently would
      // drop one of the frame_ids, so we refuse instead.
      if (!created_out_topics.insert(r.match.out_topic).second) {
        throw std::runtime_error(
          "mapping resolves multiple in_topics to the same out_topic '" + r.match.out_topic + "'");
      }

      TopicState state;
      state.topic = r.in_topic;
      state.type = r.type;
      state.vendor_hint = vendor_hint;
      state.packet_source = make_packet_source(r.type);
      state.decoder.set_vendor_hint(vendor_hint);
      states.emplace(r.in_topic, std::move(state));

      out_topic_by_in.emplace(r.in_topic, r.match.out_topic);
      frame_id_by_in.emplace(r.in_topic, r.match.frame_id);
      last_stamp_by_in.emplace(r.in_topic, 0);

      rosbag2_storage::TopicMetadata meta;
      meta.name = r.match.out_topic;
      meta.type = "sensor_msgs/msg/PointCloud2";
      meta.serialization_format = "cdr";
      writer.create_topic(meta);
    }

    // 2) Create passthrough topics for everything else, preserving the
    // original type and serialization_format.
    std::unordered_set<std::string> passthrough_topic_set;
    for (const auto & meta : all_topic_metadata) {
      if (decoded_topic_names.count(meta.name)) {
        continue;
      }
      writer.create_topic(meta);
      passthrough_topic_set.insert(meta.name);
      passthrough_topics.push_back(meta.name);
    }

    auto sink = [&](
                  const std::string & in_topic, nebula::drivers::NebulaPointCloudPtr cloud,
                  std::int64_t stamp_ns) {
      if (!cloud || cloud->empty()) {
        return;
      }
      const auto & out_topic = out_topic_by_in.at(in_topic);
      const auto & frame_id = frame_id_by_in.at(in_topic);
      const auto pc_msg = to_point_cloud2(*cloud, rclcpp::Time(stamp_ns), frame_id);
      writer.write(pc_msg, out_topic, rclcpp::Time(stamp_ns));
    };

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();

      // 3a) Decoded packet topic: decode and emit PointCloud2; the raw
      // packet is NOT passed through -- the PointCloud2 output replaces
      // it by design.
      if (auto it = states.find(bag_msg->topic_name); it != states.end()) {
        if (!it->second.packet_source) {
          continue;
        }
        rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
        auto packets = it->second.packet_source->extract(serialized);
        const std::string & in_topic = it->first;
        for (const auto & pkt : packets) {
          last_stamp_by_in[in_topic] = pkt.stamp_ns;
          feed_packet(
            it->second, pkt,
            [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t stamp_ns) {
              sink(in_topic, cloud, stamp_ns);
            });
        }
        continue;
      }

      // 3b) Passthrough: write the serialized message verbatim under
      // its original topic name. Covers unmatched packet topics, demoted
      // packet topics (Robosense / Continental radar / unidentified),
      // and any unrelated streams (TF, IMU, camera, ...).
      if (passthrough_topic_set.count(bag_msg->topic_name)) {
        writer.write(bag_msg);
      }
    }

    // Flush trailing scans held by mechanical-LiDAR decoders (see
    // single-topic convert() history for rationale).
    for (auto & [in_topic, state] : states) {
      if (auto trailing = state.decoder.flush(); trailing && *trailing) {
        ++state.clouds_produced;
        sink(in_topic, *trailing, last_stamp_by_in[in_topic]);
      }
    }
  }  // writer scope ends here -- destructor flushes and closes files.

  // If the input was a bare single file, collapse the scratch directory
  // down to a single storage file at the user-requested path.
  if (collapse_to_file) {
    const std::string ext = storage_ext_for_id(in_spec.storage_id);
    fs::path found;
    for (const auto & entry : fs::directory_iterator(writer_uri)) {
      if (entry.is_regular_file() && entry.path().extension() == ext) {
        if (!found.empty()) {
          // Writer should only emit one storage file per bag when no
          // split is configured; failing loudly here is safer than
          // silently dropping chunks.
          throw std::runtime_error(
            "writer produced multiple " + ext +
            " files; bare-file output mode does not support split bags");
        }
        found = entry.path();
      }
    }
    if (found.empty()) {
      throw std::runtime_error("writer produced no " + ext + " file at " + writer_uri.string());
    }
    fs::rename(found, final_output_path);
    fs::remove_all(writer_uri);
  }

  ConvertResult result;
  result.topics.reserve(decoded_rules.size());
  for (const auto & r : decoded_rules) {
    const auto & state = states.at(r.in_topic);
    TopicConvertResult tr;
    tr.in_topic = r.in_topic;
    tr.out_topic = r.match.out_topic;
    tr.frame_id = r.match.frame_id;
    tr.identity = state.sniffed_identity ? state.sniffed_identity : state.decoder.identity();
    tr.data_packets = state.data_packets;
    tr.clouds_written = state.clouds_produced;
    result.topics.push_back(std::move(tr));
  }
  result.passthrough_topics = std::move(passthrough_topics);
  return result;
}

}  // namespace nebuladec::bag
