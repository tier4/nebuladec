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

#include "convert_parallel.hpp"
#include "mcap_definition_writer.hpp"
#include "nebuladec_bag/message_definition.hpp"
#include "nebuladec_bag/point_cloud2.hpp"
#include "output_writer.hpp"
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
#include <thread>
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
  std::size_t packets{0};
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
  const std::unordered_map<std::string, std::size_t> & message_count_by_topic)
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
    const auto count_it = message_count_by_topic.find(pt.topic);
    result.message_count = count_it != message_count_by_topic.end() ? count_it->second : 0U;
    result.has_messages = result.message_count > 0;
    if (auto it = topic_states.find(pt.topic); it != topic_states.end()) {
      result.identity = it->second.sniffed_identity;
    }
    summary.topics.push_back(std::move(result));
  }
  return summary;
}

/// Cleanup wrapper so early returns always close the DB / finalize the
/// statement. Keeps the inspect_sqlite3_file body linear.
///
/// Both guards own a raw libsqlite3 handle, so copy/move must be banned
/// (C.21 / R.11): a duplicated pointer would lead to double-finalize /
/// double-close once both copies go out of scope. The guards are only
/// ever used as scoped locals, so suppressing all four special members
/// is sufficient.
struct SqliteStmtGuard
{
  sqlite3_stmt * stmt{nullptr};
  SqliteStmtGuard() = default;
  ~SqliteStmtGuard()
  {
    if (stmt) {
      sqlite3_finalize(stmt);
    }
  }
  SqliteStmtGuard(const SqliteStmtGuard &) = delete;
  SqliteStmtGuard & operator=(const SqliteStmtGuard &) = delete;
  SqliteStmtGuard(SqliteStmtGuard &&) = delete;
  SqliteStmtGuard & operator=(SqliteStmtGuard &&) = delete;
};

struct SqliteGuard
{
  sqlite3 * db{nullptr};
  SqliteGuard() = default;
  ~SqliteGuard()
  {
    if (db) {
      sqlite3_close(db);
    }
  }
  SqliteGuard(const SqliteGuard &) = delete;
  SqliteGuard & operator=(const SqliteGuard &) = delete;
  SqliteGuard(SqliteGuard &&) = delete;
  SqliteGuard & operator=(SqliteGuard &&) = delete;
};

struct Sqlite3TopicRow
{
  int id{0};
  std::string name;
  std::string type;
};

/// Mirror each topic's decoder identity (when known) onto its
/// `sniffed_identity` so the summary reflects the latest information
/// without re-feeding packets. Shared between the SQLite3 fast path and
/// the rosbag2 inspect path.
void refresh_sniffed_identities(TopicStateMap & states)
{
  for (auto & entry : states) {
    auto & state = entry.second;
    if (auto decoder_id = state.decoder.identity(); decoder_id) {
      state.sniffed_identity = decoder_id;
    }
  }
}

/// `SELECT id, name, type FROM topics` returned verbatim, in bag-insertion
/// order. Used by the SQLite3 fast inspect path to build the packet-topic
/// state map without going through `rosbag2_cpp::Reader`.
std::vector<Sqlite3TopicRow> query_sqlite3_topics(sqlite3 * db)
{
  std::vector<Sqlite3TopicRow> rows;
  SqliteStmtGuard sg;
  if (
    sqlite3_prepare_v2(db, "SELECT id, name, type FROM topics", -1, &sg.stmt, nullptr) !=
    SQLITE_OK) {
    throw std::runtime_error(std::string{"failed to query topics: "} + sqlite3_errmsg(db));
  }
  while (sqlite3_step(sg.stmt) == SQLITE_ROW) {
    Sqlite3TopicRow row;
    row.id = sqlite3_column_int(sg.stmt, 0);
    row.name = std::string{reinterpret_cast<const char *>(sqlite3_column_text(sg.stmt, 1))};
    row.type = std::string{reinterpret_cast<const char *>(sqlite3_column_text(sg.stmt, 2))};
    rows.push_back(std::move(row));
  }
  return rows;
}

/// Per-packet bookkeeping built once per inspect_sqlite3_file call.
/// `target_ids` is the input to the aggregate first-message query;
/// `topic_id_to_name` maps the int topic_id rows in the bag back to
/// topic names for the summary.
struct Sqlite3PacketIndex
{
  std::vector<PacketTopicSpec> packet_order;
  std::unordered_map<int, std::string> topic_id_to_name;
  TopicStateMap topic_states;
  std::vector<int> target_ids;
};

Sqlite3PacketIndex build_packet_index_from_rows(const std::vector<Sqlite3TopicRow> & rows)
{
  Sqlite3PacketIndex idx;
  for (const auto & r : rows) {
    if (!is_packet_type(r.type)) {
      continue;
    }
    idx.packet_order.push_back({r.name, r.type});
    TopicState state;
    state.topic = r.name;
    state.type = r.type;
    state.vendor_hint = vendor_from_message_type(r.type);
    state.packet_source = make_packet_source(r.type);
    state.decoder.set_vendor_hint(state.vendor_hint);
    idx.topic_states.emplace(r.name, std::move(state));
    idx.topic_id_to_name.emplace(r.id, r.name);
    idx.target_ids.push_back(r.id);
  }
  return idx;
}

/// Run the aggregate first-message query and feed each row through its
/// `TopicState`. `topics_with_messages` is populated as a side effect:
/// every non-empty packet topic produces exactly one row, so this also
/// derives the `has_messages` flag without a second scan.
///
/// `MIN(id)` is used in the inner aggregate (not `MIN(timestamp)`) so it
/// resolves directly off the integer primary key. rosbag2 writes rows in
/// timestamp order so the two coincide in practice, and for vendor/model
/// sniffing any early packet is sufficient.
void sniff_first_messages_sqlite3(
  sqlite3 * db, const std::vector<int> & target_ids,
  const std::unordered_map<int, std::string> & topic_id_to_name, TopicStateMap & topic_states)
{
  std::string q =
    "SELECT data, topic_id FROM messages WHERE id IN ("
    "SELECT MIN(id) FROM messages WHERE topic_id IN (";
  for (std::size_t i = 0; i < target_ids.size(); ++i) {
    q += (i == 0) ? "?" : ",?";
  }
  q += ") GROUP BY topic_id)";

  SqliteStmtGuard sg;
  if (sqlite3_prepare_v2(db, q.c_str(), -1, &sg.stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
      std::string{"failed to prepare first-message query: "} + sqlite3_errmsg(db));
  }
  for (std::size_t i = 0; i < target_ids.size(); ++i) {
    sqlite3_bind_int(sg.stmt, static_cast<int>(i + 1), target_ids[i]);
  }

  while (sqlite3_step(sg.stmt) == SQLITE_ROW) {
    const void * blob = sqlite3_column_blob(sg.stmt, 0);
    const int blob_size = sqlite3_column_bytes(sg.stmt, 0);
    const int topic_id = sqlite3_column_int(sg.stmt, 1);
    // sqlite3_column_bytes() returns int; guard against negative values
    // so the size_t casts below cannot silently overflow into a huge
    // buffer allocation (ES.46 / I.6).
    if (blob_size < 0) {
      continue;
    }
    auto name_it = topic_id_to_name.find(topic_id);
    if (name_it == topic_id_to_name.end()) {
      continue;
    }

    const auto blob_bytes = static_cast<std::size_t>(blob_size);
    // Wrap the raw CDR-serialized bytes in a rclcpp::SerializedMessage
    // without copying them a second time downstream -- packet_source
    // only reads from the buffer.
    rclcpp::SerializedMessage serialized(blob_bytes);
    auto & raw = serialized.get_rcl_serialized_message();
    std::memcpy(raw.buffer, blob, blob_bytes);
    raw.buffer_length = blob_bytes;

    auto ts_it = topic_states.find(name_it->second);
    if (ts_it == topic_states.end() || !ts_it->second.packet_source) {
      continue;
    }
    auto packets = ts_it->second.packet_source->extract(serialized);
    for (const auto & pkt : packets) {
      feed_packet(ts_it->second, pkt, nullptr);
    }
  }
}

/// Per-topic message count from the bare `.db3`. Runs a single grouped
/// COUNT(*) over the integer-keyed `topic_id` column so it stays cheap
/// even on large bags.
std::unordered_map<std::string, std::size_t> count_messages_per_topic_sqlite3(
  sqlite3 * db, const std::vector<int> & target_ids,
  const std::unordered_map<int, std::string> & topic_id_to_name)
{
  std::unordered_map<std::string, std::size_t> counts;
  counts.reserve(target_ids.size());
  if (target_ids.empty()) {
    return counts;
  }
  std::string q = "SELECT topic_id, COUNT(*) FROM messages WHERE topic_id IN (";
  for (std::size_t i = 0; i < target_ids.size(); ++i) {
    q += (i == 0) ? "?" : ",?";
  }
  q += ") GROUP BY topic_id";

  SqliteStmtGuard sg;
  if (sqlite3_prepare_v2(db, q.c_str(), -1, &sg.stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
      std::string{"failed to prepare per-topic count query: "} + sqlite3_errmsg(db));
  }
  for (std::size_t i = 0; i < target_ids.size(); ++i) {
    sqlite3_bind_int(sg.stmt, static_cast<int>(i + 1), target_ids[i]);
  }
  while (sqlite3_step(sg.stmt) == SQLITE_ROW) {
    const int topic_id = sqlite3_column_int(sg.stmt, 0);
    const sqlite3_int64 count = sqlite3_column_int64(sg.stmt, 1);
    if (count < 0) {
      continue;
    }
    auto name_it = topic_id_to_name.find(topic_id);
    if (name_it == topic_id_to_name.end()) {
      continue;
    }
    counts.emplace(name_it->second, static_cast<std::size_t>(count));
  }
  return counts;
}

/// Inspect a bare `.db3` file without going through `rosbag2_cpp::Reader`.
///
/// `rosbag2_cpp::Reader::open()` reconstructs `BagMetadata` (COUNT / MIN
/// / MAX over `messages`) whenever `metadata.yaml` is absent, which on a
/// 17GB / 941k-row bag takes ~16s. inspect() does not consume any of that
/// metadata, so we bypass the reader and talk to libsqlite3 directly via
/// the helpers above:
///
///   1) `query_sqlite3_topics`          -- SELECT id, name, type FROM topics
///   2) `build_packet_index_from_rows`  -- filter to packet topics
///   3) `sniff_first_messages_sqlite3`  -- aggregate first-message scan
///   4) `refresh_sniffed_identities`    -- mirror decoder identity
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

  const auto topic_rows = query_sqlite3_topics(db_guard.db);
  auto idx = build_packet_index_from_rows(topic_rows);
  if (idx.packet_order.empty()) {
    return {};
  }

  sniff_first_messages_sqlite3(db_guard.db, idx.target_ids, idx.topic_id_to_name, idx.topic_states);
  const auto message_counts =
    count_messages_per_topic_sqlite3(db_guard.db, idx.target_ids, idx.topic_id_to_name);

  refresh_sniffed_identities(idx.topic_states);
  return build_summary(idx.packet_order, idx.topic_states, message_counts);
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

  refresh_sniffed_identities(topic_states);

  return build_summary(discovered.packet_topics, topic_states, message_counts);
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
    entry.packets = t.message_count;

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

namespace
{

/// Flatten `summary.topics` into a name -> identity map for the convert
/// pipeline's support-filter step.
std::unordered_map<std::string, std::optional<Identity>> build_identity_index(
  const InspectSummary & summary)
{
  std::unordered_map<std::string, std::optional<Identity>> out;
  out.reserve(summary.topics.size());
  for (const auto & t : summary.topics) {
    out.emplace(t.topic, t.identity);
  }
  return out;
}

struct SupportPartition
{
  std::vector<ResolvedRule> decoded;
  std::vector<std::string> demoted;
};

/// Split resolved rules by `SupportRegistry::check()`. Topics the
/// registry rejects are demoted to passthrough so their raw packets
/// still land in the output bag (Continental radar, Robosense,
/// unidentified streams, ...).
SupportPartition partition_by_support(
  std::vector<ResolvedRule> resolved,
  const std::unordered_map<std::string, std::optional<Identity>> & identity_by_topic)
{
  const auto & registry = SupportRegistry::instance();
  SupportPartition out;
  out.decoded.reserve(resolved.size());
  for (auto & r : resolved) {
    std::optional<Identity> id;
    if (auto it = identity_by_topic.find(r.in_topic); it != identity_by_topic.end()) {
      id = it->second;
    }
    if (registry.check(id).level == SupportLevel::Supported) {
      out.decoded.push_back(std::move(r));
    } else {
      out.demoted.push_back(r.in_topic);
    }
  }
  return out;
}

/// Snapshot every topic in the bag with its original metadata so the
/// writer phase can recreate non-decoded topics verbatim.
std::vector<rosbag2_storage::TopicMetadata> collect_topic_metadata(
  const rosbag2_cpp::Reader & reader)
{
  const auto & topics = reader.get_metadata().topics_with_message_count;
  std::vector<rosbag2_storage::TopicMetadata> out;
  out.reserve(topics.size());
  for (const auto & info : topics) {
    out.push_back(info.topic_metadata);
  }
  return out;
}

/// rosbag2_cpp::Writer always materialises a directory (metadata.yaml +
/// <base>_<chunk>.ext). For bare-file input we want a bare-file output,
/// so the writer first creates a sibling scratch directory; the caller
/// later renames its single storage file to `final_output_path`. For
/// directory input we point the writer at `final_output_path` directly.
fs::path compute_writer_uri(const fs::path & final_output_path, bool collapse_to_file)
{
  if (!collapse_to_file) {
    return final_output_path;
  }
  const fs::path raw_parent = final_output_path.parent_path();
  fs::path scratch =
    (raw_parent.empty() ? fs::path{"."} : raw_parent) /
    (std::string{"."} + final_output_path.filename().string() + ".nebuladec_writer_scratch");
  // Defensive cleanup for leftovers from a prior failed run; Writer::
  // open() fails if the uri already exists.
  fs::remove_all(scratch);
  return scratch;
}

/// Per-input-topic routing tables for the writer phase. Kept in one
/// struct so convert() doesn't have to thread three parallel maps
/// through helper signatures.
struct DecodedRoutingTables
{
  std::unordered_map<std::string, std::string> out_topic_by_in;
  std::unordered_map<std::string, std::string> frame_id_by_in;
  std::unordered_map<std::string, std::int64_t> last_stamp_by_in;
};

/// Create one PointCloud2 output topic per decoded rule and populate
/// `states` with a fresh decoder. Throws when two rules resolve to the
/// same out_topic -- rosbag2 allows duplicate create_topic only when
/// name+type+serialization match exactly, and writer.write() dispatches
/// on name alone, so silent merging would drop one of the frame_ids.
DecodedRoutingTables register_decoded_topics(
  OutputWriter & writer, const std::vector<ResolvedRule> & decoded_rules, TopicStateMap & states)
{
  DecodedRoutingTables out;
  std::unordered_set<std::string> created_out_topics;
  for (const auto & r : decoded_rules) {
    if (!created_out_topics.insert(r.match.out_topic).second) {
      throw std::runtime_error(
        "mapping resolves multiple in_topics to the same out_topic '" + r.match.out_topic + "'");
    }
    const auto vendor_hint = vendor_from_message_type(r.type);

    TopicState state;
    state.topic = r.in_topic;
    state.type = r.type;
    state.vendor_hint = vendor_hint;
    state.packet_source = make_packet_source(r.type);
    state.decoder.set_vendor_hint(vendor_hint);
    states.emplace(r.in_topic, std::move(state));

    out.out_topic_by_in.emplace(r.in_topic, r.match.out_topic);
    out.frame_id_by_in.emplace(r.in_topic, r.match.frame_id);
    out.last_stamp_by_in.emplace(r.in_topic, 0);

    writer.create_topic(r.match.out_topic, "sensor_msgs/msg/PointCloud2", "cdr");
  }
  return out;
}

struct PassthroughTopics
{
  std::unordered_set<std::string> by_name;
  std::vector<std::string> ordered;
};

/// Create passthrough topics for everything that isn't a decoded packet
/// topic, preserving the original type and serialization_format. Returns
/// the per-name lookup set (for the writer loop) plus the bag-order list
/// (for `ConvertResult::passthrough_topics`).
PassthroughTopics register_passthrough_topics(
  OutputWriter & writer, const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata,
  const std::unordered_set<std::string> & decoded_topic_names)
{
  PassthroughTopics out;
  out.ordered.reserve(all_topic_metadata.size());
  for (const auto & meta : all_topic_metadata) {
    if (decoded_topic_names.count(meta.name)) {
      continue;
    }
    writer.create_topic(meta.name, meta.type, meta.serialization_format);
    out.by_name.insert(meta.name);
    out.ordered.push_back(meta.name);
  }
  return out;
}

/// Pull the single storage file out of the writer's scratch directory
/// and rename it onto `final_output_path`. Throws if the writer split
/// the bag into multiple files (bare-file output cannot represent a
/// split bag).
void collapse_scratch_to_bare_file(
  const fs::path & writer_uri, const fs::path & final_output_path, const std::string & storage_id)
{
  const std::string ext = storage_ext_for_id(storage_id);
  fs::path found;
  for (const auto & entry : fs::directory_iterator(writer_uri)) {
    if (!entry.is_regular_file() || entry.path().extension() != ext) {
      continue;
    }
    if (!found.empty()) {
      throw std::runtime_error(
        "writer produced multiple " + ext +
        " files; bare-file output mode does not support split bags");
    }
    found = entry.path();
  }
  if (found.empty()) {
    throw std::runtime_error("writer produced no " + ext + " file at " + writer_uri.string());
  }
  fs::rename(found, final_output_path);
  fs::remove_all(writer_uri);
}

/// Assemble the public `ConvertResult` from the decoded rules and the
/// final TopicStateMap after the writer has flushed. Run after the
/// writer scope so decoder stats are final.
ConvertResult build_convert_result(
  const std::vector<ResolvedRule> & decoded_rules, const TopicStateMap & states,
  std::vector<std::string> passthrough_topics)
{
  ConvertResult result;
  result.topics.reserve(decoded_rules.size());
  for (const auto & r : decoded_rules) {
    const auto & state = states.at(r.in_topic);
    TopicConvertResult tr;
    tr.in_topic = r.in_topic;
    tr.out_topic = r.match.out_topic;
    tr.frame_id = r.match.frame_id;
    tr.identity = state.sniffed_identity ? state.sniffed_identity : state.decoder.identity();
    tr.packets = state.packets;
    tr.clouds_written = state.clouds_produced;
    result.topics.push_back(std::move(tr));
  }
  result.passthrough_topics = std::move(passthrough_topics);
  return result;
}

/// rosbag2 renamed `SerializedBagMessage::time_stamp` to `recv_timestamp`
/// between Humble and Iron, so we cannot reference either name directly
/// without breaking one of the two distros nebuladec supports. A small
/// inheritance-priority tag picks `time_stamp` first (Humble) and falls
/// back to `recv_timestamp` (Iron/Jazzy) via SFINAE on the member access
/// expression. Both fields carry a nanosecond-resolution log timestamp,
/// so the call site is encoding-agnostic.
template <int N>
struct LogTimePriority : LogTimePriority<N - 1>
{
};
template <>
struct LogTimePriority<0>
{
};

template <typename Msg>
auto bag_message_log_time_ns(const Msg & msg, LogTimePriority<1>) -> decltype(msg.time_stamp)
{
  return msg.time_stamp;
}

template <typename Msg>
auto bag_message_log_time_ns(const Msg & msg, LogTimePriority<0>) -> decltype(msg.recv_timestamp)
{
  return msg.recv_timestamp;
}

template <typename Msg>
auto bag_message_log_time_ns(const Msg & msg)
{
  return bag_message_log_time_ns(msg, LogTimePriority<1>{});
}

/// Largest divisor of `K` that is `<= cap`. Used to snap an explicit
/// `--workers N < K` to a value that divides K evenly so each worker
/// thread can own the same number of topics. When `K` is prime and
/// `cap < K`, the result degenerates to 1 (documented behaviour;
/// callers can work around it by passing `--workers K`).
std::size_t largest_divisor_le(std::size_t K, std::size_t cap)
{
  if (K == 0) {
    return 1;
  }
  if (cap >= K) {
    return K;
  }
  for (std::size_t d = cap; d >= 1; --d) {
    if (K % d == 0) {
      return d;
    }
  }
  return 1;  // unreachable when cap >= 1
}

/// Resolved execution mode + worker count, computed once from
/// `ConvertOptions` + the discovered packet-topic count `K`.
struct WorkerPolicy
{
  bool sequential{true};   ///< true ⇒ run the legacy single-threaded path
  std::size_t workers{0};  ///< unused when sequential == true; otherwise
                           ///< the size of the decoder worker pool
};

/// Decide whether to take the sequential (legacy) path or the 3-stage
/// pipeline, and -- in pipeline mode -- how many decoder workers to
/// spawn. Encapsulates the fall-back rules so callers do not have to
/// repeat them.
///
/// Fall-back conditions (any of which forces sequential):
///   - `options.sequential` is true (explicit caller request)
///   - `std::thread::hardware_concurrency() < 3` (pipeline needs at
///     least reader + 1 worker + writer threads to be useful)
///   - `K == 0` (no decoded LiDAR topics, so the worker stage has
///     nothing to do)
WorkerPolicy resolve_worker_policy(const ConvertOptions & options, std::size_t K)
{
  WorkerPolicy out;
  if (options.sequential) {
    out.sequential = true;
    return out;
  }
  const auto cores = std::thread::hardware_concurrency();
  if (cores < 3) {
    out.sequential = true;
    return out;
  }
  if (K == 0) {
    out.sequential = true;
    return out;
  }
  const std::size_t request =
    options.workers > 0 ? options.workers : std::min<std::size_t>(cores, K);
  std::size_t effective = std::min(request, K);
  if (effective < K) {
    effective = largest_divisor_le(K, effective);
  }
  out.sequential = false;
  out.workers = effective;
  return out;
}

// Type aliases to keep nested template expressions readable (and to
// dodge a cppcheck parser quirk that mis-tokenises chained `>>>`).
using BagMsgPtr = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;
using InputQueue = BoundedQueue<BagMsgPtr>;

/// One write request emitted to the shared writer queue. The variant
/// is encoded with `kind` rather than `std::variant` so the existing
/// codebase (C++17, no <variant> dependency) stays untouched.
struct WriteItem
{
  enum class Kind : std::uint8_t { Cloud, Passthrough };
  Kind kind{Kind::Passthrough};

  // Cloud fields (populated when kind == Cloud).
  std::string out_topic;
  std::string frame_id;
  nebula::drivers::NebulaPointCloudPtr cloud;

  // Passthrough fields (populated when kind == Passthrough).
  BagMsgPtr passthrough_msg;

  std::int64_t stamp_ns{0};  ///< cloud stamp or passthrough log_time
};

using WriteQueue = BoundedQueue<WriteItem>;

/// Per-decoded-topic resources for the parallel pipeline. Each slot
/// owns a `TopicState` (Decoder + PacketSource + counters) plus the
/// input queue that feeds packets to its worker.
struct DecodedTopicSlot
{
  std::string in_topic;
  std::string out_topic;
  std::string frame_id;
  TopicState state;
  // Heap-allocated because BoundedQueue is non-movable (holds a
  // mutex). Keeping it behind unique_ptr lets the enclosing slot move
  // freely inside the owning vector.
  std::unique_ptr<InputQueue> in_queue;
  std::int64_t last_pkt_stamp_ns{0};  ///< stamp for `Decoder::flush()`
};

/// Drives `N` topic slots on a single worker thread, draining their
/// input queues sequentially and pushing decoded clouds to the shared
/// `WriteQueue`. When a slot's input queue is closed + drained the
/// worker calls `Decoder::flush()` so the trailing scan still reaches
/// the writer.
class ParallelWorker
{
public:
  ParallelWorker(std::vector<DecodedTopicSlot *> slots, WriteQueue & write_queue, AbortFlag & abort)
  : slots_(std::move(slots)), write_queue_(write_queue), abort_(abort)
  {
  }

  void run()
  {
    try {
      // Process each assigned slot to completion. Slots assigned to
      // the same worker are independent topics, so finishing one
      // before starting the next does not stall the writer (the
      // shared write queue absorbs decoded clouds in arrival order).
      for (auto * slot : slots_) {
        BagMsgPtr msg;
        while (slot->in_queue->pop(msg)) {
          process_msg(*slot, *msg);
          if (abort_.aborted()) {
            return;
          }
        }
        if (abort_.aborted()) {
          return;
        }
        flush_slot(*slot);
      }
    } catch (...) {
      abort_.set(std::current_exception());
    }
  }

private:
  std::vector<DecodedTopicSlot *> slots_;
  WriteQueue & write_queue_;
  AbortFlag & abort_;

  void process_msg(DecodedTopicSlot & slot, const rosbag2_storage::SerializedBagMessage & msg)
  {
    if (!slot.state.packet_source) {
      return;
    }
    rclcpp::SerializedMessage serialized(*msg.serialized_data);
    auto packets = slot.state.packet_source->extract(serialized);
    ++slot.state.packets;
    for (const auto & pkt : packets) {
      slot.last_pkt_stamp_ns = pkt.stamp_ns;
      feed_packet(
        slot.state, pkt, [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t cloud_stamp) {
          push_cloud(slot, std::move(cloud), cloud_stamp);
        });
    }
  }

  void flush_slot(DecodedTopicSlot & slot)
  {
    auto trailing = slot.state.decoder.flush();
    if (!trailing || !*trailing) {
      return;
    }
    ++slot.state.clouds_produced;
    push_cloud(slot, *trailing, slot.last_pkt_stamp_ns);
  }

  void push_cloud(
    DecodedTopicSlot & slot, nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t cloud_stamp)
  {
    // Deep-copy the cloud before crossing the worker->writer thread
    // boundary. Some upstream nebula drivers (e.g. the Velodyne scan
    // decoder) re-use the cloud's point buffer across consecutive
    // scans, so the shared_ptr aliases storage that the driver will
    // mutate on the next `feed()`. The sequential path was insulated
    // because it wrote the cloud immediately; we queue it, so a
    // snapshot is required.
    auto cloud_snapshot = std::make_shared<nebula::drivers::NebulaPointCloud>(*cloud);
    WriteItem item;
    item.kind = WriteItem::Kind::Cloud;
    item.out_topic = slot.out_topic;
    item.frame_id = slot.frame_id;
    item.cloud = std::move(cloud_snapshot);
    item.stamp_ns = cloud_stamp;
    write_queue_.push(std::move(item));
  }
};

/// Create one `DecodedTopicSlot` per resolved rule, register each
/// PointCloud2 output topic on the writer, and populate
/// `slot_by_topic` so the reader can route incoming bag messages
/// without re-scanning the vector. Throws when two rules resolve to
/// the same `out_topic` (caller-supplied mapping ambiguity).
std::vector<std::unique_ptr<DecodedTopicSlot>> make_decoded_slots(
  const std::vector<ResolvedRule> & decoded_rules, AbortFlag & abort, OutputWriter & writer,
  std::unordered_map<std::string, DecodedTopicSlot *> & slot_by_topic)
{
  std::vector<std::unique_ptr<DecodedTopicSlot>> slots;
  slots.reserve(decoded_rules.size());
  std::unordered_set<std::string> created_out_topics;
  for (const auto & r : decoded_rules) {
    if (!created_out_topics.insert(r.match.out_topic).second) {
      throw std::runtime_error(
        "mapping resolves multiple in_topics to the same out_topic '" + r.match.out_topic + "'");
    }
    const auto vendor_hint = vendor_from_message_type(r.type);
    auto slot = std::make_unique<DecodedTopicSlot>();
    slot->in_topic = r.in_topic;
    slot->out_topic = r.match.out_topic;
    slot->frame_id = r.match.frame_id;
    slot->state.topic = r.in_topic;
    slot->state.type = r.type;
    slot->state.vendor_hint = vendor_hint;
    slot->state.packet_source = make_packet_source(r.type);
    slot->state.decoder.set_vendor_hint(vendor_hint);
    slot->in_queue = std::make_unique<InputQueue>(k_default_input_queue_capacity, abort);

    writer.create_topic(r.match.out_topic, "sensor_msgs/msg/PointCloud2", "cdr");
    slot_by_topic.emplace(r.in_topic, slot.get());
    slots.push_back(std::move(slot));
  }
  return slots;
}

/// Parallel driver: Reader -> per-topic Worker pool -> shared
/// WriteQueue -> Writer (on the calling thread).
///
/// Producers (reader + workers) all push to one shared `WriteQueue`,
/// which the writer drains FIFO. The graph is a strict producer ->
/// consumer DAG: no producer ever waits on another producer, only on
/// the writer's drainage. That removes the bootstrap deadlock the
/// previous K-way-merge design exhibited (the merger held the
/// passthrough head until every decoded topic's "watermark" advanced
/// past `INT64_MIN`; on a bag that opens with a burst of passthrough
/// messages, the reader filled the passthrough queue before any
/// decoded packet arrived, so no decoded watermark ever moved and
/// the entire pipeline parked in futex_wait).
///
/// The output multiset matches the sequential path (see
/// `test_convert_parallel_equivalence`); strict log-time ordering of
/// the file's insertion order is not asserted -- nor was it in the
/// sequential path, which writes a decoded cloud at the point in the
/// stream where its scan-completing packet arrives.
///
/// `workers` is the resolved worker-pool size (already clamped to a
/// divisor of `K = decoded_rules.size()` by `resolve_worker_policy`),
/// so the slot-to-worker assignment is an even `K / workers` split.
ConvertResult run_convert_parallel(
  rosbag2_cpp::Reader & reader, const std::vector<ResolvedRule> & decoded_rules,
  const std::unordered_set<std::string> & decoded_topic_names,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata, OutputWriter & writer,
  std::size_t workers)
{
  AbortFlag abort;

  // 1. Create one slot per decoded topic + register its output topic.
  std::unordered_map<std::string, DecodedTopicSlot *> slot_by_topic;
  auto slots = make_decoded_slots(decoded_rules, abort, writer, slot_by_topic);

  // 2. Register passthrough topics on the writer (main thread only,
  // before any worker starts).
  std::unordered_set<std::string> passthrough_by_name;
  std::vector<std::string> passthrough_ordered;
  passthrough_ordered.reserve(all_topic_metadata.size());
  for (const auto & meta : all_topic_metadata) {
    if (decoded_topic_names.count(meta.name) != 0U) {
      continue;
    }
    writer.create_topic(meta.name, meta.type, meta.serialization_format);
    passthrough_by_name.insert(meta.name);
    passthrough_ordered.push_back(meta.name);
  }

  // 3. Assign slots to workers in `K / workers` blocks. `workers` is
  // guaranteed to divide K (see `resolve_worker_policy`), so the
  // split is exact when slots are non-empty.
  std::vector<std::vector<DecodedTopicSlot *>> worker_assignments(workers);
  if (!slots.empty() && workers > 0) {
    const std::size_t per = slots.size() / workers;
    for (std::size_t w = 0; w < workers; ++w) {
      for (std::size_t i = 0; i < per; ++i) {
        worker_assignments[w].push_back(slots[(w * per) + i].get());
      }
    }
  }

  // 4. Shared writer queue. Sized to absorb writer lag without
  // unbounded memory growth; capacity is in items, not bytes.
  WriteQueue write_queue(k_default_output_queue_capacity, abort);

  // Producer tracking: the writer's queue must close when every
  // producer (reader + each spawned worker) has exited. Initialised
  // to 1 (reader) and bumped once per spawned worker below.
  std::atomic<std::size_t> active_producers{1};

  // 5. Launch worker threads.
  std::vector<std::thread> worker_threads;
  worker_threads.reserve(workers);
  for (std::size_t w = 0; w < workers; ++w) {
    if (worker_assignments[w].empty()) {
      continue;
    }
    active_producers.fetch_add(1, std::memory_order_acq_rel);
    worker_threads.emplace_back(
      [slots_for_worker = worker_assignments[w], &write_queue, &abort, &active_producers] {
        ParallelWorker pw(slots_for_worker, write_queue, abort);
        pw.run();
        if (active_producers.fetch_sub(1, std::memory_order_acq_rel) == 1U) {
          write_queue.close();
        }
      });
  }

  // 6. Launch the reader. It pushes decoded-topic messages into the
  // matching slot's in_queue and passthrough messages directly into
  // the shared write queue.
  std::thread reader_thread([&] {
    try {
      while (!abort.aborted() && reader.has_next()) {
        auto bag_msg = reader.read_next();
        if (!bag_msg) {
          continue;
        }
        const auto log_time_ns = bag_message_log_time_ns(*bag_msg);
        // Copy the topic name out of `bag_msg` before any std::move
        // hand-off so the lookup string is stable after move.
        const std::string topic_name{bag_msg->topic_name};
        if (auto it = slot_by_topic.find(topic_name); it != slot_by_topic.end()) {
          BagMsgPtr shared_msg(std::move(bag_msg));
          it->second->in_queue->push(std::move(shared_msg));
        } else if (passthrough_by_name.count(topic_name) != 0U) {
          WriteItem item;
          item.kind = WriteItem::Kind::Passthrough;
          item.passthrough_msg = BagMsgPtr(std::move(bag_msg));
          item.stamp_ns = log_time_ns;
          write_queue.push(std::move(item));
        }
        // Topics matching no rule are silently skipped (same as the
        // sequential path).
      }
    } catch (...) {
      abort.set(std::current_exception());
    }
    // Close all input queues so workers reach EOF, even on error.
    for (auto & slot : slots) {
      slot->in_queue->close();
    }
    if (active_producers.fetch_sub(1, std::memory_order_acq_rel) == 1U) {
      write_queue.close();
    }
  });

  // 7. Writer loop on the main thread. Drains the shared queue until
  // every producer has exited and the queue is closed + drained.
  try {
    WriteItem item;
    while (write_queue.pop(item)) {
      if (item.kind == WriteItem::Kind::Cloud) {
        // `TopicState::clouds_produced` is incremented inside
        // `feed_packet` / `flush_slot` on the worker thread; the
        // writer thread just emits the cloud here. This matches the
        // sequential path's accounting (clouds_produced counted at
        // emit time, not at write time).
        if (item.cloud && !item.cloud->empty()) {
          writer.write_cloud(item.out_topic, item.frame_id, *item.cloud, item.stamp_ns);
        }
      } else {
        writer.write_passthrough(item.passthrough_msg, item.stamp_ns);
      }
    }
  } catch (...) {
    abort.set(std::current_exception());
  }

  // 8. Join all background threads. RAII-safe even on exception.
  if (reader_thread.joinable()) {
    reader_thread.join();
  }
  for (auto & t : worker_threads) {
    if (t.joinable()) {
      t.join();
    }
  }

  // 9. Rethrow the first captured exception if anything went wrong.
  if (auto ep = abort.take()) {
    std::rethrow_exception(ep);
  }

  // 10. Assemble the result from the per-slot TopicStates.
  TopicStateMap final_states;
  for (auto & slot : slots) {
    final_states.emplace(slot->in_topic, std::move(slot->state));
  }
  return build_convert_result(decoded_rules, final_states, std::move(passthrough_ordered));
}

/// Dispatch helper used by both writer paths (`rosbag2_cpp::Writer`
/// and `McapDefinitionWriter`). Picks the sequential or parallel
/// driver based on the resolved `WorkerPolicy`.
ConvertResult dispatch_to_driver(
  rosbag2_cpp::Reader & reader, const std::vector<ResolvedRule> & decoded_rules,
  const std::unordered_set<std::string> & decoded_topic_names,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata, OutputWriter & writer,
  const WorkerPolicy & policy);

/// Shared sequential driver used by both the generic `rosbag2_cpp::Writer`
/// path and the MCAP schema-forwarding fast path. The caller owns the
/// concrete writer's lifetime; this helper only routes reads → decodes →
/// writes through the `OutputWriter` facade.
///
/// Behaviour is byte-equivalent to the pre-refactor `convert()` body: per
/// `bag_msg` we look up the topic, either feed packets through the
/// per-topic `Decoder` (emitting clouds on scan completion) or pass the
/// raw bytes through. After the read loop ends, every decoder is flushed
/// so the trailing scan is not dropped (regression-tested by
/// `test_convert_velodyne_vlp16` and `test_convert_hesai_qt128`).
ConvertResult run_convert_sequential(
  rosbag2_cpp::Reader & reader, const std::vector<ResolvedRule> & decoded_rules,
  const std::unordered_set<std::string> & decoded_topic_names,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata, OutputWriter & writer)
{
  TopicStateMap states;
  auto routing = register_decoded_topics(writer, decoded_rules, states);
  auto passthrough = register_passthrough_topics(writer, all_topic_metadata, decoded_topic_names);

  auto sink = [&](
                const std::string & in_topic, nebula::drivers::NebulaPointCloudPtr cloud,
                std::int64_t stamp_ns) {
    if (!cloud || cloud->empty()) {
      return;
    }
    writer.write_cloud(
      routing.out_topic_by_in.at(in_topic), routing.frame_id_by_in.at(in_topic), *cloud, stamp_ns);
  };

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();

    if (auto it = states.find(bag_msg->topic_name); it != states.end()) {
      if (!it->second.packet_source) {
        continue;
      }
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto packets = it->second.packet_source->extract(serialized);
      const std::string & in_topic = it->first;
      ++it->second.packets;
      for (const auto & pkt : packets) {
        routing.last_stamp_by_in[in_topic] = pkt.stamp_ns;
        feed_packet(
          it->second, pkt,
          [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t cloud_stamp) {
            sink(in_topic, cloud, cloud_stamp);
          });
      }
      continue;
    }

    if (passthrough.by_name.count(bag_msg->topic_name)) {
      const auto log_time_ns = bag_message_log_time_ns(*bag_msg);
      writer.write_passthrough(bag_msg, log_time_ns);
    }
  }

  // Flush trailing scans held by mechanical-LiDAR decoders. See
  // `test_convert_velodyne_vlp16.cpp` / `test_convert_hesai_qt128.cpp`
  // for the bugs this guards against.
  for (auto & [in_topic, state] : states) {
    auto trailing = state.decoder.flush();
    if (!trailing || !*trailing) {
      continue;
    }
    ++state.clouds_produced;
    sink(in_topic, *trailing, routing.last_stamp_by_in[in_topic]);
  }

  return build_convert_result(decoded_rules, states, std::move(passthrough.ordered));
}

/// Definition of the dispatch helper forward-declared above. Routes
/// each `convert()` writer scope through either the sequential body
/// or the parallel pipeline based on the resolved policy.
ConvertResult dispatch_to_driver(
  rosbag2_cpp::Reader & reader, const std::vector<ResolvedRule> & decoded_rules,
  const std::unordered_set<std::string> & decoded_topic_names,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata, OutputWriter & writer,
  const WorkerPolicy & policy)
{
  if (policy.sequential) {
    return run_convert_sequential(
      reader, decoded_rules, decoded_topic_names, all_topic_metadata, writer);
  }
  return run_convert_parallel(
    reader, decoded_rules, decoded_topic_names, all_topic_metadata, writer, policy.workers);
}

/// Bare-file MCAP convert path that bypasses `rosbag2_cpp::Writer` so
/// schema records can be sourced from the input bag's embedded
/// definitions. Entered only when the input bag is MCAP, output mirrors
/// it as a single file, and `registry` carries at least one definition.
ConvertResult convert_via_definition_writer(
  rosbag2_cpp::Reader & reader, const std::vector<ResolvedRule> & decoded_rules,
  const std::unordered_set<std::string> & decoded_topic_names,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata,
  const fs::path & final_output_path, const MessageDefinitionRegistry & registry,
  const WorkerPolicy & policy)
{
  if (fs::exists(final_output_path)) {
    throw std::runtime_error(
      "output path already exists: " + final_output_path.string() + " (refusing to overwrite)");
  }

  ConvertResult result;
  {
    McapDefinitionWriter mcap_writer{final_output_path, registry};
    McapDefinitionOutputWriter writer{mcap_writer};
    result = dispatch_to_driver(
      reader, decoded_rules, decoded_topic_names, all_topic_metadata, writer, policy);
  }  // mcap_writer destructor finalises the mcap file
  return result;
}

}  // namespace

ConvertResult convert(const ConvertOptions & options)
{
  const auto in_spec = detect_input(options.input_path);

  // Phase 1: quick inspect so we know each packet topic's sniffed
  // identity before opening the main reader. Decoding vs passthrough
  // hinges on `SupportRegistry::check(identity)`: a topic that matches
  // a mapping rule but whose vendor/model is not supported is demoted
  // to passthrough instead of producing an empty PointCloud2 topic.
  const auto identity_by_topic = build_identity_index(inspect(options.input_path));

  rosbag2_cpp::Reader reader;
  reader.open(to_storage_options(in_spec));

  // Phase 2: discover packet topics, resolve them against the mapping,
  // then split by SupportRegistry into decoded vs demoted.
  const auto discovered = discover_topics(reader);
  auto partition = resolve_discovered_topics(discovered.packet_topics, options.mapping);
  auto support = partition_by_support(std::move(partition.resolved), identity_by_topic);

  std::unordered_set<std::string> decoded_topic_names;
  decoded_topic_names.reserve(support.decoded.size());
  for (const auto & r : support.decoded) {
    decoded_topic_names.insert(r.in_topic);
  }
  const auto all_topic_metadata = collect_topic_metadata(reader);

  // Resolve the execution policy (sequential vs 3-stage pipeline)
  // once we know how many decode-target topics the bag carries.
  // Default is pipeline; auto-falls-back to sequential when
  // `options.sequential` is true, `hardware_concurrency() < 3`, or
  // K (= support.decoded.size()) is zero. The resolved `workers`
  // count is already clamped to a divisor of K so each worker thread
  // can own an even share of topics (`K / workers`).
  const auto worker_policy = resolve_worker_policy(options, support.decoded.size());

  // Mirror the input's file-vs-directory layout on the output side.
  const fs::path final_output_path{options.output_path};
  const bool collapse_to_file = !in_spec.is_directory;

  // Schema-forwarding fast path: when the input bag carries embedded
  // message definitions and we are writing a bare-file MCAP, take the
  // definition-writer branch so unknown-but-embedded types make it
  // into the output. The branch is intentionally narrow:
  //
  //   * input must be MCAP -- only MCAP guarantees Schema records on
  //     the read side.
  //   * output must be bare-file -- the directory layout (multiple
  //     chunked mcaps + metadata.yaml) is out of scope for this fix.
  //   * registry must be non-empty -- otherwise the existing
  //     `rosbag2_cpp::Writer` path is byte-equivalent and we avoid
  //     touching well-tested code.
  if (in_spec.storage_id == "mcap" && collapse_to_file) {
    auto registry = load_definition_registry(in_spec);
    if (!registry.empty()) {
      return convert_via_definition_writer(
        reader, support.decoded, decoded_topic_names, all_topic_metadata, final_output_path,
        registry, worker_policy);
    }
  }

  if (fs::exists(final_output_path)) {
    throw std::runtime_error(
      "output path already exists: " + final_output_path.string() + " (refusing to overwrite)");
  }
  const auto writer_uri = compute_writer_uri(final_output_path, collapse_to_file);

  // Phase 3: open the writer, register output topics, run the
  // reader-loop (sequential or parallel per `worker_policy`), then
  // flush trailing scans before the writer closes.
  ConvertResult result;
  {
    rosbag2_cpp::Writer rb2_writer;
    rosbag2_storage::StorageOptions out_opts;
    out_opts.uri = writer_uri.string();
    out_opts.storage_id = in_spec.storage_id;  // mirror input plugin
    rb2_writer.open(out_opts);
    Rosbag2OutputWriter writer{rb2_writer};
    result = dispatch_to_driver(
      reader, support.decoded, decoded_topic_names, all_topic_metadata, writer, worker_policy);
  }  // rb2_writer scope ends here -- destructor flushes and closes files.

  // Phase 4: collapse scratch -> bare file for bare-file inputs, then
  // return the result.
  if (collapse_to_file) {
    collapse_scratch_to_bare_file(writer_uri, final_output_path, in_spec.storage_id);
  }
  return result;
}

}  // namespace nebuladec::bag
