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

#include "mcap_definition_writer.hpp"
#include "nebuladec_bag/message_definition.hpp"
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
  const std::unordered_map<int, std::string> & topic_id_to_name, TopicStateMap & topic_states,
  std::unordered_set<std::string> & topics_with_messages)
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
    topics_with_messages.insert(name_it->second);

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

  std::unordered_set<std::string> topics_with_messages;
  sniff_first_messages_sqlite3(
    db_guard.db, idx.target_ids, idx.topic_id_to_name, idx.topic_states, topics_with_messages);

  refresh_sniffed_identities(idx.topic_states);
  return build_summary(idx.packet_order, idx.topic_states, topics_with_messages);
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
  rosbag2_cpp::Writer & writer, const std::vector<ResolvedRule> & decoded_rules,
  TopicStateMap & states)
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

    rosbag2_storage::TopicMetadata meta;
    meta.name = r.match.out_topic;
    meta.type = "sensor_msgs/msg/PointCloud2";
    meta.serialization_format = "cdr";
    writer.create_topic(meta);
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
  rosbag2_cpp::Writer & writer,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata,
  const std::unordered_set<std::string> & decoded_topic_names)
{
  PassthroughTopics out;
  out.ordered.reserve(all_topic_metadata.size());
  for (const auto & meta : all_topic_metadata) {
    if (decoded_topic_names.count(meta.name)) {
      continue;
    }
    writer.create_topic(meta);
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
    tr.data_packets = state.data_packets;
    tr.clouds_written = state.clouds_produced;
    result.topics.push_back(std::move(tr));
  }
  result.passthrough_topics = std::move(passthrough_topics);
  return result;
}

/// Bare-file MCAP convert path that bypasses `rosbag2_cpp::Writer` so
/// schema records can be sourced from the input bag's embedded
/// definitions. Entered only when the input bag is MCAP, output mirrors
/// it as a single file, and `registry` carries at least one definition
/// -- i.e. the input bag has something the local environment may lack.
///
/// Reuses the already-opened `reader` and the `decoded`/`all_topic_*`
/// snapshots from the public `convert()` so this helper does no
/// rediscovery. The PointCloud2 produced by each decoded rule is
/// serialized with `rclcpp::Serialization` and handed to the
/// McapDefinitionWriter alongside passthrough bytes copied verbatim.
ConvertResult convert_via_definition_writer(
  rosbag2_cpp::Reader & reader, const std::vector<ResolvedRule> & decoded_rules,
  const std::unordered_set<std::string> & decoded_topic_names,
  const std::vector<rosbag2_storage::TopicMetadata> & all_topic_metadata,
  const fs::path & final_output_path, const MessageDefinitionRegistry & registry)
{
  if (fs::exists(final_output_path)) {
    throw std::runtime_error(
      "output path already exists: " + final_output_path.string() + " (refusing to overwrite)");
  }

  TopicStateMap states;
  std::unordered_map<std::string, std::string> out_topic_by_in;
  std::unordered_map<std::string, std::string> frame_id_by_in;
  std::unordered_map<std::string, std::int64_t> last_stamp_by_in;
  PassthroughTopics passthrough;

  {
    McapDefinitionWriter writer{final_output_path, registry};

    // Register decoded topics (PointCloud2 outputs) plus their state.
    // Mirrors `register_decoded_topics` but routes through the
    // definition writer; we keep the duplicate-output-topic guard so
    // the same ambiguity check fires regardless of writer kind.
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
      out_topic_by_in.emplace(r.in_topic, r.match.out_topic);
      frame_id_by_in.emplace(r.in_topic, r.match.frame_id);
      last_stamp_by_in.emplace(r.in_topic, 0);
      writer.create_topic(r.match.out_topic, "sensor_msgs/msg/PointCloud2", "cdr");
    }

    // Register passthrough topics in bag-metadata order.
    passthrough.ordered.reserve(all_topic_metadata.size());
    for (const auto & meta : all_topic_metadata) {
      if (decoded_topic_names.count(meta.name)) {
        continue;
      }
      writer.create_topic(meta.name, meta.type, meta.serialization_format);
      passthrough.by_name.insert(meta.name);
      passthrough.ordered.push_back(meta.name);
    }

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;

    auto sink = [&](
                  const std::string & in_topic, nebula::drivers::NebulaPointCloudPtr cloud,
                  std::int64_t stamp_ns) {
      if (!cloud || cloud->empty()) {
        return;
      }
      const auto & out_topic = out_topic_by_in.at(in_topic);
      const auto & frame_id = frame_id_by_in.at(in_topic);
      const auto pc_msg = to_point_cloud2(*cloud, rclcpp::Time(stamp_ns), frame_id);
      // Manual CDR serialization: the definition writer takes raw
      // bytes, so we cannot rely on rosbag2_cpp::Writer's typed
      // overload.
      rclcpp::SerializedMessage serialized;
      pc2_serializer.serialize_message(&pc_msg, &serialized);
      const auto & raw = serialized.get_rcl_serialized_message();
      writer.write_serialized(
        out_topic, reinterpret_cast<const std::byte *>(raw.buffer),  // NOLINT
        raw.buffer_length, stamp_ns, stamp_ns);
    };

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      const auto stamp_ns = bag_msg->time_stamp;

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
            [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t cloud_stamp) {
              sink(in_topic, cloud, cloud_stamp);
            });
        }
        continue;
      }

      if (passthrough.by_name.count(bag_msg->topic_name)) {
        // Pass the raw CDR bytes through untouched. SerializedBagMessage
        // owns a shared rcl buffer (rcl_serialized_message_t) backed by
        // a contiguous heap allocation -- we treat it as opaque bytes.
        const auto & raw = bag_msg->serialized_data;
        writer.write_serialized(
          bag_msg->topic_name, reinterpret_cast<const std::byte *>(raw->buffer),  // NOLINT
          raw->buffer_length, stamp_ns, stamp_ns);
      }
    }

    for (auto & [in_topic, state] : states) {
      auto trailing = state.decoder.flush();
      if (!trailing || !*trailing) {
        continue;
      }
      ++state.clouds_produced;
      sink(in_topic, *trailing, last_stamp_by_in[in_topic]);
    }
  }  // writer destructor finalises the mcap file

  return build_convert_result(decoded_rules, states, std::move(passthrough.ordered));
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

  // Mirror the input's file-vs-directory layout on the output side.
  const fs::path final_output_path{options.output_path};
  const bool collapse_to_file = !in_spec.is_directory;

  // Schema-forwarding fast path: when the input bag carries embedded
  // message definitions and we are writing a bare-file MCAP, take the
  // definition-writer branch so unknown-but-embedded types make it into
  // the output. The branch is intentionally narrow:
  //
  //   * input must be MCAP -- only MCAP guarantees Schema records on
  //     the read side.
  //   * output must be bare-file -- the directory layout (multiple
  //     chunked mcaps + metadata.yaml) is out of scope for this fix.
  //   * registry must be non-empty -- otherwise the existing
  //     `rosbag2_cpp::Writer` path is byte-equivalent and we avoid
  //     touching well-tested code.
  //
  // When any condition fails we fall through to the legacy writer
  // exactly as before, so existing tests stay on the same code path.
  if (in_spec.storage_id == "mcap" && collapse_to_file) {
    auto registry = load_definition_registry(in_spec);
    if (!registry.empty()) {
      return convert_via_definition_writer(
        reader, support.decoded, decoded_topic_names, all_topic_metadata, final_output_path,
        registry);
    }
  }

  if (fs::exists(final_output_path)) {
    throw std::runtime_error(
      "output path already exists: " + final_output_path.string() + " (refusing to overwrite)");
  }
  const auto writer_uri = compute_writer_uri(final_output_path, collapse_to_file);

  // Only `states` and `passthrough` need to outlive the writer scope:
  // build_convert_result reads per-topic decoder stats from `states`
  // and forwards `passthrough.ordered` into the result.
  TopicStateMap states;
  PassthroughTopics passthrough;

  // Phase 3: open the writer, register output topics, run the
  // reader-loop, then flush trailing scans before the writer closes.
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions out_opts;
    out_opts.uri = writer_uri.string();
    out_opts.storage_id = in_spec.storage_id;  // mirror input plugin
    writer.open(out_opts);

    auto routing = register_decoded_topics(writer, support.decoded, states);
    passthrough = register_passthrough_topics(writer, all_topic_metadata, decoded_topic_names);

    auto sink = [&](
                  const std::string & in_topic, nebula::drivers::NebulaPointCloudPtr cloud,
                  std::int64_t stamp_ns) {
      if (!cloud || cloud->empty()) {
        return;
      }
      const auto & out_topic = routing.out_topic_by_in.at(in_topic);
      const auto & frame_id = routing.frame_id_by_in.at(in_topic);
      const auto pc_msg = to_point_cloud2(*cloud, rclcpp::Time(stamp_ns), frame_id);
      writer.write(pc_msg, out_topic, rclcpp::Time(stamp_ns));
    };

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();

      // Decoded packet topic: decode and emit PointCloud2; the raw
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
          routing.last_stamp_by_in[in_topic] = pkt.stamp_ns;
          feed_packet(
            it->second, pkt,
            [&](nebula::drivers::NebulaPointCloudPtr cloud, std::int64_t stamp_ns) {
              sink(in_topic, cloud, stamp_ns);
            });
        }
        continue;
      }

      // Passthrough: write the serialized message verbatim under its
      // original topic name. Covers unmatched packet topics, demoted
      // packet topics (Robosense / Continental radar / unidentified),
      // and any unrelated streams (TF, IMU, camera, ...).
      if (passthrough.by_name.count(bag_msg->topic_name)) {
        writer.write(bag_msg);
      }
    }

    // Flush trailing scans held by mechanical-LiDAR decoders (see
    // single-topic convert() history for rationale).
    for (auto & [in_topic, state] : states) {
      auto trailing = state.decoder.flush();
      if (!trailing || !*trailing) {
        continue;
      }
      ++state.clouds_produced;
      sink(in_topic, *trailing, routing.last_stamp_by_in[in_topic]);
    }
  }  // writer scope ends here -- destructor flushes and closes files.

  // Phase 4: collapse scratch -> bare file for bare-file inputs, then
  // assemble and return the result.
  if (collapse_to_file) {
    collapse_scratch_to_bare_file(writer_uri, final_output_path, in_spec.storage_id);
  }
  return build_convert_result(support.decoded, states, std::move(passthrough.ordered));
}

}  // namespace nebuladec::bag
