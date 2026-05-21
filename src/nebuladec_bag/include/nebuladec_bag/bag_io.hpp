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

#ifndef NEBULADEC_BAG__BAG_IO_HPP_
#define NEBULADEC_BAG__BAG_IO_HPP_

#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/topic_mapping.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace nebuladec::bag
{

/// @brief Describes how a rosbag2 input should be opened.
///
/// The storage plugin (`"mcap"` / `"sqlite3"`) and whether the input is a
/// single file or a directory containing metadata.yaml are derived from
/// the path itself so callers can be oblivious.
struct InputSpec
{
  std::string uri;         ///< value fed to rosbag2 StorageOptions
  std::string storage_id;  ///< "mcap" or "sqlite3"
  bool is_directory{false};
};

/// Detect an InputSpec from a path. Throws std::invalid_argument when the
/// path does not look like a rosbag2 recording (no matching file, no
/// metadata.yaml, unknown storage extension, ...).
InputSpec detect_input(const std::string & path);

/// @brief Per-topic result of an `inspect()` run.
///
/// `inspect()` only reads the first packet message per topic, so callers
/// get vendor + model quickly without walking the whole bag. Vendor is
/// always derived from the sniffed model, never from the ROS 2 message
/// type -- `nebula_msgs/NebulaPackets` is shared by Seyond and
/// Continental, so the type alone does not pin down a vendor. Topics
/// with zero ROS messages in the bag are reported with `message_count=0`
/// and `identity` unset; `inspect()`'s own CLI filters those out, but
/// `plan_convert()` surfaces them so the dry-run table can explain
/// exactly why a topic will be skipped.
struct TopicInspectResult
{
  std::string topic;
  /// Vendor + model resolved by feeding the first packet through the
  /// sniffer. Empty when the packet bytes could not be identified by the
  /// sniffer or when the topic has zero messages.
  std::optional<Identity> identity;
  /// Whether the topic had at least one message in the bag. False means
  /// no sniffing was attempted; `identity` will be empty.
  bool has_messages{false};
  /// Number of ROS 2 messages on this topic, taken from bag metadata
  /// (no full scan required). 0 when the topic is declared but silent.
  std::size_t message_count{0};
};

/// @brief Summary of an `inspect()` run across all packet topics.
struct InspectSummary
{
  std::vector<TopicInspectResult> topics;
};

/// Open `input_path`, auto-discover every Nebula packet topic (LiDAR and
/// Continental radar alike), feed each topic through its own sniffer /
/// decoder pipeline, and return one summary entry per topic. Radar
/// topics and Robosense LiDAR topics are identified but not decoded.
InspectSummary inspect(const std::string & input_path);

/// @brief Aggregated decode-progress snapshot.
///
/// Reports cumulative reader progress across every decoded topic in the
/// bag (passthrough traffic is not counted). `messages_total` is the sum
/// of bag-metadata message counts for the decoded topics; it is fixed
/// for the duration of a `convert()` call. `messages_done` only ever
/// increases.
///
/// Emitted from the reader stage, not the worker pool, so the figure
/// tracks bytes pulled from the input rather than decoded clouds
/// produced -- it leads worker throughput slightly but is the right
/// signal for a "how much of the bag is left" UI.
struct ProgressEvent
{
  std::size_t messages_done{0};
  std::size_t messages_total{0};
};

/// @brief Output-side MCAP writer tuning.
///
/// The output storage plugin is driven by the output path's extension
/// (`.mcap` -> mcap, `.db3` -> sqlite3) for bare-file outputs, so these
/// options take effect whenever the output bag is MCAP -- including
/// cross-format conversions like `.db3 -> .mcap`. When the output is
/// not MCAP (sqlite3 output, or a directory output that mirrors a
/// sqlite3 input), `convert()` emits a single warning and ignores the
/// options -- this keeps the CLI's MCAP flags harmless against a mixed
/// workflow.
///
/// `kAuto` means "do not override; let the writer use its own
/// default". The library never enforces a specific value -- defaults
/// match `rosbag2_storage_mcap` / libmcap out-of-the-box behaviour.
enum class McapCompression : std::uint8_t { kAuto, kNone, kLz4, kZstd };
enum class McapCompressionLevel : std::uint8_t {
  kAuto,
  kFastest,
  kFast,
  kDefault,
  kSlow,
  kSlowest,
};

struct McapWriteOptions
{
  McapCompression compression{McapCompression::kAuto};
  McapCompressionLevel compression_level{McapCompressionLevel::kAuto};
  /// 0 means "let the writer pick" (rosbag2_storage_mcap default ~=
  /// libmcap's 768 KiB). Larger chunks (e.g. 4-64 MiB) reduce the
  /// number of compression invocations the writer thread has to
  /// perform and typically improve write throughput on writer-bound
  /// workloads at the cost of a small memory footprint.
  std::uint64_t chunk_size_bytes{0};
};

/// @brief Options accepted by `convert()`.
///
/// The set of (in_topic -> out_topic, frame_id) pairs is driven entirely
/// by `mapping`; no CLI overrides exist for individual topics. Packet
/// topics in the bag that match no rule are skipped and logged by the
/// caller.
///
/// `sequential` and `workers` control the execution model:
///   * `sequential = true` forces the legacy single-threaded path.
///   * `sequential = false` (default) selects the 3-stage pipeline
///     (Reader → Worker pool → Writer). The pipeline auto-falls-back
///     to the sequential path when `hardware_concurrency() < 3` or
///     when the bag has no decodable LiDAR topics.
///   * `workers = 0` (default) means "auto": pick
///     `min(hardware_concurrency, K)` where K is the count of decoded
///     packet topics. Any positive value is clamped to `min(N, K)` and,
///     when the result is `< K`, snapped down to the largest divisor of
///     K so each worker can be assigned an equal share of topics.
///   * `workers` is ignored when `sequential = true`.
///
/// `on_progress`, when set, is invoked periodically from the reader
/// stage with the latest cumulative count. It is called at most every
/// ~50 ms while reading, plus exactly once at end-of-input with the
/// final count. Callbacks are serialised behind an internal lock and
/// thrown exceptions are swallowed, so a UI bug cannot corrupt the
/// pipeline. Leave empty (default) to disable; the per-message branch
/// is a single nullptr check when disabled.
struct ConvertOptions
{
  std::string input_path;
  std::string output_path;
  TopicMapping mapping;
  bool sequential{false};
  std::size_t workers{0};
  std::function<void(const ProgressEvent &)> on_progress{};
  /// Output-side MCAP writer tuning. Honoured only when the output
  /// bag is MCAP (driven by the output path's extension for bare-file
  /// outputs); ignored with a single warning on sqlite3 output.
  McapWriteOptions mcap{};
};

/// @brief Per-in-topic conversion statistics produced by `convert()`.
struct TopicConvertResult
{
  std::string in_topic;
  std::string out_topic;
  std::string frame_id;
  std::optional<Identity> identity;
  /// Number of ROS 2 packet messages (e.g. PandarScan, VelodyneScan,
  /// nebula_msgs::NebulaPackets) read from the input topic. NOT the
  /// number of individual sensor packets inside those messages.
  std::size_t packets{0};
  std::size_t clouds_written{0};
};

/// @brief Aggregate result for a `convert()` run.
struct ConvertResult
{
  /// Topics that were decoded end-to-end into PointCloud2.
  std::vector<TopicConvertResult> topics;
  /// Topics that were copied verbatim from the input bag to the output
  /// bag. This includes every topic that was NOT in `topics`: packet
  /// topics that matched no rule, packet topics whose vendor/model is
  /// not supported, and any unrelated streams (TF, IMU, camera, ...).
  /// Exposed so the CLI can report how much data was preserved alongside
  /// the decoded output.
  std::vector<std::string> passthrough_topics;
};

/// @brief One row of a convert dry-run plan.
///
/// `status` values:
///   * "ok"      -- the in_topic matched exactly one rule and would be
///                  converted; `out_topic` and `frame_id` are filled in.
///   * "skipped" -- the in_topic matched no rule; other fields are empty.
///   * "error"   -- the in_topic matched multiple rules; `message`
///                  carries the diagnostic and other fields are empty.
struct ConvertPlanEntry
{
  std::string in_topic;
  std::string out_topic;
  std::string frame_id;
  std::optional<Identity> identity;
  /// Number of ROS 2 packet messages on this input topic, from bag
  /// metadata. Mirrors `TopicConvertResult::packets` so the dry-run
  /// table can show what convert() would consume.
  std::size_t packets{0};
  std::string status;
  std::string message;
};

/// Resolve every packet topic discovered in `input_path` against `mapping`
/// without reading beyond what `inspect()` needs. Returns one entry per
/// packet topic present in the bag, preserving metadata order.
std::vector<ConvertPlanEntry> plan_convert(
  const std::string & input_path, const TopicMapping & mapping);

/// Read `options.input_path`, decode every packet-topic that matches
/// `options.mapping` AND whose vendor/model is supported, and write the
/// resolved PointCloud2 topics to `options.output_path`. Every other
/// topic in the input bag (unmatched packet topics, unsupported-vendor
/// packet topics, TF, IMU, camera streams, ...) is preserved verbatim in
/// the output bag.
///
/// The output bag's file-vs-directory **layout** mirrors the input
/// (bare-file in -> bare-file out, directory in -> directory out).
/// The output **storage plugin** is driven by the output path's
/// extension for bare-file outputs (`.mcap` -> mcap, `.db3` -> sqlite3),
/// enabling cross-format conversion (`.db3 <-> .mcap`); same-format
/// outputs (`.db3 -> .db3`, `.mcap -> .mcap`) are byte-equivalent to a
/// plugin-mirror. Bare-file outputs with an unrecognised or absent
/// extension fall back to mirroring the input plugin, and directory
/// outputs always mirror the input plugin.
///
/// Throws `std::runtime_error` when a single in-topic matches multiple
/// rules (ambiguous config).
ConvertResult convert(const ConvertOptions & options);

}  // namespace nebuladec::bag

#endif  // NEBULADEC_BAG__BAG_IO_HPP_
