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
/// `inspect()` only reads the first packet message per topic (plus the
/// first unique info-topic message, for Robosense DIFOP), so callers get
/// vendor + model quickly without walking the whole bag. Vendor is
/// always derived from the sniffed model, never from the ROS 2 message
/// type -- `nebula_msgs/NebulaPackets` is shared by Seyond and
/// Continental, so the type alone does not pin down a vendor. Topics
/// with zero ROS messages in the bag are omitted from the summary --
/// they carry no information inspect() can report.
struct TopicInspectResult
{
  std::string topic;
  /// Vendor + model resolved by feeding the first packet (and DIFOP, if
  /// present) through the sniffer. Empty when the packet bytes could not
  /// be identified by the sniffer.
  std::optional<Identity> identity;
};

/// @brief Summary of an `inspect()` run across all packet topics.
struct InspectSummary
{
  std::vector<TopicInspectResult> topics;
};

/// Open `input_path`, auto-discover every Nebula packet topic (LiDAR and
/// Continental radar alike), feed each topic through its own sniffer /
/// decoder pipeline, and return one summary entry per topic. Radar
/// topics are identified but not decoded.
InspectSummary inspect(const std::string & input_path);

/// @brief Options accepted by `convert()`.
///
/// The set of (in_topic -> out_topic, frame_id, info_topic) pairs is
/// driven entirely by `mapping`; no CLI overrides exist for individual
/// topics. Packet topics in the bag that match no rule are skipped and
/// logged by the caller.
struct ConvertOptions
{
  std::string input_path;
  std::string output_path;
  TopicMapping mapping;
};

/// @brief Per-in-topic conversion statistics produced by `convert()`.
struct TopicConvertResult
{
  std::string in_topic;
  std::string out_topic;
  std::string frame_id;
  /// Empty when the matched rule had no `info_topic`.
  std::string info_topic;
  std::optional<Identity> identity;
  std::size_t data_packets{0};
  std::size_t info_packets{0};
  std::size_t clouds_written{0};
};

/// @brief Aggregate result for a `convert()` run.
struct ConvertResult
{
  std::vector<TopicConvertResult> topics;
  /// Packet topics present in the bag that matched no mapping rule. Kept
  /// so CLI code can warn the user about likely config gaps.
  std::vector<std::string> skipped_topics;
};

/// @brief One row of a convert dry-run plan.
///
/// `status` values:
///   * "ok"      -- the in_topic matched exactly one rule and would be
///                  converted; `out_topic`, `frame_id`, and optional
///                  `info_topic` are filled in.
///   * "skipped" -- the in_topic matched no rule; other fields are empty.
///   * "error"   -- the in_topic matched multiple rules; `message`
///                  carries the diagnostic and other fields are empty.
struct ConvertPlanEntry
{
  std::string in_topic;
  std::string out_topic;
  std::string frame_id;
  std::string info_topic;
  std::optional<Identity> identity;
  std::string status;
  std::string message;
};

/// Resolve every packet topic discovered in `input_path` against `mapping`
/// without reading beyond what `inspect()` needs. Returns one entry per
/// packet topic present in the bag, preserving metadata order.
std::vector<ConvertPlanEntry> plan_convert(
  const std::string & input_path, const TopicMapping & mapping);

/// Read `options.input_path`, decode every packet-topic that matches
/// `options.mapping`, and write the resolved PointCloud2 topics to
/// `options.output_path`. The output bag uses the same storage plugin
/// and file/directory layout as the input.
///
/// Packet topics that match no rule are skipped (and reported back via
/// `ConvertResult::skipped_topics`). Throws `std::runtime_error` when a
/// single in-topic matches multiple rules (ambiguous config).
ConvertResult convert(const ConvertOptions & options);

}  // namespace nebuladec::bag

#endif  // NEBULADEC_BAG__BAG_IO_HPP_
