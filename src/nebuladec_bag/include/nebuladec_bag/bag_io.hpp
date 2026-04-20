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
struct TopicInspectResult
{
  std::string topic;
  std::string message_type;
  /// Vendor derivable from the ROS 2 message type alone. `UNKNOWN` for
  /// `nebula_msgs/NebulaPackets`, which needs packet-level sniffing.
  Vendor vendor_by_message_type{Vendor::UNKNOWN};
  /// Vendor + model resolved by feeding packets through the sniffer.
  /// Empty when no packet could be identified (e.g. an empty topic).
  std::optional<Identity> identity;
  std::size_t data_packets{0};
  std::size_t info_packets{0};
  std::size_t clouds_produced{0};
  /// Robosense DIFOP topic paired with this packet topic. Only populated
  /// when the packet vendor is Robosense and the bag has exactly one
  /// info topic; with zero or multiple info topics pairing is ambiguous
  /// and this stays empty. Topic names are never used to guess sensor
  /// model -- model comes strictly from message type and packet bytes.
  std::string info_topic;
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
struct ConvertOptions
{
  std::string input_path;
  std::string output_path;
  std::string output_topic{"/nebuladec/pointcloud"};
  std::string frame_id{"lidar"};
  /// When unset, the packet topic is auto-discovered from the bag.
  std::optional<std::string> packets_topic;
  /// When unset, Robosense info topic is auto-discovered.
  std::optional<std::string> info_topic;
};

struct ConvertResult
{
  std::optional<Identity> identity;
  std::size_t data_packets{0};
  std::size_t info_packets{0};
  std::size_t clouds_written{0};
  std::string packets_topic;
  std::string info_topic;
};

/// Read `options.input_path`, decode every packet, and write a sibling
/// PointCloud2 topic to `options.output_path`. The output bag uses the
/// same storage plugin and file/directory layout as the input.
ConvertResult convert(const ConvertOptions & options);

}  // namespace nebuladec::bag

#endif  // NEBULADEC_BAG__BAG_IO_HPP_
