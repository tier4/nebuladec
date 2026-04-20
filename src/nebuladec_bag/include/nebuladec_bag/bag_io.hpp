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

/// @brief Summary of an `inspect()` run.
struct InspectSummary
{
  std::optional<Identity> identity;
  std::size_t data_packets{0};
  std::size_t info_packets{0};
  std::size_t clouds_produced{0};
  std::string packets_topic;
  std::string info_topic;
};

/// Open `input_path`, auto-discover the Nebula packet topic (and DIFOP
/// topic when present), feed every packet through a Decoder, and return
/// a summary. No outputs are written.
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
