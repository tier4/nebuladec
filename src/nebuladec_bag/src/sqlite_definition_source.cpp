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

#include "sqlite_definition_source.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebuladec::bag
{

namespace
{

namespace fs = std::filesystem;

/// Resolve the `metadata.yaml` path for a sqlite3 rosbag2 input.
/// Returns an empty path if no metadata.yaml exists -- bare-file `.db3`
/// recordings without a sibling metadata.yaml are common (rosbag2 ros2
/// bag record always writes one, but third-party producers and certain
/// test fixtures do not), and the user wants those to soft-fail rather
/// than abort.
[[nodiscard]] fs::path resolve_metadata_yaml(const InputSpec & spec)
{
  if (spec.is_directory) {
    const auto candidate = fs::path{spec.uri} / "metadata.yaml";
    return fs::exists(candidate) ? candidate : fs::path{};
  }
  const auto candidate = fs::path{spec.uri}.parent_path() / "metadata.yaml";
  return fs::exists(candidate) ? candidate : fs::path{};
}

/// Source backed by rosbag2's `metadata.yaml`.
///
/// On Iron+ each entry under
///   rosbag2_bagfile_information.topics_with_message_count[*].topic_metadata
/// can carry a `message_definition` string. We read that when present;
/// the lookup is fault-tolerant by design so older bags (Humble) yield
/// an empty vector rather than an exception.
class SqliteMessageDefinitionSource final : public MessageDefinitionSource
{
public:
  explicit SqliteMessageDefinitionSource(fs::path metadata_path)
  : metadata_path_(std::move(metadata_path))
  {
  }

  std::vector<MessageDefinition> load() override
  {
    if (metadata_path_.empty()) {
      return {};
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(metadata_path_.string());
    } catch (const YAML::Exception &) {
      // metadata.yaml is optional context for this source. A parse
      // error degrades to the same outcome as a missing file: empty
      // definitions, soft-fail downstream.
      return {};
    }
    const auto info = root["rosbag2_bagfile_information"];
    if (!info) {
      return {};
    }
    const auto topics = info["topics_with_message_count"];
    if (!topics || !topics.IsSequence()) {
      return {};
    }
    std::vector<MessageDefinition> out;
    out.reserve(topics.size());
    for (const auto & entry : topics) {
      const auto topic_meta = entry["topic_metadata"];
      if (!topic_meta) {
        continue;
      }
      const auto type_node = topic_meta["type"];
      const auto def_node = topic_meta["message_definition"];
      if (!type_node || !def_node) {
        continue;
      }
      auto text = def_node.as<std::string>(std::string{});
      if (text.empty()) {
        continue;
      }
      MessageDefinition def;
      def.type_name = type_node.as<std::string>(std::string{});
      if (def.type_name.empty()) {
        continue;
      }
      // metadata.yaml does not declare a schema encoding; rosbag2 only
      // serialises the .msg text. Match `rosbag2_storage_mcap`'s
      // implicit default of "ros2msg" so downstream consumers see the
      // same tag as for a freshly recorded mcap.
      def.encoding = "ros2msg";
      def.text = std::move(text);
      out.push_back(std::move(def));
    }
    return out;
  }

private:
  fs::path metadata_path_;
};

}  // namespace

std::unique_ptr<MessageDefinitionSource> make_sqlite_definition_source(const InputSpec & spec)
{
  return std::make_unique<SqliteMessageDefinitionSource>(resolve_metadata_yaml(spec));
}

}  // namespace nebuladec::bag
