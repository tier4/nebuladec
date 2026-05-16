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

#include "mcap_definition_writer.hpp"

#include "ament_message_loader.hpp"

#include <mcap/writer.hpp>

#include <rcutils/logging_macros.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace nebuladec::bag
{

namespace
{

constexpr const char * g_log_name = "nebuladec_bag";

}  // namespace

McapDefinitionWriter::McapDefinitionWriter(
  const std::filesystem::path & output_path, const MessageDefinitionRegistry & registry)
: output_path_(output_path), registry_(registry), writer_(std::make_unique<mcap::McapWriter>())
{
  mcap::McapWriterOptions opts{"ros2"};
  // Defaults chosen to match `rosbag2_storage_mcap`'s out-of-the-box
  // behaviour: zstd-compressed chunks, summary section enabled, indexed
  // messages. Downstream tools (ros2 bag info, foxglove studio) only
  // open files that carry these by default.
  opts.compression = mcap::Compression::Zstd;
  opts.compressionLevel = mcap::CompressionLevel::Default;
  if (const auto status = writer_->open(output_path.string(), opts); !status.ok()) {
    throw std::runtime_error(
      "failed to open mcap writer for " + output_path.string() + ": " + status.message);
  }
  open_ = true;
}

McapDefinitionWriter::~McapDefinitionWriter()
{
  close();
}

bool McapDefinitionWriter::is_open() const noexcept
{
  return open_;
}

void McapDefinitionWriter::close()
{
  if (!open_) {
    return;
  }
  writer_->close();
  open_ = false;
}

std::uint16_t McapDefinitionWriter::intern_schema(std::string_view type_name)
{
  const std::string key{type_name};
  if (const auto it = schema_id_by_type_.find(key); it != schema_id_by_type_.end()) {
    return it->second;
  }

  // Lookup priority:
  //   1. Registry built from the input bag (covers types not installed
  //      locally -- the headline feature).
  //   2. ament index fallback (covers locally installed types that the
  //      bag does not embed, e.g. PointCloud2 produced by the decoder).
  //   3. Soft fail: emit a Schema record with empty data and no
  //      encoding, matching the warning-path behaviour the user signed
  //      off on.
  std::optional<MessageDefinition> def = registry_.find(type_name);
  if (!def) {
    def = load_message_definition_from_ament(type_name);
  }
  if (!def) {
    RCUTILS_LOG_WARN_NAMED(
      g_log_name,
      "no message definition available for %s (not embedded in the bag and not installed "
      "locally); writing schema record with empty body",
      key.c_str());
  }

  mcap::Schema schema{
    type_name, def ? std::string_view{def->encoding} : std::string_view{},
    def ? std::string_view{def->text} : std::string_view{}};
  writer_->addSchema(schema);  // libmcap fills in schema.id
  schema_id_by_type_.emplace(key, schema.id);
  return schema.id;
}

void McapDefinitionWriter::create_topic(
  std::string_view topic, std::string_view type, std::string_view serialization_format)
{
  if (!open_) {
    throw std::runtime_error(
      "create_topic called on closed McapDefinitionWriter for " + output_path_.string());
  }
  const std::string topic_key{topic};
  if (channel_by_topic_.count(topic_key) > 0) {
    return;
  }
  const auto schema_id = intern_schema(type);
  // `messageEncoding` is the mcap-level tag; rosbag2 always uses "cdr".
  // We forward the rosbag2 `serialization_format` field verbatim so an
  // input bag using a non-default format would round-trip correctly.
  const std::string encoding =
    serialization_format.empty() ? std::string{"cdr"} : std::string{serialization_format};
  mcap::Channel channel{topic, encoding, schema_id};
  writer_->addChannel(channel);  // libmcap fills in channel.id
  channel_by_topic_.emplace(topic_key, ChannelEntry{channel.id, schema_id, std::string{type}});
}

void McapDefinitionWriter::write_serialized(
  std::string_view topic, const std::byte * data, std::size_t size, std::int64_t log_time_ns,
  std::int64_t publish_time_ns)
{
  if (!open_) {
    throw std::runtime_error(
      "write_serialized called on closed McapDefinitionWriter for " + output_path_.string());
  }
  const auto it = channel_by_topic_.find(std::string{topic});
  if (it == channel_by_topic_.end()) {
    throw std::runtime_error(
      "write_serialized called for unregistered topic '" + std::string{topic} + "'");
  }
  mcap::Message msg{};
  msg.channelId = it->second.channel_id;
  msg.sequence = 0;
  msg.logTime = static_cast<mcap::Timestamp>(log_time_ns);
  msg.publishTime = static_cast<mcap::Timestamp>(publish_time_ns);
  msg.dataSize = size;
  msg.data = data;
  if (const auto status = writer_->write(msg); !status.ok()) {
    throw std::runtime_error(
      std::string{"mcap write failed: "} + status.message + " (topic=" + std::string{topic} + ")");
  }
}

}  // namespace nebuladec::bag
