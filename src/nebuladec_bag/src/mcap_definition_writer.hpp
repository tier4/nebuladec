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

#ifndef MCAP_DEFINITION_WRITER_HPP_
#define MCAP_DEFINITION_WRITER_HPP_

#include "nebuladec_bag/message_definition.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

// Forward-declare libmcap types so callers do not pay the cost of
// pulling in the full mcap/writer.hpp through this header.
namespace mcap
{
class McapWriter;
}  // namespace mcap

namespace nebuladec::bag
{

/// @brief Bare-file MCAP writer that injects schemas from a definition
/// registry, falling back to the local ament index for installed types.
///
/// Designed as a drop-in replacement for `rosbag2_cpp::Writer` on the
/// MCAP output path **only when** the input bag carries embedded
/// definitions for types that are not installed in the current ROS 2
/// environment. The class encapsulates one `mcap::McapWriter`, the
/// schema-id / channel-id bookkeeping, and the lookup policy:
///
///   1. exact `type_name` match in the supplied `registry`
///   2. fallback to `load_message_definition_from_ament(type_name)`
///   3. soft-fail: emit a Schema record with empty `data` and `encoding`
///      (matches `rosbag2_storage_mcap`'s behaviour for missing types)
///
/// The writer is non-copyable and non-movable; it owns a `FILE*`-backed
/// libmcap writer that must stay anchored to the output path for the
/// duration of the convert run.
class McapDefinitionWriter
{
public:
  /// Open `output_path` for writing. Throws `std::runtime_error` when
  /// the file cannot be created or libmcap rejects the path.
  McapDefinitionWriter(
    const std::filesystem::path & output_path, const MessageDefinitionRegistry & registry);
  ~McapDefinitionWriter();

  McapDefinitionWriter(const McapDefinitionWriter &) = delete;
  McapDefinitionWriter & operator=(const McapDefinitionWriter &) = delete;
  McapDefinitionWriter(McapDefinitionWriter &&) = delete;
  McapDefinitionWriter & operator=(McapDefinitionWriter &&) = delete;

  /// Register a topic that will receive `write_serialized` calls. May be
  /// called multiple times for the same `topic` -- subsequent calls are
  /// no-ops (matches `rosbag2_cpp::Writer::create_topic` semantics).
  /// `serialization_format` is forwarded as the MCAP `messageEncoding`
  /// tag (always `"cdr"` for ROS 2 in practice).
  void create_topic(
    std::string_view topic, std::string_view type, std::string_view serialization_format);

  /// Write one serialized message. The topic must have been registered
  /// via `create_topic` first; calling write on an unknown topic throws.
  /// `log_time_ns` and `publish_time_ns` are nanosecond timestamps;
  /// they may be identical for rosbag-recorded traffic.
  void write_serialized(
    std::string_view topic, const std::byte * data, std::size_t size, std::int64_t log_time_ns,
    std::int64_t publish_time_ns);

  /// Finalize the MCAP file. Idempotent. Always called by the
  /// destructor; expose it so callers can sequence flush errors before
  /// destruction.
  void close();

  /// Return true when `close()` has not yet finalized the file. Useful
  /// from tests.
  [[nodiscard]] bool is_open() const noexcept;

private:
  struct ChannelEntry
  {
    std::uint16_t channel_id;
    std::uint16_t schema_id;  // 0 == no schema
    std::string type_name;
  };

  /// Allocate (or reuse) a Schema record for `type_name`. Returns 0
  /// when no definition is available and the channel must reference no
  /// schema -- this is the soft-fail path.
  std::uint16_t intern_schema(std::string_view type_name);

  std::filesystem::path output_path_;
  const MessageDefinitionRegistry & registry_;  // borrowed; lifetime managed by caller
  std::unique_ptr<mcap::McapWriter> writer_;
  bool open_{false};
  std::unordered_map<std::string, std::uint16_t> schema_id_by_type_;
  std::unordered_map<std::string, ChannelEntry> channel_by_topic_;
};

}  // namespace nebuladec::bag

#endif  // MCAP_DEFINITION_WRITER_HPP_
