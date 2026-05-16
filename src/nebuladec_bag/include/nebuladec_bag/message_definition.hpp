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

#ifndef NEBULADEC_BAG__MESSAGE_DEFINITION_HPP_
#define NEBULADEC_BAG__MESSAGE_DEFINITION_HPP_

#include "nebuladec_bag/bag_io.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace nebuladec::bag
{

/// @brief One ROS 2 message-type definition as it is embedded inside a
/// rosbag2 recording.
///
/// `text` is the raw schema payload exactly as the bag stored it (for MCAP
/// this is the `Schema.data` byte range; for rosbag2 metadata.yaml it is
/// the `message_definition` string introduced in Iron+). Callers must not
/// try to parse or normalise it -- the whole point of forwarding the
/// embedded definition is to round-trip it byte-for-byte so consumers see
/// the same schema the producer wrote.
///
/// `encoding` is the MCAP-style schema-encoding tag (`"ros2msg"` or
/// `"ros2idl"`). When the source bag does not carry an explicit encoding
/// it defaults to `"ros2msg"`, matching `rosbag2_storage_mcap`'s own
/// behaviour.
struct MessageDefinition
{
  std::string type_name;  ///< e.g. "oxts_msgs/msg/Ncom"
  std::string encoding;   ///< "ros2msg" or "ros2idl"
  std::string text;       ///< raw schema bytes from the bag
};

/// @brief Abstract source of embedded message-type definitions.
///
/// One concrete implementation per rosbag2 storage plugin. Implementations
/// must be cheap to construct; `load()` performs the actual I/O and is the
/// only method that may throw.
class MessageDefinitionSource
{
public:
  virtual ~MessageDefinitionSource() = default;
  MessageDefinitionSource(const MessageDefinitionSource &) = delete;
  MessageDefinitionSource & operator=(const MessageDefinitionSource &) = delete;
  MessageDefinitionSource(MessageDefinitionSource &&) = delete;
  MessageDefinitionSource & operator=(MessageDefinitionSource &&) = delete;

  /// Read every embedded definition from the underlying bag and return
  /// them in storage order. Throws `std::runtime_error` on unrecoverable
  /// I/O errors. A successful call with an empty return vector means the
  /// bag exists but carries no embedded definitions (e.g. a rosbag2
  /// Humble `metadata.yaml` whose schema predates `message_definitions:`).
  [[nodiscard]] virtual std::vector<MessageDefinition> load() = 0;

protected:
  MessageDefinitionSource() = default;
};

/// Factory dispatching on `spec.storage_id`. Returns a non-null pointer
/// even for storage backends that cannot carry embedded definitions -- the
/// returned source's `load()` will simply yield an empty vector. Callers
/// therefore never need to special-case storage IDs.
[[nodiscard]] std::unique_ptr<MessageDefinitionSource> make_definition_source(
  const InputSpec & spec);

/// @brief Type-name keyed view over a `MessageDefinitionSource::load()`
/// result.
///
/// Cheap to construct and to copy by reference; lookups are O(1). Used by
/// the writer path to decide whether a passthrough topic's schema can be
/// forwarded from the input bag.
class MessageDefinitionRegistry
{
public:
  MessageDefinitionRegistry() = default;

  /// Insert one definition. The first insertion for a given `type_name`
  /// wins -- duplicate names with conflicting payloads are silently
  /// ignored. rosbag2 always writes one Schema record per type, so the
  /// duplicate case only arises when two storage layers (e.g. mcap
  /// Schemas + metadata.yaml) agree on the same type but differ in
  /// formatting; preserving the first matches the bag's own ordering.
  void add(MessageDefinition definition);

  /// O(1) lookup by ROS 2 type string ("pkg/msg/Name").
  [[nodiscard]] std::optional<MessageDefinition> find(std::string_view type_name) const;

  /// True iff `find(type_name)` would return an entry.
  [[nodiscard]] bool contains(std::string_view type_name) const;

  /// Total number of stored definitions.
  [[nodiscard]] std::size_t size() const noexcept { return by_type_.size(); }

  /// True when no definitions have been added.
  [[nodiscard]] bool empty() const noexcept { return by_type_.empty(); }

private:
  std::unordered_map<std::string, MessageDefinition> by_type_;
};

/// Convenience: build a registry by loading from a freshly constructed
/// source. Equivalent to:
///   `MessageDefinitionRegistry r; for (auto & d : src->load()) r.add(d);`
/// but routes any source-construction failure into the same exception
/// path as `load()` so callers only need one try/catch.
[[nodiscard]] MessageDefinitionRegistry load_definition_registry(const InputSpec & spec);

}  // namespace nebuladec::bag

#endif  // NEBULADEC_BAG__MESSAGE_DEFINITION_HPP_
