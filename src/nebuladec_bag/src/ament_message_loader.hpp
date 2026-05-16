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

#ifndef AMENT_MESSAGE_LOADER_HPP_
#define AMENT_MESSAGE_LOADER_HPP_

#include "nebuladec_bag/message_definition.hpp"

#include <optional>
#include <string_view>

namespace nebuladec::bag
{

/// Resolve a ROS 2 message type ("pkg/msg/Type" or "pkg/Type") through
/// the local ament index and return a fully concatenated `ros2msg`
/// schema text suitable for embedding into an MCAP Schema record.
///
/// Mirrors the algorithm used by `rosbag2_storage_mcap::internal::
/// MessageDefinitionCache::get_full_text()`: the primary `.msg` file is
/// read first, then every non-primitive dependency it references is
/// resolved recursively, with the secondary definitions appended after
/// an `=` separator line and an `MSG: pkg/Type` header line, exactly as
/// specified at <https://mcap.dev/specification/appendix.html
/// #ros2msg-data-format>.
///
/// Returns `nullopt` when any required `.msg` file cannot be located --
/// the caller is expected to fall back to the embedded-bag definition
/// registry or to a soft-fail empty schema in that case.
[[nodiscard]] std::optional<MessageDefinition> load_message_definition_from_ament(
  std::string_view type_name);

}  // namespace nebuladec::bag

#endif  // AMENT_MESSAGE_LOADER_HPP_
