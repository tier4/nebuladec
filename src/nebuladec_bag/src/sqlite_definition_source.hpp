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

#ifndef SQLITE_DEFINITION_SOURCE_HPP_
#define SQLITE_DEFINITION_SOURCE_HPP_

#include "nebuladec_bag/bag_io.hpp"
#include "nebuladec_bag/message_definition.hpp"

#include <memory>

namespace nebuladec::bag
{

/// Build a `MessageDefinitionSource` for a rosbag2 sqlite3 input.
///
/// On distros that store message definitions in `metadata.yaml`
/// (Iron and later, via the per-topic `message_definition` key) this
/// source returns one `MessageDefinition` per topic. On Humble the key
/// is absent and `load()` returns an empty vector -- conversion falls
/// back to `rosbag2_storage_mcap`'s existing "definition file(s)
/// missing" warning path, which is the soft-fail behaviour the user
/// signed off on.
[[nodiscard]] std::unique_ptr<MessageDefinitionSource> make_sqlite_definition_source(
  const InputSpec & spec);

}  // namespace nebuladec::bag

#endif  // SQLITE_DEFINITION_SOURCE_HPP_
