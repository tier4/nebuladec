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

#include "nebuladec_bag/message_definition.hpp"

#include "mcap_definition_source.hpp"
#include "sqlite_definition_source.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebuladec::bag
{

namespace
{

/// Null source: always returns an empty vector. Used as the factory
/// fallback for unknown storage IDs so callers can stay branch-free.
class EmptyMessageDefinitionSource final : public MessageDefinitionSource
{
public:
  std::vector<MessageDefinition> load() override { return {}; }
};

}  // namespace

void MessageDefinitionRegistry::add(MessageDefinition definition)
{
  // First-insert-wins: the underlying bag stores at most one record per
  // type so a duplicate insertion only happens when two layers (mcap
  // Schemas + metadata.yaml) carry the same type. Preserving the first
  // entry matches the bag's own record ordering.
  by_type_.emplace(definition.type_name, std::move(definition));
}

std::optional<MessageDefinition> MessageDefinitionRegistry::find(std::string_view type_name) const
{
  // unordered_map heterogeneous lookup is not available until C++20's
  // transparent hash; construct a temporary key once per call. The
  // registry holds O(10) entries in practice so this is negligible.
  const auto it = by_type_.find(std::string{type_name});
  if (it == by_type_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool MessageDefinitionRegistry::contains(std::string_view type_name) const
{
  return by_type_.find(std::string{type_name}) != by_type_.end();
}

std::unique_ptr<MessageDefinitionSource> make_definition_source(const InputSpec & spec)
{
  if (spec.storage_id == "mcap") {
    return make_mcap_definition_source(spec);
  }
  if (spec.storage_id == "sqlite3") {
    return make_sqlite_definition_source(spec);
  }
  return std::make_unique<EmptyMessageDefinitionSource>();
}

MessageDefinitionRegistry load_definition_registry(const InputSpec & spec)
{
  auto source = make_definition_source(spec);
  MessageDefinitionRegistry registry;
  for (auto & def : source->load()) {
    registry.add(std::move(def));
  }
  return registry;
}

}  // namespace nebuladec::bag
