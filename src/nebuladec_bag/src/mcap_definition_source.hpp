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

#ifndef MCAP_DEFINITION_SOURCE_HPP_
#define MCAP_DEFINITION_SOURCE_HPP_

#include "nebuladec_bag/bag_io.hpp"
#include "nebuladec_bag/message_definition.hpp"

#include <memory>

namespace nebuladec::bag
{

/// Build a `MessageDefinitionSource` that pulls Schema records from an
/// MCAP input. Resolves the on-disk MCAP file from `spec`: when
/// `is_directory` is true the source scans the rosbag2 layout for the
/// single `.mcap` payload alongside `metadata.yaml`; otherwise `spec.uri`
/// is the file itself. Throws `std::runtime_error` from `load()` if the
/// file cannot be opened or the MCAP summary section cannot be parsed.
[[nodiscard]] std::unique_ptr<MessageDefinitionSource> make_mcap_definition_source(
  const InputSpec & spec);

}  // namespace nebuladec::bag

#endif  // MCAP_DEFINITION_SOURCE_HPP_
