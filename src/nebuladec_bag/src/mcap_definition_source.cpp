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

#include "mcap_definition_source.hpp"

// libmcap is shipped pre-built by mcap_vendor (libmcap.so), so we must
// NOT define MCAP_IMPLEMENTATION here -- doing so re-emits the
// implementation symbols and causes ODR violations / link errors.
#include <mcap/reader.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebuladec::bag
{

namespace
{

namespace fs = std::filesystem;

/// Locate the single `.mcap` payload inside a rosbag2 MCAP directory
/// layout. rosbag2's mcap plugin always writes one or more files named
/// `<bagdir-name>_<chunk>.mcap` next to `metadata.yaml`. We pick the
/// first hit deterministically (sorted by filename) because the schema
/// records are duplicated across split files -- they have to be, since
/// any consumer might start reading at any chunk.
[[nodiscard]] fs::path resolve_mcap_file(const InputSpec & spec)
{
  if (!spec.is_directory) {
    return fs::path{spec.uri};
  }
  std::vector<fs::path> candidates;
  for (const auto & entry : fs::directory_iterator(fs::path{spec.uri})) {
    if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
      candidates.push_back(entry.path());
    }
  }
  if (candidates.empty()) {
    throw std::runtime_error("no .mcap files found under directory: " + spec.uri);
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.front();
}

/// `MessageDefinitionSource` backed by an MCAP file's Summary section.
///
/// `load()` opens the file, parses the summary (a single seek/read at
/// the end of the file thanks to MCAP's footer-anchored layout) and
/// streams each Schema record into a `MessageDefinition`. The reader is
/// closed before returning so the source can be safely held while the
/// bag is reopened by other layers (e.g. `rosbag2_cpp::Reader`).
class McapMessageDefinitionSource final : public MessageDefinitionSource
{
public:
  explicit McapMessageDefinitionSource(fs::path mcap_path) : mcap_path_(std::move(mcap_path)) {}

  std::vector<MessageDefinition> load() override
  {
    mcap::McapReader reader;
    if (const auto status = reader.open(mcap_path_.string()); !status.ok()) {
      throw std::runtime_error(
        "failed to open mcap for schema scan: " + mcap_path_.string() + ": " + status.message);
    }
    // The Summary section is mandatory for indexed rosbag2 MCAP outputs
    // but can be absent in hand-rolled or truncated files. Fall back to
    // a sequential scan in that case so we still recover whatever schema
    // records the file does contain.
    if (const auto status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
        !status.ok()) {
      reader.close();
      throw std::runtime_error(
        "failed to read mcap summary: " + mcap_path_.string() + ": " + status.message);
    }

    std::vector<MessageDefinition> out;
    const auto & schemas = reader.schemas();
    out.reserve(schemas.size());
    for (const auto & entry : schemas) {
      const auto & schema_ptr = entry.second;
      if (!schema_ptr) {
        continue;
      }
      const auto & schema = *schema_ptr;
      // Skip dummies emitted when rosbag2_storage_mcap could not resolve
      // the type locally on the producer side -- forwarding an empty
      // schema would silently overwrite a definition the downstream
      // consumer might already have.
      if (schema.data.empty()) {
        continue;
      }
      MessageDefinition def;
      def.type_name = schema.name;
      def.encoding = schema.encoding.empty() ? std::string{"ros2msg"} : schema.encoding;
      // mcap's Schema.data is std::vector<std::byte>; .msg / .idl text
      // is ASCII/UTF-8, so a byte->char round trip is safe. Allocate
      // the destination up front and memcpy to keep both cpplint and
      // clang-tidy quiet (no reinterpret_cast, no pointer arithmetic).
      def.text.resize(schema.data.size());
      if (!schema.data.empty()) {
        std::memcpy(def.text.data(), schema.data.data(), schema.data.size());
      }
      out.push_back(std::move(def));
    }
    reader.close();
    return out;
  }

private:
  fs::path mcap_path_;
};

}  // namespace

std::unique_ptr<MessageDefinitionSource> make_mcap_definition_source(const InputSpec & spec)
{
  return std::make_unique<McapMessageDefinitionSource>(resolve_mcap_file(spec));
}

}  // namespace nebuladec::bag
