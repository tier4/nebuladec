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

#include "nebuladec_bag/bag_io.hpp"
#include "nebuladec_bag/message_definition.hpp"

#include <mcap/writer.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

namespace
{

namespace fs = std::filesystem;

/// Scratch directory unique to the test process; cleaned up on
/// destruction. Mirrors the pattern used by test_detect_input.
class TempDir
{
public:
  TempDir()
  {
    std::string templ = (fs::temp_directory_path() / "nebuladec_msgdef_XXXXXX").string();
    char * resolved = mkdtemp(templ.data());
    if (resolved == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path_ = resolved;
  }
  ~TempDir()
  {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  TempDir(TempDir &&) = delete;
  TempDir & operator=(TempDir &&) = delete;
  [[nodiscard]] const fs::path & path() const noexcept { return path_; }

private:
  fs::path path_;
};

/// Build a minimal MCAP file at `path` containing two Schema records
/// for the supplied (type, encoding, text) pairs and no messages. The
/// file is closed before returning so callers can re-open it.
void write_schema_only_mcap(
  const fs::path & path, std::string_view type_a, std::string_view text_a, std::string_view type_b,
  std::string_view text_b)
{
  mcap::McapWriter writer;
  mcap::McapWriterOptions opts{"ros2"};
  opts.compression = mcap::Compression::None;
  const auto status = writer.open(path.string(), opts);
  ASSERT_TRUE(status.ok()) << status.message;

  mcap::Schema schema_a{type_a, "ros2msg", text_a};
  mcap::Schema schema_b{type_b, "ros2idl", text_b};
  writer.addSchema(schema_a);
  writer.addSchema(schema_b);
  writer.close();
}

}  // namespace

TEST(McapMessageDefinitionSource, RoundTripsSchemaRecordsByteForByte)
{
  TempDir scratch;
  const auto mcap_path = scratch.path() / "fixture.mcap";
  const std::string_view ncom_text =
    "# fake oxts_msgs/msg/Ncom\n"
    "float64 latitude\n"
    "float64 longitude\n";
  const std::string_view idl_text = "module fake { struct Msg { double v; }; };\n";
  write_schema_only_mcap(mcap_path, "oxts_msgs/msg/Ncom", ncom_text, "fake/msg/Idl", idl_text);

  const nebuladec::bag::InputSpec spec{mcap_path.string(), "mcap", /*is_directory=*/false};
  auto registry = nebuladec::bag::load_definition_registry(spec);

  ASSERT_EQ(registry.size(), 2U);
  const auto a = registry.find("oxts_msgs/msg/Ncom");
  ASSERT_TRUE(a.has_value());
  EXPECT_EQ(a->encoding, "ros2msg");
  EXPECT_EQ(a->text, std::string{ncom_text});

  const auto b = registry.find("fake/msg/Idl");
  ASSERT_TRUE(b.has_value());
  EXPECT_EQ(b->encoding, "ros2idl");
  EXPECT_EQ(b->text, std::string{idl_text});

  EXPECT_FALSE(registry.contains("nope/msg/Missing"));
}

TEST(McapMessageDefinitionSource, ReturnsEmptyForFileWithoutSchemas)
{
  TempDir scratch;
  const auto mcap_path = scratch.path() / "empty.mcap";
  mcap::McapWriter writer;
  mcap::McapWriterOptions opts{"ros2"};
  opts.compression = mcap::Compression::None;
  ASSERT_TRUE(writer.open(mcap_path.string(), opts).ok());
  writer.close();

  const nebuladec::bag::InputSpec spec{mcap_path.string(), "mcap", false};
  auto registry = nebuladec::bag::load_definition_registry(spec);
  EXPECT_TRUE(registry.empty());
}

TEST(McapMessageDefinitionSource, ThrowsOnMissingFile)
{
  const nebuladec::bag::InputSpec spec{"/tmp/nebuladec-msgdef-nonexistent.mcap", "mcap", false};
  auto source = nebuladec::bag::make_definition_source(spec);
  EXPECT_THROW(static_cast<void>(source->load()), std::runtime_error);
}

TEST(SqliteMessageDefinitionSource, ReturnsEmptyForBareDb3WithoutMetadata)
{
  // Simulates the Humble "bare .db3 in /tmp" case -- no metadata.yaml,
  // so the source must soft-fail to empty (matches the user's signed-
  // off soft-fail policy when no embedded definitions exist).
  TempDir scratch;
  const auto db3_path = scratch.path() / "bare.db3";
  std::ofstream{db3_path} << "fake-db3";

  const nebuladec::bag::InputSpec spec{db3_path.string(), "sqlite3", false};
  auto registry = nebuladec::bag::load_definition_registry(spec);
  EXPECT_TRUE(registry.empty());
}

TEST(SqliteMessageDefinitionSource, ReadsIronStyleMessageDefinitionBlock)
{
  TempDir scratch;
  // Synthetic Iron-flavoured metadata.yaml: rosbag2 from Iron onward
  // writes one `topic_metadata.message_definition` per topic.
  const auto yaml_path = scratch.path() / "metadata.yaml";
  std::ofstream{yaml_path} << "rosbag2_bagfile_information:\n"
                           << "  version: 5\n"
                           << "  storage_identifier: sqlite3\n"
                           << "  topics_with_message_count:\n"
                           << "    - topic_metadata:\n"
                           << "        name: /sensor/ncom\n"
                           << "        type: oxts_msgs/msg/Ncom\n"
                           << "        serialization_format: cdr\n"
                           << "        message_definition: |\n"
                           << "          float64 latitude\n"
                           << "          float64 longitude\n"
                           << "    - topic_metadata:\n"
                           << "        name: /no_def_here\n"
                           << "        type: pkg/msg/NoDef\n"
                           << "        serialization_format: cdr\n";

  const nebuladec::bag::InputSpec spec{scratch.path().string(), "sqlite3", true};
  auto registry = nebuladec::bag::load_definition_registry(spec);
  EXPECT_EQ(registry.size(), 1U);
  const auto def = registry.find("oxts_msgs/msg/Ncom");
  ASSERT_TRUE(def.has_value());
  EXPECT_EQ(def->encoding, "ros2msg");
  EXPECT_NE(def->text.find("latitude"), std::string::npos);
  EXPECT_FALSE(registry.contains("pkg/msg/NoDef"));
}
