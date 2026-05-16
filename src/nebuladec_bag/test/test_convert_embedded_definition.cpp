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

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>

namespace
{

namespace fs = std::filesystem;

class TempDir
{
public:
  TempDir()
  {
    std::string templ = (fs::temp_directory_path() / "nebuladec_convert_embed_XXXXXX").string();
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

/// Build an MCAP file containing one passthrough topic whose ROS 2
/// message type package is NOT installed in the test environment, plus
/// the embedded Schema record carrying its .msg text. This is the
/// minimum repro for the user's scenario.
void write_unknown_type_mcap(const fs::path & path)
{
  mcap::McapWriter writer;
  mcap::McapWriterOptions opts{"ros2"};
  opts.compression = mcap::Compression::None;
  ASSERT_TRUE(writer.open(path.string(), opts).ok());

  // Synthetic schema text. The contents are opaque to the converter --
  // it only needs to round-trip them byte-for-byte into the output.
  const std::string schema_text =
    "# fake unknown type used by test_convert_embedded_definition\n"
    "uint64 stamp_ns\n"
    "string label\n";
  mcap::Schema schema{"fake_msgs/msg/UnknownType", "ros2msg", schema_text};
  writer.addSchema(schema);
  mcap::Channel channel{"/fake/unknown", "cdr", schema.id};
  writer.addChannel(channel);

  // One CDR-shaped payload. The bytes are arbitrary -- nebuladec
  // passthrough does not deserialize them, so any non-empty blob works.
  const std::byte payload[] = {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00},
                               std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  mcap::Message msg{};
  msg.channelId = channel.id;
  msg.sequence = 0;
  msg.logTime = 1000;
  msg.publishTime = 1000;
  msg.dataSize = sizeof(payload);
  msg.data = payload;
  ASSERT_TRUE(writer.write(msg).ok());
  writer.close();
}

struct ReadbackSchema
{
  std::string encoding;
  std::string text;
};

std::unordered_map<std::string, ReadbackSchema> read_schemas(const fs::path & path)
{
  mcap::McapReader reader;
  EXPECT_TRUE(reader.open(path.string()).ok());
  EXPECT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan).ok());
  std::unordered_map<std::string, ReadbackSchema> out;
  for (const auto & [id, schema_ptr] : reader.schemas()) {
    if (!schema_ptr) {
      continue;
    }
    ReadbackSchema rb;
    rb.encoding = schema_ptr->encoding;
    rb.text.resize(schema_ptr->data.size());
    if (!schema_ptr->data.empty()) {
      std::memcpy(rb.text.data(), schema_ptr->data.data(), schema_ptr->data.size());
    }
    out.emplace(schema_ptr->name, std::move(rb));
  }
  reader.close();
  return out;
}

}  // namespace

TEST(ConvertEmbeddedDefinition, ForwardsSchemaForUnknownPassthroughType)
{
  TempDir scratch;
  const auto input = scratch.path() / "input.mcap";
  const auto output = scratch.path() / "output.mcap";
  write_unknown_type_mcap(input);

  // Empty mapping -> no decode rules -> every topic is passthrough.
  nebuladec::bag::ConvertOptions opts;
  opts.input_path = input.string();
  opts.output_path = output.string();
  opts.mapping = nebuladec::TopicMapping{};
  ASSERT_NO_THROW(nebuladec::bag::convert(opts));
  ASSERT_TRUE(fs::exists(output));

  const auto schemas = read_schemas(output);
  const auto it = schemas.find("fake_msgs/msg/UnknownType");
  ASSERT_NE(it, schemas.end()) << "passthrough schema dropped";
  EXPECT_EQ(it->second.encoding, "ros2msg");
  EXPECT_NE(it->second.text.find("uint64 stamp_ns"), std::string::npos);
  EXPECT_NE(it->second.text.find("string label"), std::string::npos);
}
