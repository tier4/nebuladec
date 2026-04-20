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

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace nebuladec::bag
{

namespace
{

namespace fs = std::filesystem;

fs::path make_tmp_dir(const std::string & label)
{
  auto base = fs::temp_directory_path() / ("nebuladec_bag_" + label + "_XXXXXX");
  std::string name_template = base.string();
  char * made = ::mkdtemp(name_template.data());
  if (!made) {
    throw std::runtime_error("mkdtemp failed");
  }
  return fs::path(made);
}

void write_metadata(const fs::path & dir, const std::string & storage_identifier)
{
  std::ofstream out((dir / "metadata.yaml").string());
  out << "rosbag2_bagfile_information:\n"
      << "  version: 5\n"
      << "  storage_identifier: " << storage_identifier << "\n";
}

}  // namespace

TEST(DetectInput, DirectoryWithMcapMetadata)
{
  const auto dir = make_tmp_dir("mcap_dir");
  write_metadata(dir, "mcap");
  const auto spec = detect_input(dir.string());
  EXPECT_TRUE(spec.is_directory);
  EXPECT_EQ(spec.storage_id, "mcap");
  fs::remove_all(dir);
}

TEST(DetectInput, DirectoryWithSqlite3Metadata)
{
  const auto dir = make_tmp_dir("db3_dir");
  write_metadata(dir, "sqlite3");
  const auto spec = detect_input(dir.string());
  EXPECT_TRUE(spec.is_directory);
  EXPECT_EQ(spec.storage_id, "sqlite3");
  fs::remove_all(dir);
}

TEST(DetectInput, SingleMcapFile)
{
  const auto dir = make_tmp_dir("mcap_file");
  const auto file = dir / "bag.mcap";
  std::ofstream(file.string()) << "stub";
  const auto spec = detect_input(file.string());
  EXPECT_FALSE(spec.is_directory);
  EXPECT_EQ(spec.storage_id, "mcap");
  fs::remove_all(dir);
}

TEST(DetectInput, SingleDb3File)
{
  const auto dir = make_tmp_dir("db3_file");
  const auto file = dir / "bag.db3";
  std::ofstream(file.string()) << "stub";
  const auto spec = detect_input(file.string());
  EXPECT_FALSE(spec.is_directory);
  EXPECT_EQ(spec.storage_id, "sqlite3");
  fs::remove_all(dir);
}

TEST(DetectInput, DirectoryWithoutMetadataRejected)
{
  const auto dir = make_tmp_dir("no_meta");
  EXPECT_THROW(detect_input(dir.string()), std::invalid_argument);
  fs::remove_all(dir);
}

TEST(DetectInput, UnknownExtensionRejected)
{
  const auto dir = make_tmp_dir("bad_ext");
  const auto file = dir / "bag.txt";
  std::ofstream(file.string()) << "stub";
  EXPECT_THROW(detect_input(file.string()), std::invalid_argument);
  fs::remove_all(dir);
}

TEST(DetectInput, MissingPathRejected)
{
  EXPECT_THROW(detect_input("/no/such/path/nebuladec"), std::invalid_argument);
}

}  // namespace nebuladec::bag
