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
//
// Contract test for `ConvertOptions::mcap`. The MCAP writer options
// only take effect when the input bag is MCAP (`convert()` mirrors
// the storage plugin on output). When the input is sqlite3 the
// library is required to log a warning and silently ignore the
// options. This test pins that behaviour against the existing VLP16
// sqlite3 fixture: setting every MCAP option to a non-default value
// must not change the conversion outcome.

#include "nebuladec_bag/bag_io.hpp"

#include <nebuladec_core/topic_mapping.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#ifndef NEBULADEC_BAG_TEST_VLP16_BAG
#error "NEBULADEC_BAG_TEST_VLP16_BAG must be defined to the VLP16 ground-truth .db3 path"
#endif

namespace nebuladec::bag
{
namespace
{

namespace fs = std::filesystem;

fs::path make_tmp_dir(const std::string & label)
{
  auto base = fs::temp_directory_path() / ("nebuladec_mcap_opts_test_" + label + "_XXXXXX");
  std::string name_template = base.string();
  char * made = ::mkdtemp(name_template.data());
  if (made == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  fs::path path(made);
  fs::remove(path);  // convert() refuses to overwrite
  return path;
}

ConvertOptions make_options(const fs::path & in_bag, const fs::path & out_path)
{
  ConvertOptions opts;
  opts.input_path = in_bag.string();
  opts.output_path = out_path.string();
  opts.mapping = TopicMapping::from_yaml_string(
    "mapping:\n"
    "  - in_topic:  /velodyne_packets\n"
    "    frame_id:  lidar\n"
    "    out_topic: /velodyne_points\n");
  return opts;
}

TEST(ConvertMcapOptions, Sqlite3InputIgnoresMcapOptions)
{
  const fs::path bag_path = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;
  const auto out_path = make_tmp_dir("warn_ignore");

  auto opts = make_options(bag_path, out_path);
  // Set every MCAP knob to something obviously non-default. On a
  // sqlite3 input these must be silently ignored (with a single
  // WARN-level log line emitted by the library, which we do not
  // capture here -- the contract is "do not fail").
  opts.mcap.compression = McapCompression::kLz4;
  opts.mcap.compression_level = McapCompressionLevel::kFastest;
  opts.mcap.chunk_size_bytes = 4U * 1024U * 1024U;

  ConvertResult result;
  ASSERT_NO_THROW(result = convert(opts));

  // Output bag must exist and the conversion must have produced the
  // single expected decoded topic, exactly as it would without any
  // MCAP option set.
  ASSERT_TRUE(fs::exists(out_path));
  ASSERT_EQ(result.topics.size(), 1U);
  EXPECT_EQ(result.topics.front().in_topic, "/velodyne_packets");
  EXPECT_EQ(result.topics.front().out_topic, "/velodyne_points");
  EXPECT_GT(result.topics.front().packets, 0U);
}

TEST(ConvertMcapOptions, EmptyOptionsAreNoOp)
{
  // Sanity: the default-constructed McapWriteOptions{} must not
  // change behaviour for any input. Covers the kAuto / 0 path.
  const fs::path bag_path = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;
  const auto out_path = make_tmp_dir("noop");

  auto opts = make_options(bag_path, out_path);
  opts.mcap = McapWriteOptions{};  // explicit "no overrides"

  ConvertResult result;
  ASSERT_NO_THROW(result = convert(opts));
  EXPECT_EQ(result.topics.size(), 1U);
}

}  // namespace
}  // namespace nebuladec::bag
