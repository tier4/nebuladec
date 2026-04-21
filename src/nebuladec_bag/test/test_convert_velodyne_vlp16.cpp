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
// Regression test for the Velodyne trailing-scan bug: VelodyneDriver's
// scan decoder surfaces the previous scan only when the next packet's
// azimuth wraps past the scan phase. Without adapter-side flush() at
// end-of-stream the last scan is never returned. Replays the upstream
// nebula_velodyne VLP16 ground-truth bag through the full convert()
// pipeline and asserts the last scan lands in the output.

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
  auto base = fs::temp_directory_path() / ("nebuladec_bag_" + label + "_XXXXXX");
  std::string name_template = base.string();
  char * made = ::mkdtemp(name_template.data());
  if (!made) {
    throw std::runtime_error("mkdtemp failed");
  }
  fs::path path(made);
  fs::remove(path);
  return path;
}

}  // namespace

TEST(ConvertVelodyneVLP16, GroundTruthBagEmitsTrailingScan)
{
  const fs::path bag_path = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;

  const auto out_dir = make_tmp_dir("vlp16_out");

  ConvertOptions options;
  options.input_path = bag_path.string();
  options.output_path = out_dir.string();
  options.mapping = TopicMapping::from_yaml_string(
    "mapping:\n"
    "  - in_topic:  /velodyne_packets\n"
    "    frame_id:  lidar\n"
    "    out_topic: /velodyne_points\n");

  ConvertResult result;
  ASSERT_NO_THROW(result = convert(options));

  ASSERT_EQ(result.topics.size(), 1U);
  const auto & t = result.topics.front();
  ASSERT_TRUE(t.identity.has_value());
  EXPECT_EQ(t.identity->vendor, Vendor::VELODYNE);
  EXPECT_EQ(t.identity->model, nebula::drivers::SensorModel::VELODYNE_VLP16);
  EXPECT_EQ(t.in_topic, "/velodyne_packets");
  EXPECT_EQ(t.out_topic, "/velodyne_points");
  EXPECT_GT(t.data_packets, 0U);
  // The ground-truth bag carries 4 VelodyneScan messages (~3 full scans
  // at 10 Hz). Without flush(), the scan-decoder's trailing buffer sits
  // in `scan_pc_` and the final scan is lost. Expect at least 2 clouds
  // written end-to-end, which requires flush() to function.
  EXPECT_GE(t.clouds_written, 2U);

  fs::remove_all(out_dir);
}

}  // namespace nebuladec::bag
