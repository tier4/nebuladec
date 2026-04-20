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
// Regression test for the Hesai cloud-ownership bug: HesaiDecoder clears
// its frame buffer immediately after invoking the scan callback, so the
// adapter must own a copy of the cloud it queues. Before the fix, every
// cloud handed back by HesaiAdapter::feed() was empty and silently
// dropped by Decoder::feed()'s min_points filter -- reporting
// "clouds_written: 0" even though the sensor was correctly identified.
//
// Replays the upstream nebula_hesai QT128 ground-truth bag through the
// full convert() pipeline and asserts at least one cloud lands in the
// output.

#include "nebuladec_bag/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#ifndef NEBULADEC_BAG_TEST_QT128_BAG
#error "NEBULADEC_BAG_TEST_QT128_BAG must be defined to the QT128 ground-truth .db3 path"
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
  // convert() opens the output for write, so the directory itself must
  // not pre-exist. mkdtemp leaves it empty, so remove then pass.
  fs::path path(made);
  fs::remove(path);
  return path;
}

}  // namespace

TEST(ConvertHesaiQT128, GroundTruthBagProducesClouds)
{
  const fs::path bag_path = NEBULADEC_BAG_TEST_QT128_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;

  const auto out_dir = make_tmp_dir("qt128_out");

  ConvertOptions options;
  options.input_path = bag_path.string();
  options.output_path = out_dir.string();

  ConvertResult result;
  ASSERT_NO_THROW(result = convert(options));

  ASSERT_TRUE(result.identity.has_value());
  EXPECT_EQ(result.identity->vendor, Vendor::HESAI);
  EXPECT_EQ(result.identity->model, nebula::drivers::SensorModel::HESAI_PANDARQT128);
  EXPECT_GT(result.data_packets, 0U);
  // Regression #1: with the pre-fix adapter, ready_clouds_ held
  // shared_ptrs to the decoder's frame buffer, which was cleared
  // immediately after the callback. Every cloud looked empty
  // (< min_points) and was dropped -- clouds_written stayed at 0
  // despite full decoding.
  //
  // Regression #2: the ground-truth bag contains exactly two PandarScan
  // messages (one full QT128 rotation at 10 Hz), but the decoder emits
  // a cloud only when the *next* packet crosses the cut angle. Without
  // flush() at end-of-stream, the trailing scan stays buffered inside
  // the driver. Expect both scans to land in the output.
  EXPECT_GE(result.clouds_written, 2U);

  fs::remove_all(out_dir);
}

}  // namespace nebuladec::bag
