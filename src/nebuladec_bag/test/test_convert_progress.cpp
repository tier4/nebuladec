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
// End-to-end test for the `ConvertOptions::on_progress` hook. Asserts:
//
//   * The callback fires at least once across a real bag conversion.
//   * The final emitted ProgressEvent satisfies
//     `messages_done == messages_total > 0` -- i.e. the bag library
//     calls `ProgressReporter::finalize()` after the writer scope so
//     the bar always lands on 100%.
//   * `messages_total` is identical for every event the callback sees
//     (it is fixed at convert() entry from bag metadata).
//   * A callback that throws does not propagate out of convert() and
//     does not stop later progress events -- the ProgressReporter
//     swallows callback exceptions so a UI bug cannot abort decoding.
//
// Both the sequential path and the default 3-stage pipeline are
// exercised, since reader-side progress emission lives on different
// code paths in `run_convert_sequential` vs `run_convert_parallel`.

#include "nebuladec_bag/bag_io.hpp"

#include <nebuladec_core/topic_mapping.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

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
  auto base = fs::temp_directory_path() / ("nebuladec_progress_" + label + "_XXXXXX");
  std::string name_template = base.string();
  char * made = ::mkdtemp(name_template.data());
  if (made == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  fs::path path(made);
  fs::remove(path);  // convert() refuses to overwrite an existing path
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

/// Thread-safe sink that records every event for later assertions.
struct ProgressLog
{
  std::mutex mu;
  std::vector<ProgressEvent> events;

  void operator()(const ProgressEvent & ev)
  {
    std::lock_guard<std::mutex> lk(mu);
    events.push_back(ev);
  }
};

void run_and_check_progress(bool sequential)
{
  const fs::path bag_path = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;
  const auto out_path = make_tmp_dir(sequential ? "vlp16_seq_progress" : "vlp16_par_progress");

  auto opts = make_options(bag_path, out_path);
  opts.sequential = sequential;
  auto log = std::make_shared<ProgressLog>();
  opts.on_progress = [log](const ProgressEvent & ev) { (*log)(ev); };

  ASSERT_NO_THROW(convert(opts));

  std::lock_guard<std::mutex> lk(log->mu);
  ASSERT_FALSE(log->events.empty()) << "on_progress was never invoked";

  const auto total = log->events.front().messages_total;
  EXPECT_GT(total, 0U) << "decoded-topic message total must be non-zero";
  for (const auto & ev : log->events) {
    EXPECT_EQ(ev.messages_total, total) << "messages_total must be fixed for the call";
    EXPECT_LE(ev.messages_done, ev.messages_total);
  }
  // Monotonic non-decreasing done count.
  for (std::size_t i = 1; i < log->events.size(); ++i) {
    EXPECT_GE(log->events[i].messages_done, log->events[i - 1].messages_done);
  }
  // ProgressReporter::finalize() always fires the last snapshot.
  EXPECT_EQ(log->events.back().messages_done, total)
    << "final ProgressEvent must report messages_done == messages_total";
}

TEST(ConvertProgress, SequentialPathFiresCallback)
{
  run_and_check_progress(true);
}

TEST(ConvertProgress, ParallelPathFiresCallback)
{
  run_and_check_progress(false);
}

TEST(ConvertProgress, ThrowingCallbackDoesNotAbortConvert)
{
  const fs::path bag_path = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;
  const auto out_path = make_tmp_dir("vlp16_throw_progress");

  auto opts = make_options(bag_path, out_path);
  // Default pipeline (parallel). Callback throws every time -- the
  // ProgressReporter must swallow the exception, so convert() still
  // produces a valid result. We also assert the callback was actually
  // entered (a swallow + zero invocations would be a regression).
  std::atomic<std::size_t> call_count{0};
  opts.on_progress = [&call_count](const ProgressEvent &) {
    call_count.fetch_add(1, std::memory_order_acq_rel);
    throw std::runtime_error("test: progress callback failure");
  };

  ConvertResult result;
  ASSERT_NO_THROW(result = convert(opts));
  EXPECT_GT(call_count.load(), 0U);
  // Decode-side accounting is independent of the UI callback, so a
  // throwing on_progress must not corrupt packet counts.
  ASSERT_EQ(result.topics.size(), 1U);
  EXPECT_GT(result.topics.front().packets, 0U);
}

}  // namespace
}  // namespace nebuladec::bag
