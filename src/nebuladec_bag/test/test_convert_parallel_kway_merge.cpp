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
// Bag-agnostic unit tests for `k_way_merge_drive`, exercising four
// scenarios:
//
//   1. Single source, N items -> merger emits all N in order.
//   2. Two sources with interleaved stamps -> merger interleaves them
//      by `log_time`.
//   3. One source long-empty with a low watermark -> merger does NOT
//      consume the other source's available head until the watermark
//      advances past it (head-of-line blocking avoidance).
//   4. AbortFlag.set() while the merger is waiting -> merger returns
//      promptly even though no source has produced new items.

#include "../src/convert_parallel.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <thread>
#include <vector>

namespace nebuladec::bag
{
namespace
{

/// Minimal in-test source backed by a `BoundedQueue<int64_t>` plus a
/// `Watermark`. The test thread pushes stamps and closes; the merger
/// pops them into `consumed_log`.
struct TestSource
{
  TestSource(AbortFlag & abort, std::vector<std::int64_t> & consumed)
  : queue(64, abort), wm(abort), consumed_log(consumed)
  {
  }

  BoundedQueue<std::int64_t> queue;
  Watermark wm;
  std::vector<std::int64_t> & consumed_log;

  OutputSource as_source()
  {
    OutputSource src;
    src.peek = [this](std::int64_t & out_stamp) -> bool {
      std::int64_t head = 0;
      if (queue.peek(head)) {
        out_stamp = head;
        return true;
      }
      return false;
    };
    src.is_eof = [this]() -> bool { return queue.empty_and_closed(); };
    src.watermark = [this]() -> std::int64_t { return wm.load(); };
    src.consume_head = [this]() {
      std::int64_t stamp = 0;
      if (queue.try_pop(stamp)) {
        consumed_log.push_back(stamp);
      }
    };
    return src;
  }
};

TEST(KWayMerge, SingleSourceInOrder)
{
  AbortFlag abort;
  std::vector<std::int64_t> consumed;
  TestSource s(abort, consumed);

  for (std::int64_t t : {100, 200, 300, 400, 500}) {
    ASSERT_TRUE(s.queue.push(t));
  }
  s.wm.advance(500);
  s.queue.close();
  s.wm.close();

  std::vector<OutputSource> sources;
  sources.push_back(s.as_source());

  std::thread merger([&] { k_way_merge_drive(sources, abort); });
  merger.join();

  ASSERT_EQ(consumed.size(), 5U);
  EXPECT_EQ(consumed, (std::vector<std::int64_t>{100, 200, 300, 400, 500}));
}

TEST(KWayMerge, TwoSourcesInterleavedByLogTime)
{
  AbortFlag abort;
  std::vector<std::int64_t> consumed;
  TestSource a(abort, consumed);
  TestSource b(abort, consumed);

  // a: 100, 300, 500. b: 200, 400, 600. Merged: 100..600.
  for (std::int64_t t : {100, 300, 500}) {
    ASSERT_TRUE(a.queue.push(t));
  }
  for (std::int64_t t : {200, 400, 600}) {
    ASSERT_TRUE(b.queue.push(t));
  }
  // Set watermarks to each source's last stamp so the merger never
  // blocks waiting for empty sources during the drain.
  a.wm.advance(500);
  b.wm.advance(600);
  a.queue.close();
  b.queue.close();
  a.wm.close();
  b.wm.close();

  std::vector<OutputSource> sources;
  sources.push_back(a.as_source());
  sources.push_back(b.as_source());

  std::thread merger([&] { k_way_merge_drive(sources, abort); });
  merger.join();

  EXPECT_EQ(consumed, (std::vector<std::int64_t>{100, 200, 300, 400, 500, 600}));
}

TEST(KWayMerge, WatermarkBlocksThenUnblocks)
{
  AbortFlag abort;
  std::vector<std::int64_t> consumed;
  TestSource a(abort, consumed);
  TestSource b(abort, consumed);

  // a has stamp 200 ready; b is empty with watermark 100 -- the
  // merger does NOT yet know b's future stamps are >= 200, so it
  // must wait before emitting a's head.
  ASSERT_TRUE(a.queue.push(200));
  a.wm.advance(200);
  b.wm.advance(100);

  std::vector<OutputSource> sources;
  sources.push_back(a.as_source());
  sources.push_back(b.as_source());

  std::thread merger([&] { k_way_merge_drive(sources, abort); });

  // Give the merger time to settle on its wait. With the watermark
  // blocker in place, nothing should have been consumed.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_TRUE(consumed.empty()) << "merger consumed prematurely (head-of-line guard broken)";

  // Advance b's watermark to 250: b can never produce a stamp < 250,
  // so a's 200 head is safe to emit.
  b.wm.advance(250);

  // Allow the merger to consume a's 200 head, then close both.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  a.queue.close();
  b.queue.close();
  a.wm.close();
  b.wm.close();

  merger.join();

  ASSERT_EQ(consumed.size(), 1U);
  EXPECT_EQ(consumed.front(), 200);
}

TEST(KWayMerge, AbortReturnsPromptlyEvenWhenWaiting)
{
  AbortFlag abort;
  std::vector<std::int64_t> consumed;
  TestSource a(abort, consumed);
  TestSource b(abort, consumed);

  // Both sources empty, neither closed -> merger waits.
  std::vector<OutputSource> sources;
  sources.push_back(a.as_source());
  sources.push_back(b.as_source());

  std::thread merger([&] { k_way_merge_drive(sources, abort); });

  // Let the merger enter its wait.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Fire abort. The merger must return within the deadline.
  const auto t0 = std::chrono::steady_clock::now();
  abort.set(std::make_exception_ptr(std::runtime_error("test abort")));
  merger.join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(elapsed, std::chrono::milliseconds(200))
    << "merger did not return promptly after AbortFlag::set";
  EXPECT_TRUE(abort.aborted());

  auto ep = abort.take();
  ASSERT_TRUE(ep) << "AbortFlag::take must return the captured exception_ptr";
}

}  // namespace
}  // namespace nebuladec::bag
