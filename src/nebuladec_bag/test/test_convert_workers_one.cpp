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
// Regression test for the `--workers 1` (more generally `workers < K`)
// deadlock that the original parallel pipeline shipped with.
//
// The old per-slot input queue + serial-per-slot worker drained slot
// `i` to EOF before touching slot `i+1`. With workers < K the reader,
// forced to push messages in bag log-time order, filled the
// unattended slot's queue to its `k_default_input_queue_capacity` cap
// and blocked on `push()`; the worker waited forever on the closed
// signal of the slot it was draining. End result: convert() never
// returned.
//
// This test reproduces that scenario end-to-end. It synthesises a
// two-topic VelodyneScan bag from the existing VLP16 fixture
// (replicating each source message many times across both topics so
// the per-worker queue capacity is exceeded), runs `convert()` with
// `workers = 1` inside `std::async`, and asserts the call finishes
// inside a wall-clock budget. A hang fails the test via
// `wait_for(timeout) == timeout`.

#include "nebuladec_bag/bag_io.hpp"

#include <nebuladec_core/topic_mapping.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef NEBULADEC_BAG_TEST_VLP16_BAG
#error "NEBULADEC_BAG_TEST_VLP16_BAG must be defined to the VLP16 ground-truth .db3 path"
#endif

namespace nebuladec::bag
{
namespace
{

namespace fs = std::filesystem;

// Mirrors the SFINAE shim in bag_io.cpp -- rosbag2 renamed
// `SerializedBagMessage::time_stamp` to `recv_timestamp` between
// Humble and Iron, so we resolve to whichever the active distro has.
template <int N>
struct LogTimePriority : LogTimePriority<N - 1>
{
};
template <>
struct LogTimePriority<0>
{
};

template <typename Msg>
auto read_log_time(const Msg & m, LogTimePriority<1> /*tag*/) -> decltype(m.time_stamp)
{
  return m.time_stamp;
}
template <typename Msg>
auto read_log_time(const Msg & m, LogTimePriority<0> /*tag*/) -> decltype(m.recv_timestamp)
{
  return m.recv_timestamp;
}
template <typename Msg>
auto read_log_time(const Msg & m)
{
  return read_log_time(m, LogTimePriority<1>{});
}

template <typename Msg>
auto write_log_time(Msg & m, std::int64_t stamp_ns, LogTimePriority<1> /*tag*/)
  -> decltype(m.time_stamp, void())
{
  m.time_stamp = stamp_ns;
}
template <typename Msg>
auto write_log_time(Msg & m, std::int64_t stamp_ns, LogTimePriority<0> /*tag*/)
  -> decltype(m.recv_timestamp, void())
{
  m.recv_timestamp = stamp_ns;
}
template <typename Msg>
void write_log_time(Msg & m, std::int64_t stamp_ns)
{
  write_log_time(m, stamp_ns, LogTimePriority<1>{});
}

fs::path make_tmp_dir(const std::string & label)
{
  auto base = fs::temp_directory_path() / ("nebuladec_workers_one_" + label + "_XXXXXX");
  std::string name_template = base.string();
  char * made = ::mkdtemp(name_template.data());
  if (made == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  fs::path path(made);
  fs::remove(path);  // convert() refuses to overwrite an existing path
  return path;
}

/// Build a synthetic two-topic VelodyneScan bag at `out_bag_path` by
/// reading every message out of the VLP16 fixture, then writing each
/// source message `replication` times into each of two target topics.
/// Messages are interleaved a/b/a/b/... so the reader stage of
/// `convert()` sees both topics arriving concurrently. Per-topic count
/// is chosen well above `k_default_input_queue_capacity = 256` so the
/// pre-fix per-slot queue would have filled and deadlocked.
void build_two_topic_bag(
  const fs::path & src_bag_path, const fs::path & out_bag_path, std::size_t replication)
{
  struct SrcMessage
  {
    std::shared_ptr<rcutils_uint8_array_t> serialized_data;
    std::int64_t time_stamp_ns{0};
  };
  std::vector<SrcMessage> src_messages;
  std::string src_type;
  std::string src_serialization;
  {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions opts;
    opts.uri = src_bag_path.string();
    opts.storage_id = "sqlite3";
    reader.open(opts);
    for (const auto & info : reader.get_metadata().topics_with_message_count) {
      const auto & meta = info.topic_metadata;
      if (meta.name == "/velodyne_packets") {
        src_type = meta.type;
        src_serialization = meta.serialization_format;
        break;
      }
    }
    if (src_type.empty()) {
      throw std::runtime_error("VLP16 fixture missing /velodyne_packets topic");
    }
    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      SrcMessage sm;
      sm.serialized_data = bag_msg->serialized_data;
      sm.time_stamp_ns = read_log_time(*bag_msg);
      src_messages.push_back(std::move(sm));
    }
  }
  ASSERT_FALSE(src_messages.empty()) << "VLP16 fixture has zero messages";

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions out_opts;
  out_opts.uri = out_bag_path.string();
  out_opts.storage_id = "sqlite3";
  writer.open(out_opts);

  const std::string topic_a = "/velodyne_packets_a";
  const std::string topic_b = "/velodyne_packets_b";
  rosbag2_storage::TopicMetadata meta_a;
  meta_a.name = topic_a;
  meta_a.type = src_type;
  meta_a.serialization_format = src_serialization;
  rosbag2_storage::TopicMetadata meta_b = meta_a;
  meta_b.name = topic_b;
  writer.create_topic(meta_a);
  writer.create_topic(meta_b);

  // Interleave a/b/a/b/... with strictly increasing timestamps so
  // rosbag2 stores them in this exact order. The reader then pulls
  // them in the same order, which is the worst case for the old
  // per-slot queue (one slot fills to cap while the other waits).
  std::int64_t stamp_ns = src_messages.front().time_stamp_ns;
  constexpr std::int64_t k_step_ns = 1'000;  // 1 us between consecutive writes
  for (std::size_t r = 0; r < replication; ++r) {
    for (const auto & sm : src_messages) {
      auto write_one = [&](const std::string & topic) {
        auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_msg->topic_name = topic;
        bag_msg->serialized_data = sm.serialized_data;
        write_log_time(*bag_msg, stamp_ns);
        writer.write(bag_msg);
        stamp_ns += k_step_ns;
      };
      write_one(topic_a);
      write_one(topic_b);
    }
  }
}

TEST(ConvertWorkersOne, DoesNotDeadlockOnTwoTopics)
{
  const fs::path src_bag = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(src_bag)) << src_bag;

  const auto staging = make_tmp_dir("staging");
  fs::create_directories(staging);
  const auto two_topic_bag = staging / "two_topic_vlp16";
  // 80 reps * 4 src msgs = 320 per topic, 640 total. Pre-fix code
  // would deadlock at ~256 messages on the second topic's per-slot
  // queue (default cap).
  ASSERT_NO_THROW(build_two_topic_bag(src_bag, two_topic_bag, 80U));

  const auto out_path = make_tmp_dir("out");

  ConvertOptions opts;
  opts.input_path = two_topic_bag.string();
  opts.output_path = out_path.string();
  opts.mapping = TopicMapping::from_yaml_string(
    "mapping:\n"
    "  - in_topic:  /velodyne_packets_a\n"
    "    frame_id:  lidar_a\n"
    "    out_topic: /velodyne_points_a\n"
    "  - in_topic:  /velodyne_packets_b\n"
    "    frame_id:  lidar_b\n"
    "    out_topic: /velodyne_points_b\n");
  opts.sequential = false;  // force the parallel driver
  opts.workers = 1U;        // K=2, workers=1 -- the deadlock condition

  // Run convert() on a worker thread and budget a generous 60s for
  // completion. The pre-fix code never returned; a passing run takes
  // well under a second on a laptop. Using a future + wait_for keeps
  // the gtest binary itself responsive even if a future regression
  // re-introduces the hang.
  auto fut = std::async(std::launch::async, [&] { return convert(opts); });
  const auto status = fut.wait_for(std::chrono::seconds(60));
  ASSERT_EQ(status, std::future_status::ready)
    << "convert(workers=1, K=2) hung -- per-worker input queue regression";

  ConvertResult result;
  ASSERT_NO_THROW(result = fut.get());
  ASSERT_EQ(result.topics.size(), 2U);
  for (const auto & t : result.topics) {
    EXPECT_GT(t.packets, 0U) << "topic " << t.in_topic << " produced no decoded packets";
  }
}

}  // namespace
}  // namespace nebuladec::bag
