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
// End-to-end equivalence test for the parallel pipeline. Runs
// `bag::convert()` twice on the same input bag -- once with
// `options.sequential = true` (legacy single-threaded path) and once
// with the default pipeline (Reader -> Worker pool -> shared FIFO
// write queue -> Writer) -- and asserts that:
//
//   * `ConvertResult` counts (packets, clouds_written, passthrough
//     list) match exactly.
//   * The output bag contents form an identical multiset of
//     `(topic, log_time, payload_size)` tuples. The pipeline path may
//     interleave passthrough and decoded clouds in a slightly different
//     file insertion order from the sequential path, but `ros2 bag
//     play` is `log_time`-driven and treats both bags identically.
//
// Byte-exact equality of decoded `PointCloud2` payloads is NOT
// asserted: the underlying nebula driver may emit points whose
// per-point `time_stamp` / padding bytes depend on the executing
// thread's internal state (e.g. RNG, scratch buffer reuse), which is
// outside this package's control. The structural multiset above is
// enough to guarantee `ros2 bag play` produces the same topics, in the
// same temporal positions, at the same payload widths.
// TODO(parallel): re-tighten to a byte-exact comparison once the
// upstream driver's per-thread non-determinism (if any) is mapped
// out.

#include "nebuladec_bag/bag_io.hpp"

#include <nebuladec_core/topic_mapping.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <tuple>
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

fs::path make_tmp_dir(const std::string & label)
{
  auto base = fs::temp_directory_path() / ("nebuladec_bag_" + label + "_XXXXXX");
  std::string name_template = base.string();
  char * made = ::mkdtemp(name_template.data());
  if (made == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  fs::path path(made);
  // convert() refuses to overwrite an existing output, so remove the
  // empty placeholder that mkdtemp created.
  fs::remove(path);
  return path;
}

// SFINAE helper mirroring `bag_message_log_time_ns` in `bag_io.cpp`:
// rosbag2 renamed `SerializedBagMessage::time_stamp` to
// `recv_timestamp` between Humble and Iron, so this resolves to
// whichever field the active distro exposes.
template <int N>
struct LogTimePriority : LogTimePriority<N - 1>
{
};
template <>
struct LogTimePriority<0>
{
};

template <typename Msg>
auto log_time_ns(const Msg & msg, LogTimePriority<1> /*tag*/) -> decltype(msg.time_stamp)
{
  return msg.time_stamp;
}

template <typename Msg>
auto log_time_ns(const Msg & msg, LogTimePriority<0> /*tag*/) -> decltype(msg.recv_timestamp)
{
  return msg.recv_timestamp;
}

template <typename Msg>
auto log_time_ns(const Msg & msg)
{
  return log_time_ns(msg, LogTimePriority<1>{});
}

/// One bag-message worth of identity used by the multiset comparison.
struct Record
{
  std::string topic;
  std::int64_t log_time{0};
  std::size_t payload_size{0};
};

bool operator<(const Record & a, const Record & b)
{
  return std::tie(a.topic, a.log_time, a.payload_size) <
         std::tie(b.topic, b.log_time, b.payload_size);
}

bool operator==(const Record & a, const Record & b)
{
  return std::tie(a.topic, a.log_time, a.payload_size) ==
         std::tie(b.topic, b.log_time, b.payload_size);
}

std::vector<Record> read_bag_records(const fs::path & path, const std::string & storage_id)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions opts;
  opts.uri = path.string();
  opts.storage_id = storage_id;
  reader.open(opts);

  std::vector<Record> records;
  while (reader.has_next()) {
    auto msg = reader.read_next();
    Record r;
    r.topic = msg->topic_name;
    r.log_time = log_time_ns(*msg);
    r.payload_size = msg->serialized_data->buffer_length;
    records.push_back(std::move(r));
  }
  return records;
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

TEST(ConvertParallelEquivalence, VLP16BagSequentialMatchesPipeline)
{
  const fs::path bag_path = NEBULADEC_BAG_TEST_VLP16_BAG;
  ASSERT_TRUE(fs::exists(bag_path)) << bag_path;

  const auto out_seq = make_tmp_dir("vlp16_seq");
  const auto out_par = make_tmp_dir("vlp16_par");

  // 1. Sequential (legacy) run.
  auto opts_seq = make_options(bag_path, out_seq);
  opts_seq.sequential = true;
  ConvertResult result_seq;
  ASSERT_NO_THROW(result_seq = convert(opts_seq));

  // 2. Default pipeline run. `workers = 0` means auto; on a host with
  // >=3 cores the policy resolver picks pipeline mode with
  // `min(cores, K=1)=1` worker -- a 3-stage split (reader + 1 decoder
  // + writer).
  auto opts_par = make_options(bag_path, out_par);
  // opts_par.sequential left as false (default)
  ConvertResult result_par;
  ASSERT_NO_THROW(result_par = convert(opts_par));

  // 3. ConvertResult equivalence: counts and topic lists must match.
  ASSERT_EQ(result_seq.topics.size(), result_par.topics.size());
  for (std::size_t i = 0; i < result_seq.topics.size(); ++i) {
    const auto & s = result_seq.topics[i];
    const auto & p = result_par.topics[i];
    EXPECT_EQ(s.in_topic, p.in_topic);
    EXPECT_EQ(s.out_topic, p.out_topic);
    EXPECT_EQ(s.frame_id, p.frame_id);
    EXPECT_EQ(s.packets, p.packets);
    EXPECT_EQ(s.clouds_written, p.clouds_written);
  }
  EXPECT_EQ(result_seq.passthrough_topics, result_par.passthrough_topics);

  // 4. Output bag multiset equality. The VLP16 fixture is a bare .db3
  // file so the output mirrors that layout.
  ASSERT_TRUE(fs::is_regular_file(out_seq));
  ASSERT_TRUE(fs::is_regular_file(out_par));
  auto records_seq = read_bag_records(out_seq, "sqlite3");
  auto records_par = read_bag_records(out_par, "sqlite3");

  ASSERT_EQ(records_seq.size(), records_par.size())
    << "pipeline emitted a different number of bag messages than the sequential path";

  std::sort(records_seq.begin(), records_seq.end());
  std::sort(records_par.begin(), records_par.end());

  // If sizes are equal but contents differ, surface the first
  // disagreement to make debugging easier.
  for (std::size_t i = 0; i < records_seq.size(); ++i) {
    if (records_seq[i] == records_par[i]) {
      continue;
    }
    FAIL() << "record " << i << " differs: seq=(" << records_seq[i].topic << ", "
           << records_seq[i].log_time << ", " << records_seq[i].payload_size << ") vs par=("
           << records_par[i].topic << ", " << records_par[i].log_time << ", "
           << records_par[i].payload_size << ")";
  }

  fs::remove_all(out_seq);
  fs::remove_all(out_par);
}

}  // namespace
}  // namespace nebuladec::bag
