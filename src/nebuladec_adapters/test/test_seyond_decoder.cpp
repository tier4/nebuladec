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

#include "../src/seyond_decoder.hpp"

#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_packet.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace nebuladec::adapters
{
namespace
{

nebula::drivers::SeyondSensorConfiguration make_robin_w_config()
{
  nebula::drivers::SeyondSensorConfiguration config;
  config.sensor_model = nebula::drivers::SeyondSensorModel::ROBIN_W;
  config.connection.host_ip = "";
  config.connection.sensor_ip = "";
  config.connection.netmask = "";
  config.connection.gateway = "";
  config.connection.udp_port = 0;
  config.connection.udp_message_port = 0;
  config.connection.udp_status_port = 0;
  config.fov.azimuth = {0.0F, 0.0F};
  config.fov.elevation = {0.0F, 0.0F};
  config.use_sensor_time = false;
  config.frame_id = "seyond";
  config.setup_sensor = false;
  config.return_mode = nebula::drivers::ReturnMode::STRONGEST;
  return config;
}

SeyondDecoder::pointcloud_callback_t noop_callback()
{
  return [](nebula::drivers::NebulaPointCloudPtr, std::uint64_t) {};
}

/// Build a Seyond UDP packet whose first `sizeof(SeyondDataPacket)`
/// bytes are a valid header carrying the supplied magic / type / size /
/// timestamp, padded out with zero body bytes up to `body_size`.
std::vector<std::uint8_t> make_packet(
  std::uint16_t magic, std::uint8_t type, double ts_us, std::size_t body_size,
  std::uint32_t declared_size_override = 0)
{
  constexpr auto header_size = sizeof(nebula::drivers::SeyondDataPacket);
  std::vector<std::uint8_t> bytes(header_size + body_size, 0);

  nebula::drivers::SeyondDataPacket header{};
  header.common.magic_number = magic;
  header.common.size = declared_size_override != 0
                         ? declared_size_override
                         : static_cast<std::uint32_t>(header_size + body_size);
  header.common.ts_start_us = ts_us;
  header.type = type;
  std::memcpy(bytes.data(), &header, header_size);
  return bytes;
}

constexpr std::uint16_t k_magic = nebula::drivers::detail::seyond_data_packet_magic;
constexpr std::uint8_t k_robinw_angle_hv = nebula::drivers::detail::robinw_angle_hv_table_type;
constexpr std::uint8_t k_robine1x_angle_hv = nebula::drivers::detail::robine1x_angle_hv_table_type;

}  // namespace

TEST(SeyondDecoderWrapper, ConstructsWithoutCalibration)
{
  EXPECT_NO_THROW(SeyondDecoder decoder(make_robin_w_config(), noop_callback()));
}

TEST(SeyondDecoderWrapper, DetectsAngleHvByMagicAndType)
{
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  const auto packet = make_packet(k_magic, k_robinw_angle_hv, 1234.0, /*body_size=*/128);

  const auto result = decoder.unpack(packet);

  EXPECT_EQ(result.points_unpacked, 0U);
  EXPECT_FALSE(result.scan_complete);
  // ts_start_us=1234 microseconds -> 1_234_000 nanoseconds.
  EXPECT_EQ(result.sensor_timestamp_ns, 1234000ULL);
}

TEST(SeyondDecoderWrapper, DetectsAllFourAngleHvTypeCodes)
{
  for (std::uint8_t type :
       {nebula::drivers::detail::robinw_angle_hv_table_type,
        nebula::drivers::detail::robine1x_angle_hv_table_type,
        nebula::drivers::detail::hummingbird_angle_hv_table_type,
        nebula::drivers::detail::robine2x_angle_hv_table_type}) {
    SeyondDecoder decoder(make_robin_w_config(), noop_callback());
    const auto packet = make_packet(k_magic, type, 0.0, /*body_size=*/64);
    const auto result = decoder.unpack(packet);
    EXPECT_EQ(result.points_unpacked, 0U) << "type=" << static_cast<int>(type);
    EXPECT_FALSE(result.scan_complete) << "type=" << static_cast<int>(type);
  }
}

TEST(SeyondDecoderWrapper, IgnoresPacketSmallerThanHeader)
{
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  std::vector<std::uint8_t> tiny(10, 0);
  // Wrapper does not recognize as angle_hv (too small) -> forwards to
  // inner, which also rejects size < SeyondDataPacket. No crash, no
  // exception.
  EXPECT_NO_THROW({
    const auto result = decoder.unpack(tiny);
    EXPECT_EQ(result.points_unpacked, 0U);
    EXPECT_FALSE(result.scan_complete);
  });
}

TEST(SeyondDecoderWrapper, IgnoresWrongMagic)
{
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  // Wrong magic -> wrapper forwards; nebula's inner unpack returns
  // {0, 0, false} on magic mismatch.
  const auto packet = make_packet(/*magic=*/0xDEAD, k_robinw_angle_hv, 0.0, /*body_size=*/64);
  const auto result = decoder.unpack(packet);
  EXPECT_EQ(result.points_unpacked, 0U);
  EXPECT_FALSE(result.scan_complete);
}

TEST(SeyondDecoderWrapper, IgnoresNonAngleHvType)
{
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  // Correct magic but type=1 is FALCON's sphere pointcloud, not
  // angle_hv. Wrapper forwards to inner.
  const auto packet = make_packet(k_magic, /*type=*/1, 5000.0, /*body_size=*/64);
  const auto result = decoder.unpack(packet);
  EXPECT_EQ(result.points_unpacked, 0U);
  EXPECT_FALSE(result.scan_complete);
}

TEST(SeyondDecoderWrapper, MalformedAngleHvDoesNotCrash)
{
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  // declared common.size = 1 GiB -> way past actual buffer.
  // Wrapper detects angle_hv (magic+type ok), then extract fails the
  // sanity check and bails without latching.
  constexpr std::uint32_t bogus_size = 1U << 30U;
  const auto packet = make_packet(
    k_magic, k_robinw_angle_hv, 100.0, /*body_size=*/64, /*declared_size_override=*/bogus_size);

  const auto result = decoder.unpack(packet);
  EXPECT_EQ(result.points_unpacked, 0U);
  EXPECT_FALSE(result.scan_complete);
  EXPECT_EQ(result.sensor_timestamp_ns, 100000ULL);

  // Follow-up well-formed packet must still be accepted by the wrapper.
  const auto good = make_packet(k_magic, k_robinw_angle_hv, 200.0, /*body_size=*/128);
  const auto result2 = decoder.unpack(good);
  EXPECT_EQ(result2.points_unpacked, 0U);
  EXPECT_FALSE(result2.scan_complete);
  EXPECT_EQ(result2.sensor_timestamp_ns, 200000ULL);
}

TEST(SeyondDecoderWrapper, RepeatedAngleHvPacketsAreAccepted)
{
  // Black-box: subsequent angle_hv packets return the same shape; we
  // cannot directly observe `angle_hv_applied_` without test hooks.
  // The point is that the wrapper accepts the stream without crashing
  // or producing surprising side effects.
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  const auto p1 = make_packet(k_magic, k_robinw_angle_hv, 100.0, /*body_size=*/128);
  const auto p2 = make_packet(k_magic, k_robinw_angle_hv, 200.0, /*body_size=*/256);
  const auto p3 = make_packet(k_magic, k_robine1x_angle_hv, 300.0, /*body_size=*/64);

  for (const auto * pkt : {&p1, &p2, &p3}) {
    const auto result = decoder.unpack(*pkt);
    EXPECT_EQ(result.points_unpacked, 0U);
    EXPECT_FALSE(result.scan_complete);
  }
}

TEST(SeyondDecoderWrapper, AngleHvBeforeAnyPointPacket)
{
  // Regression guard for the "angle_hv arrives at packet_idx=0" case
  // (typical of RobinW). The wrapper must accept angle_hv as the very
  // first input without requiring a prior packet.
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());
  const auto packet = make_packet(k_magic, k_robinw_angle_hv, 42.0, /*body_size=*/256);

  const auto result = decoder.unpack(packet);
  EXPECT_EQ(result.points_unpacked, 0U);
  EXPECT_FALSE(result.scan_complete);
  EXPECT_EQ(result.sensor_timestamp_ns, 42000ULL);
}

TEST(SeyondDecoderWrapper, AngleHvMidStreamDoesNotCrash)
{
  // Sequence: non-angle_hv -> angle_hv (rebuild) -> non-angle_hv.
  // Validates that rebuild mid-stream leaves the wrapper in a usable
  // state for subsequent forwarding.
  SeyondDecoder decoder(make_robin_w_config(), noop_callback());

  const auto pre = make_packet(k_magic, /*type=*/7, 1000.0, /*body_size=*/64);
  const auto angle_hv = make_packet(k_magic, k_robinw_angle_hv, 2000.0, /*body_size=*/128);
  const auto post = make_packet(k_magic, /*type=*/7, 3000.0, /*body_size=*/64);

  EXPECT_NO_THROW(decoder.unpack(pre));

  const auto result_angle_hv = decoder.unpack(angle_hv);
  EXPECT_EQ(result_angle_hv.points_unpacked, 0U);
  EXPECT_FALSE(result_angle_hv.scan_complete);
  EXPECT_EQ(result_angle_hv.sensor_timestamp_ns, 2000000ULL);

  EXPECT_NO_THROW(decoder.unpack(post));
}

}  // namespace nebuladec::adapters
