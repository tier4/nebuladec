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
// End-to-end integration tests for the Decoder facade.
//
// These exercise the full Sniffer -> make_adapter -> AnyDecoder::feed
// path with realistic byte sequences per vendor, confirming that the
// SDK classifies, routes, and forwards packets the way each adapter
// test already verifies in isolation. Point-level regression against
// Nebula's own ROS wrapper for real rosbag fixtures is deferred to the
// bag I/O milestone, where rosbag2_cpp will be available.

#include "nebuladec_adapters/decoder.hpp"

#include <nebuladec_core/identity.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{
namespace
{

std::vector<std::uint8_t> make_hesai_pandar40p_packet()
{
  std::vector<std::uint8_t> pkt(1256, 0);
  pkt[0] = 0xEE;   // sop high  (big-endian 0xEEFF)
  pkt[1] = 0xFF;   // sop low
  pkt[6] = 40;     // laser_num
  pkt[7] = 10;     // block_num
  pkt[10] = 0x37;  // return_num == single strongest
  return pkt;
}

std::vector<std::uint8_t> make_velodyne_vlp16_packet()
{
  std::vector<std::uint8_t> pkt(1206, 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xEE;
  pkt[1204] = 55;    // single strongest
  pkt[1205] = 0x22;  // VLP16 product id
  return pkt;
}

std::vector<std::uint8_t> make_seyond_stub_packet(std::size_t size = 512)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = 0x6A;   // magic_number low  (uint16 LE == 0x176A)
  pkt[1] = 0x17;   // magic_number high
  pkt[38] = 0x01;  // type:8 == sphere_pointcloud (SeyondDataPacket data frame)
  return pkt;
}

std::vector<std::uint8_t> make_robosense_helios_msop_packet()
{
  std::vector<std::uint8_t> pkt(1248, 0);
  const std::uint8_t magic[4] = {0x55, 0xAA, 0x05, 0x5A};
  for (std::size_t i = 0; i < 4; ++i) {
    pkt[i] = magic[i];
  }
  return pkt;
}

std::vector<std::uint8_t> make_robosense_bpearl_v3_msop_packet()
{
  std::vector<std::uint8_t> pkt(1248, 0);
  const std::uint8_t magic[8] = {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0};
  for (std::size_t i = 0; i < 8; ++i) {
    pkt[i] = magic[i];
  }
  return pkt;
}

}  // namespace

// --- Hesai --------------------------------------------------------------

TEST(DecoderIntegration, HesaiPandar40PIsIdentifiedAndRouted)
{
  Decoder decoder;
  (void)decoder.feed(make_hesai_pandar40p_packet(), 0.0);
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::HESAI);
  EXPECT_EQ(decoder.identity()->model, nebula::drivers::SensorModel::HESAI_PANDAR40P);
}

TEST(DecoderIntegration, HesaiStreamRetainsIdentityAcrossPackets)
{
  Decoder decoder;
  const auto pkt = make_hesai_pandar40p_packet();
  for (int i = 0; i < 16; ++i) {
    (void)decoder.feed(pkt, static_cast<double>(i) * 0.1);
  }
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::HESAI);
}

// --- Velodyne -----------------------------------------------------------

TEST(DecoderIntegration, VelodyneVLP16IsIdentifiedAndRouted)
{
  Decoder decoder;
  (void)decoder.feed(make_velodyne_vlp16_packet(), 0.0);
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::VELODYNE);
  EXPECT_EQ(decoder.identity()->model, nebula::drivers::SensorModel::VELODYNE_VLP16);
}

// --- Seyond -------------------------------------------------------------

TEST(DecoderIntegration, SeyondIsIdentifiedAndRouted)
{
  Decoder decoder;
  (void)decoder.feed(make_seyond_stub_packet(), 0.0);
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::SEYOND);
}

// --- Robosense (identified, not decoded) --------------------------------

TEST(DecoderIntegration, RobosenseHeliosMsopIsIdentifiedButNotDecoded)
{
  // Robosense streams are sniffed for vendor and model, but
  // nebuladec_adapters intentionally does not provide a PointCloud2
  // adapter for them, so feed() never produces a cloud.
  Decoder decoder;
  for (int i = 0; i < 8; ++i) {
    auto cloud = decoder.feed(make_robosense_helios_msop_packet(), 0.0);
    EXPECT_FALSE(cloud.has_value());
  }
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::ROBOSENSE);
  EXPECT_EQ(decoder.identity()->model, nebula::drivers::SensorModel::ROBOSENSE_HELIOS);
}

TEST(DecoderIntegration, RobosenseBpearlV3MsopIsIdentified)
{
  Decoder decoder;
  (void)decoder.feed(make_robosense_bpearl_v3_msop_packet(), 0.0);
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::ROBOSENSE);
  EXPECT_EQ(decoder.identity()->model, nebula::drivers::SensorModel::ROBOSENSE_BPEARL_V3);
}

// --- Robustness ---------------------------------------------------------

TEST(DecoderIntegration, GarbagePacketsLeaveDecoderUninitialised)
{
  Decoder decoder;
  std::vector<std::uint8_t> garbage(256, 0xAB);
  for (int i = 0; i < 32; ++i) {
    auto cloud = decoder.feed(garbage, 0.0);
    EXPECT_FALSE(cloud.has_value());
  }
  EXPECT_FALSE(decoder.identity().has_value());
}

TEST(DecoderIntegration, MixedVendorStreamLocksOnFirstMatch)
{
  // Once a vendor is chosen, subsequent packets from a different vendor
  // are still forwarded to the selected adapter (which will likely drop
  // them). The identity must not flap on every packet.
  Decoder decoder;
  (void)decoder.feed(make_seyond_stub_packet(), 0.0);
  ASSERT_TRUE(decoder.identity().has_value());
  const auto first = decoder.identity()->vendor;
  (void)decoder.feed(make_hesai_pandar40p_packet(), 0.1);
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, first);
}

}  // namespace nebuladec
