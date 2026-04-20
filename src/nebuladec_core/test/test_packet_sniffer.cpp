// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_core/packet_sniffer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{
namespace
{

using nebula::drivers::ReturnMode;
using nebula::drivers::SensorModel;

// Hesai packets start with SOP == 0xEEFF (uint16 LE => bytes {0xFF, 0xEE}).
// Header12B layout: sop(2), proto_major(1), proto_minor(1), reserved(2),
// laser_num(1), block_num(1), reserved(1), dis_unit(1), return_num(1), flags(1).
std::vector<std::uint8_t> make_hesai_packet(
  std::uint8_t laser_num, std::uint8_t block_num, std::uint8_t return_num, std::size_t size,
  std::uint8_t protocol_major = 0)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xEE;
  pkt[2] = protocol_major;
  pkt[6] = laser_num;
  pkt[7] = block_num;
  pkt[10] = return_num;
  return pkt;
}

// Velodyne packets are always 1206 bytes; the first block header is
// g_upper_bank (0xEEFF LE => {0xFF, 0xEE}). Byte 1204 = return_mode
// (55/56/57 = strongest/last/dual). Byte 1205 = product id.
std::vector<std::uint8_t> make_velodyne_packet(
  std::uint8_t return_mode_byte, std::uint8_t product_id)
{
  std::vector<std::uint8_t> pkt(1206, 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xEE;
  pkt[1204] = return_mode_byte;
  pkt[1205] = product_id;
  return pkt;
}

// Robosense MSOP magic varies by family.
std::vector<std::uint8_t> make_robosense_bpearl_v3_packet()
{
  std::vector<std::uint8_t> pkt(1248, 0);
  const std::uint8_t magic[8] = {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0};
  for (std::size_t i = 0; i < 8; ++i) {
    pkt[i] = magic[i];
  }
  return pkt;
}

std::vector<std::uint8_t> make_robosense_helios_packet()
{
  std::vector<std::uint8_t> pkt(1248, 0);
  const std::uint8_t magic[4] = {0x55, 0xAA, 0x05, 0x5A};
  for (std::size_t i = 0; i < 4; ++i) {
    pkt[i] = magic[i];
  }
  return pkt;
}

// Seyond data packets start with magic_number == 0x176A (uint16 LE => {0x6A, 0x17}).
// SeyondPacketCommon.lidar_type is at byte offset 15.
std::vector<std::uint8_t> make_seyond_packet(std::uint8_t lidar_type, std::size_t size = 512)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = 0x6A;
  pkt[1] = 0x17;
  pkt[15] = lidar_type;
  return pkt;
}

}  // namespace

TEST(PacketSniffer, EmptyPacketReturnsNullopt)
{
  PacketSniffer sniffer;
  std::vector<std::uint8_t> empty;
  EXPECT_FALSE(sniffer.identify(empty).has_value());
}

TEST(PacketSniffer, GarbageReturnsNullopt)
{
  PacketSniffer sniffer;
  std::vector<std::uint8_t> garbage(256, 0xAB);
  EXPECT_FALSE(sniffer.identify(garbage).has_value());
}

TEST(PacketSniffer, HesaiPandar40P)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(40, 10, 0x37 /*single strongest*/, 1256);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDAR40P);
  EXPECT_EQ(id->return_mode, ReturnMode::SINGLE_STRONGEST);
}

TEST(PacketSniffer, HesaiPandar64)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(64, 6, 0x38, 1194);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDAR64);
  EXPECT_EQ(id->return_mode, ReturnMode::SINGLE_LAST);
}

TEST(PacketSniffer, HesaiPandarXT32)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(32, 8, 0x39, 1080);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDARXT32);
  EXPECT_EQ(id->return_mode, ReturnMode::DUAL_LAST_STRONGEST);
}

TEST(PacketSniffer, HesaiPandarXT16)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(16, 8, 0x37, 1080);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDARXT16);
}

TEST(PacketSniffer, HesaiPandarXT32M)
{
  PacketSniffer sniffer;
  // XT32M has 6 blocks, 32 channels, up to 3 returns.
  auto pkt = make_hesai_packet(32, 6, 0x37, 820);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDARXT32M);
}

TEST(PacketSniffer, HesaiPandarQT64)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(64, 4, 0x38, 854);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDARQT64);
}

TEST(PacketSniffer, HesaiPandarQT128)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 1095);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDARQT128);
}

TEST(PacketSniffer, HesaiPandarAT128)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 1078);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDARAT128);
}

TEST(PacketSniffer, HesaiPandar128E3X)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 861, /*protocol_major=*/3);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDAR128_E3X);
}

TEST(PacketSniffer, HesaiPandar128E4X)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 861, /*protocol_major=*/4);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDAR128_E4X);
}

TEST(PacketSniffer, HesaiPandar128UnknownProtocolDefaultsToE4X)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 861, /*protocol_major=*/0);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDAR128_E4X);
}

TEST(PacketSniffer, HesaiPandar128UnknownSizeStaysUnresolved)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 999);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::UNKNOWN);
}

TEST(PacketSniffer, VelodyneVLP16)
{
  PacketSniffer sniffer;
  auto pkt = make_velodyne_packet(57 /*dual*/, 0x22 /*VLP16*/);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::VELODYNE);
  EXPECT_EQ(id->model, SensorModel::VELODYNE_VLP16);
  EXPECT_EQ(id->return_mode, ReturnMode::DUAL);
}

TEST(PacketSniffer, VelodyneVLP32)
{
  PacketSniffer sniffer;
  auto pkt = make_velodyne_packet(55 /*strongest*/, 0x28 /*VLP32C*/);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::VELODYNE);
  EXPECT_EQ(id->model, SensorModel::VELODYNE_VLP32);
  EXPECT_EQ(id->return_mode, ReturnMode::SINGLE_STRONGEST);
}

TEST(PacketSniffer, VelodyneVLS128)
{
  PacketSniffer sniffer;
  auto pkt = make_velodyne_packet(56 /*last*/, 0xA1 /*VLS128*/);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::VELODYNE);
  EXPECT_EQ(id->model, SensorModel::VELODYNE_VLS128);
  EXPECT_EQ(id->return_mode, ReturnMode::SINGLE_LAST);
}

TEST(PacketSniffer, VelodyneHDL32)
{
  PacketSniffer sniffer;
  auto pkt = make_velodyne_packet(57, 0x21 /*HDL32*/);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::VELODYNE);
  EXPECT_EQ(id->model, SensorModel::VELODYNE_HDL32);
}

TEST(PacketSniffer, VelodyneWrongSizeDoesNotMatch)
{
  PacketSniffer sniffer;
  std::vector<std::uint8_t> pkt(1000, 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xEE;
  // Size 1000 != 1206; this should not be classified as Velodyne. It may
  // instead be picked up by the Hesai branch if the header makes sense,
  // but here the laser_num/block_num are zero so Hesai also rejects.
  auto id = sniffer.identify(pkt);
  // Either no match, or at least not Velodyne.
  if (id.has_value()) {
    EXPECT_NE(id->vendor, Vendor::VELODYNE);
  }
}

TEST(PacketSniffer, RobosenseBpearlV3)
{
  PacketSniffer sniffer;
  auto pkt = make_robosense_bpearl_v3_packet();
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::ROBOSENSE);
  EXPECT_EQ(id->model, SensorModel::ROBOSENSE_BPEARL_V3);
}

TEST(PacketSniffer, RobosenseHeliosFamily)
{
  PacketSniffer sniffer;
  auto pkt = make_robosense_helios_packet();
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::ROBOSENSE);
  // Helios is the canonical 4-byte-magic model in Nebula's supported set.
  EXPECT_EQ(id->model, SensorModel::ROBOSENSE_HELIOS);
}

TEST(PacketSniffer, SeyondFalconK1)
{
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/0);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::SEYOND);
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::FALCON_K);
}

TEST(PacketSniffer, SeyondRobinW)
{
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/1);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::ROBIN_W);
}

TEST(PacketSniffer, SeyondFalconK2)
{
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/3);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::FALCON_K);
}

TEST(PacketSniffer, SeyondFalconIII)
{
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/4);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::FALCON_K);
}

TEST(PacketSniffer, SeyondRobinE1X)
{
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/5);  // RobinELITE
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::ROBIN_E1X);
}

TEST(PacketSniffer, SeyondHummingbird)
{
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/7);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::HUMMINGBIRD_D1);
}

TEST(PacketSniffer, SeyondUnmappedLidarType)
{
  // RobinE2X (6), RobinE (2), RobinE2 (8) are not in Nebula's
  // SeyondSensorModel; vendor is still SEYOND but seyond_model stays empty.
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/6);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::SEYOND);
  EXPECT_FALSE(id->seyond_model.has_value());
}

TEST(PacketSniffer, SeyondRejectsShortPacket)
{
  PacketSniffer sniffer;
  std::vector<std::uint8_t> pkt{0x6A, 0x17};  // magic only, no room for header
  EXPECT_FALSE(sniffer.identify(pkt).has_value());
}

}  // namespace nebuladec
