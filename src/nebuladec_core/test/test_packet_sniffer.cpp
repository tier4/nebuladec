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

// Hesai packets start with SOP 0xEEFF written big-endian => bytes
// {0xEE, 0xFF} at offsets 0 and 1. Header12B layout: sop(2),
// proto_major(1), proto_minor(1), reserved(2), laser_num(1), block_num(1),
// reserved(1), dis_unit(1), return_num(1), flags(1).
std::vector<std::uint8_t> make_hesai_packet(
  std::uint8_t laser_num, std::uint8_t block_num, std::uint8_t return_num, std::size_t size,
  std::uint8_t protocol_major = 0)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = 0xEE;
  pkt[1] = 0xFF;
  pkt[2] = protocol_major;
  pkt[6] = laser_num;
  pkt[7] = block_num;
  pkt[10] = return_num;
  return pkt;
}

// Velodyne packets are always 1206 bytes; the first block header is
// g_upper_bank (0xEEFF little-endian => bytes {0xFF, 0xEE}). Byte 1204
// = return_mode (55/56/57 = strongest/last/dual). Byte 1205 = product id.
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

// Seyond packets start with magic_number == 0x176A (uint16 LE => {0x6A, 0x17}).
// Model identification relies on two bytes: SeyondPacketCommon.lidar_type at
// offset 15, and SeyondDataPacket.type (lower 8 bits of the packed `type:8 |
// item_number:24` uint32) at offset 38. Non-data item types are rejected by
// the sniffer, so tests must set a data item_type by default.
constexpr std::uint8_t k_seyond_item_type_sphere_pointcloud = 1;
std::vector<std::uint8_t> make_seyond_packet(
  std::uint8_t lidar_type, std::uint8_t item_type = k_seyond_item_type_sphere_pointcloud,
  std::size_t size = 512)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = 0x6A;
  pkt[1] = 0x17;
  pkt[15] = lidar_type;
  pkt[38] = item_type;
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
  // Verified against Nebula's qt128 decoder_ground_truth capture: the
  // first packet is 1127 bytes with proto_major=3.
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 1127, /*protocol_major=*/3);
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

TEST(PacketSniffer, HesaiPandarOT128)
{
  // Verified against Nebula's ot128 decoder_ground_truth capture: the
  // first packet is 861 bytes with proto_major=1. Nebula maps OT128 onto
  // HESAI_PANDAR128_E4X (see Packet128E4X = Packet128E3X alias).
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(128, 2, 0x37, 861, /*protocol_major=*/1);
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
  // RobinW sensors typically emit item_type=7 (sphere) or 13 (compact);
  // both must resolve to ROBIN_W via the lidar_type byte.
  auto pkt_sphere = make_seyond_packet(/*lidar_type=*/1, /*item_type=*/7);
  auto id_sphere = sniffer.identify(pkt_sphere);
  ASSERT_TRUE(id_sphere.has_value());
  ASSERT_TRUE(id_sphere->seyond_model.has_value());
  EXPECT_EQ(*id_sphere->seyond_model, nebula::drivers::SeyondSensorModel::ROBIN_W);

  auto pkt_compact = make_seyond_packet(/*lidar_type=*/1, /*item_type=*/13);
  auto id_compact = sniffer.identify(pkt_compact);
  ASSERT_TRUE(id_compact.has_value());
  ASSERT_TRUE(id_compact->seyond_model.has_value());
  EXPECT_EQ(*id_compact->seyond_model, nebula::drivers::SeyondSensorModel::ROBIN_W);
}

TEST(PacketSniffer, SeyondRejectsStatusPackets)
{
  // Real RobinW sensors were observed emitting FalconK-flavoured status
  // packets (item_type=MESSAGE/MESSAGE_LOG/STATUS with lidar_type=0)
  // interleaved with real data packets. The sniffer must reject these
  // by item_type or it misclassifies the stream as FalconK.
  PacketSniffer sniffer;
  for (std::uint8_t non_data_type : {0, 2, 3, 4, 6, 8, 9, 100, 255}) {
    auto pkt = make_seyond_packet(/*lidar_type=*/0, non_data_type);
    EXPECT_FALSE(sniffer.identify(pkt).has_value())
      << "item_type=" << static_cast<unsigned>(non_data_type) << " should be rejected as non-data";
  }
}

TEST(PacketSniffer, SeyondHummingbirdCompact)
{
  // Hummingbird compact-pointcloud packet: lidar_type=7 + item_type=22.
  PacketSniffer sniffer;
  auto pkt = make_seyond_packet(/*lidar_type=*/7, /*item_type=*/22);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  ASSERT_TRUE(id->seyond_model.has_value());
  EXPECT_EQ(*id->seyond_model, nebula::drivers::SeyondSensorModel::HUMMINGBIRD_D1);
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
  // Magic only, not enough room for SeyondPacketCommon + data-packet
  // prefix up to the item_type byte at offset 38.
  std::vector<std::uint8_t> pkt{0x6A, 0x17};
  EXPECT_FALSE(sniffer.identify(pkt).has_value());

  // 38-byte packet stops just before the item_type byte and must also
  // be rejected -- we cannot tell data from status without it.
  std::vector<std::uint8_t> pkt38(38, 0);
  pkt38[0] = 0x6A;
  pkt38[1] = 0x17;
  EXPECT_FALSE(sniffer.identify(pkt38).has_value());
}

// --- Continental radar --------------------------------------------------
//
// ARS548 UDP payload: {service_id:BE u16 = 0, method_id:BE u16, ...}.
// SRR520 UDP payload: {can_message_id:BE u32, ...}.
// Matching requires BOTH the header value AND the exact payload size.

namespace
{
std::vector<std::uint8_t> make_ars548_packet(std::uint16_t method_id, std::size_t size)
{
  std::vector<std::uint8_t> pkt(size, 0);
  // BE u16 service_id = 0
  pkt[0] = 0x00;
  pkt[1] = 0x00;
  // BE u16 method_id
  pkt[2] = static_cast<std::uint8_t>((method_id >> 8) & 0xFF);
  pkt[3] = static_cast<std::uint8_t>(method_id & 0xFF);
  return pkt;
}

std::vector<std::uint8_t> make_srr520_packet(std::uint32_t can_id, std::size_t size)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = static_cast<std::uint8_t>((can_id >> 24) & 0xFF);
  pkt[1] = static_cast<std::uint8_t>((can_id >> 16) & 0xFF);
  pkt[2] = static_cast<std::uint8_t>((can_id >> 8) & 0xFF);
  pkt[3] = static_cast<std::uint8_t>(can_id & 0xFF);
  return pkt;
}
}  // namespace

TEST(PacketSniffer, ContinentalArs548DetectionList)
{
  PacketSniffer sniffer;
  auto pkt = make_ars548_packet(/*method_id=*/336, /*size=*/35336);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::CONTINENTAL);
  EXPECT_EQ(id->model, SensorModel::CONTINENTAL_ARS548);
}

TEST(PacketSniffer, ContinentalArs548ObjectListCommon)
{
  PacketSniffer sniffer;
  auto pkt = make_ars548_packet(/*method_id=*/329, /*size=*/9401);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::CONTINENTAL);
  EXPECT_EQ(id->model, SensorModel::CONTINENTAL_ARS548);
}

TEST(PacketSniffer, ContinentalArs548SensorStatus)
{
  PacketSniffer sniffer;
  auto pkt = make_ars548_packet(/*method_id=*/380, /*size=*/84);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::CONTINENTAL);
  EXPECT_EQ(id->model, SensorModel::CONTINENTAL_ARS548);
}

TEST(PacketSniffer, ContinentalArs548WrongSizeRejected)
{
  PacketSniffer sniffer;
  // Correct method id, wrong payload size.
  auto pkt = make_ars548_packet(/*method_id=*/336, /*size=*/1000);
  EXPECT_FALSE(sniffer.identify(pkt).has_value());
}

TEST(PacketSniffer, ContinentalSrr520DetectionHeader)
{
  PacketSniffer sniffer;
  auto pkt = make_srr520_packet(/*can_id=*/900, /*size=*/32);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::CONTINENTAL);
  EXPECT_EQ(id->model, SensorModel::CONTINENTAL_SRR520);
}

TEST(PacketSniffer, ContinentalSrr520ObjectElement)
{
  PacketSniffer sniffer;
  auto pkt = make_srr520_packet(/*can_id=*/1201, /*size=*/64);
  auto id = sniffer.identify(pkt);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::CONTINENTAL);
  EXPECT_EQ(id->model, SensorModel::CONTINENTAL_SRR520);
}

TEST(PacketSniffer, VendorHintRestrictsDetection)
{
  PacketSniffer sniffer;
  // Hesai packet, but hint says VELODYNE — sniffer should NOT return HESAI.
  auto pkt = make_hesai_packet(40, 10, 0x37, 1256);
  auto id = sniffer.identify(pkt, Vendor::VELODYNE);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::VELODYNE);
  EXPECT_EQ(id->model, SensorModel::UNKNOWN);
}

TEST(PacketSniffer, VendorHintPropagatesModelWhenMatched)
{
  PacketSniffer sniffer;
  auto pkt = make_hesai_packet(40, 10, 0x37, 1256);
  auto id = sniffer.identify(pkt, Vendor::HESAI);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::HESAI);
  EXPECT_EQ(id->model, SensorModel::HESAI_PANDAR40P);
}

TEST(PacketSniffer, VendorHintContinentalIdentifiesArs548)
{
  PacketSniffer sniffer;
  auto pkt = make_ars548_packet(/*method_id=*/336, /*size=*/35336);
  auto id = sniffer.identify(pkt, Vendor::CONTINENTAL);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->vendor, Vendor::CONTINENTAL);
  EXPECT_EQ(id->model, SensorModel::CONTINENTAL_ARS548);
}

}  // namespace nebuladec
