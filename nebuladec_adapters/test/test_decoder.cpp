// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_adapters/decoder.hpp"
#include "nebuladec_adapters/seyond_adapter.hpp"

#include <nebuladec_core/identity.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{
namespace
{

// Synthesize a Seyond data packet whose common header passes the sniffer's
// magic check. The payload remains zero-filled; SeyondDecoder will reject
// it internally (size mismatch) which is fine — the adapter's job in this
// test is to accept, forward, and not crash.
std::vector<std::uint8_t> make_seyond_stub_packet(std::size_t size = 512)
{
  std::vector<std::uint8_t> pkt(size, 0);
  pkt[0] = 0x6A;  // magic_number low byte  (uint16 LE 0x176A)
  pkt[1] = 0x17;  // magic_number high byte
  return pkt;
}

}  // namespace

TEST(Decoder, IdentityEmptyBeforeFirstPacket)
{
  Decoder decoder;
  EXPECT_FALSE(decoder.identity().has_value());
}

TEST(Decoder, UnknownPacketLeavesIdentityEmpty)
{
  Decoder decoder;
  std::vector<std::uint8_t> garbage(128, 0xAB);
  auto cloud = decoder.feed(garbage, 0.0);
  EXPECT_FALSE(cloud.has_value());
  EXPECT_FALSE(decoder.identity().has_value());
}

TEST(Decoder, SeyondPacketRoutesToSeyondAdapter)
{
  Decoder decoder;
  auto packet = make_seyond_stub_packet();
  // First feed should identify the stream even if the stub packet cannot
  // be decoded into a scan.
  auto cloud = decoder.feed(packet, 0.0);
  EXPECT_FALSE(cloud.has_value());  // stub payload will not complete a scan
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, Vendor::SEYOND);
}

TEST(Decoder, SecondSeyondPacketStaysOnSeyondAdapter)
{
  Decoder decoder;
  auto packet = make_seyond_stub_packet();
  (void)decoder.feed(packet, 0.0);
  ASSERT_TRUE(decoder.identity().has_value());
  const auto first_vendor = decoder.identity()->vendor;
  (void)decoder.feed(packet, 0.1);
  ASSERT_TRUE(decoder.identity().has_value());
  EXPECT_EQ(decoder.identity()->vendor, first_vendor);
}

TEST(MakeAdapter, ReturnsSeyondAdapterForSeyondIdentity)
{
  Identity id;
  id.vendor = Vendor::SEYOND;
  auto adapter = make_adapter(id);
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(adapter->identity().vendor, Vendor::SEYOND);
}

TEST(MakeAdapter, ReturnsNullForUnsupportedVendors)
{
  // Hesai and Velodyne are handled separately (they need a specific
  // model to be ready); see their dedicated adapter tests. M6 covers
  // Robosense.
  for (auto v : {Vendor::ROBOSENSE, Vendor::UNKNOWN}) {
    Identity id;
    id.vendor = v;
    auto adapter = make_adapter(id);
    EXPECT_EQ(adapter, nullptr) << "unexpected adapter for vendor " << to_string(v);
  }
}

TEST(SeyondAdapter, FeedEmptyPacketReturnsNullopt)
{
  Identity id;
  id.vendor = Vendor::SEYOND;
  adapters::SeyondAdapter adapter(id);
  std::vector<std::uint8_t> empty;
  EXPECT_FALSE(adapter.feed(empty, 0.0).has_value());
}

}  // namespace nebuladec
