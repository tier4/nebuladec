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

TEST(MakeAdapter, ReturnsNullForUnknownVendor)
{
  Identity id;
  id.vendor = Vendor::UNKNOWN;
  EXPECT_EQ(make_adapter(id), nullptr);
}

TEST(Decoder, MinPointsDefaultIs1024)
{
  Decoder decoder;
  EXPECT_EQ(decoder.min_points(), 1024U);
}

TEST(Decoder, MinPointsSetterRoundTrips)
{
  Decoder decoder;
  decoder.set_min_points(0);
  EXPECT_EQ(decoder.min_points(), 0U);
  decoder.set_min_points(10000);
  EXPECT_EQ(decoder.min_points(), 10000U);
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
