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

#include <nebuladec_core/identity.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{

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

}  // namespace nebuladec
