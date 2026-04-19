// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_core/decoder.hpp"
#include "nebuladec_core/packet_sniffer.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace nebuladec
{

TEST(PacketSniffer, SkeletonReturnsNullopt)
{
  PacketSniffer sniffer;
  std::vector<std::uint8_t> empty;
  EXPECT_FALSE(sniffer.identify(empty).has_value());
}

TEST(Decoder, ConstructsWithoutCrashing)
{
  Decoder decoder;
  EXPECT_FALSE(decoder.identity().has_value());
}

}  // namespace nebuladec
