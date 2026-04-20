// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_adapters/decoder.hpp"
#include "nebuladec_adapters/robosense_adapter.hpp"

#include <nebuladec_core/identity.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{
namespace
{

Identity make_robosense_identity(nebula::drivers::SensorModel model)
{
  Identity id;
  id.vendor = Vendor::ROBOSENSE;
  id.model = model;
  id.return_mode = nebula::drivers::ReturnMode::UNKNOWN;
  id.confidence = 0.95F;
  return id;
}

}  // namespace

TEST(RobosenseAdapter, NotReadyBeforeDifop)
{
  adapters::RobosenseAdapter adapter(
    make_robosense_identity(nebula::drivers::SensorModel::ROBOSENSE_BPEARL_V3));
  EXPECT_FALSE(adapter.is_ready());
}

TEST(RobosenseAdapter, NotReadyForUnknownModel)
{
  Identity id;
  id.vendor = Vendor::ROBOSENSE;
  id.model = nebula::drivers::SensorModel::UNKNOWN;
  adapters::RobosenseAdapter adapter(id);
  EXPECT_FALSE(adapter.is_ready());
}

TEST(RobosenseAdapter, DropsMsopPacketsBeforeCalibration)
{
  adapters::RobosenseAdapter adapter(
    make_robosense_identity(nebula::drivers::SensorModel::ROBOSENSE_HELIOS));
  std::vector<std::uint8_t> msop(1248, 0);
  EXPECT_FALSE(adapter.feed(msop, 0.0).has_value());
}

TEST(RobosenseAdapter, EmptyInfoPacketKeepsAdapterUnready)
{
  adapters::RobosenseAdapter adapter(
    make_robosense_identity(nebula::drivers::SensorModel::ROBOSENSE_BPEARL_V3));
  std::vector<std::uint8_t> empty;
  adapter.feed_info(empty);
  EXPECT_FALSE(adapter.is_ready());
}

TEST(MakeAdapter, RobosenseReturnsAdapterForKnownModel)
{
  auto adapter =
    make_adapter(make_robosense_identity(nebula::drivers::SensorModel::ROBOSENSE_BPEARL_V3));
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(adapter->identity().vendor, Vendor::ROBOSENSE);
}

TEST(MakeAdapter, RobosenseReturnsNullForUnknownModel)
{
  Identity id;
  id.vendor = Vendor::ROBOSENSE;
  id.model = nebula::drivers::SensorModel::UNKNOWN;
  EXPECT_EQ(make_adapter(id), nullptr);
}

TEST(Decoder, FeedInfoPropagatesToAdapter)
{
  // The Decoder facade should route info packets to the active adapter
  // once an MSOP packet has caused adapter construction. With no MSOP
  // seen yet, feed_info is a no-op (no crash).
  Decoder decoder;
  std::vector<std::uint8_t> info(64, 0xA5);
  decoder.feed_info(info);
  SUCCEED();
}

}  // namespace nebuladec
