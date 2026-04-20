// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "nebuladec_adapters/decoder.hpp"
#include "nebuladec_adapters/hesai_adapter.hpp"

#include <nebuladec_core/identity.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace nebuladec
{
namespace
{

Identity make_hesai_identity(nebula::drivers::SensorModel model)
{
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = model;
  id.return_mode = nebula::drivers::ReturnMode::SINGLE_STRONGEST;
  id.confidence = 0.9F;
  return id;
}

}  // namespace

// Constructs and loads the bundled Pandar40P CSV from nebula_hesai_decoders.
TEST(HesaiAdapter, ReadyForPandar40P)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDAR40P));
  EXPECT_TRUE(adapter.is_ready());
  EXPECT_EQ(adapter.identity().model, nebula::drivers::SensorModel::HESAI_PANDAR40P);
}

TEST(HesaiAdapter, ReadyForPandarXT32)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDARXT32));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, NotReadyForUnknownModel)
{
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = nebula::drivers::SensorModel::UNKNOWN;
  adapters::HesaiAdapter adapter(id);
  EXPECT_FALSE(adapter.is_ready());
}

TEST(HesaiAdapter, EmptyPacketDoesNotProduceCloud)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDAR40P));
  ASSERT_TRUE(adapter.is_ready());
  std::vector<std::uint8_t> empty;
  EXPECT_FALSE(adapter.feed(empty, 0.0).has_value());
}

TEST(MakeAdapter, HesaiReturnsAdapterForKnownModel)
{
  auto adapter = make_adapter(make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDAR64));
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(adapter->identity().vendor, Vendor::HESAI);
}

TEST(MakeAdapter, HesaiReturnsNullForUnknownModel)
{
  Identity id;
  id.vendor = Vendor::HESAI;
  id.model = nebula::drivers::SensorModel::UNKNOWN;
  EXPECT_EQ(make_adapter(id), nullptr);
}

}  // namespace nebuladec
