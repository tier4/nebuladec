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

// The remaining models route through `AcceleratedHesaiDriver` after the
// template was extended to cover every `HesaiSensor<PacketT>` family member.
// Each test verifies that the per-model bundled calibration loads cleanly and
// that `AcceleratedHesaiDecoder<SensorT>` instantiates against the real
// configuration --- a smoke check that exercises the template's compatibility
// with every concrete SensorT.

TEST(HesaiAdapter, ReadyForPandar64)
{
  adapters::HesaiAdapter adapter(make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDAR64));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, ReadyForPandarQT64)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDARQT64));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, ReadyForPandarQT128)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDARQT128));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, ReadyForPandarXT16)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDARXT16));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, ReadyForPandarXT32M)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDARXT32M));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, ReadyForPandarAT128)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDARAT128));
  EXPECT_TRUE(adapter.is_ready());
}

TEST(HesaiAdapter, ReadyForPandar128E4X)
{
  adapters::HesaiAdapter adapter(
    make_hesai_identity(nebula::drivers::SensorModel::HESAI_PANDAR128_E4X));
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
