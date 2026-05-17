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
// Stress tests that verify the per-instance thread-safety contract that
// `Decoder`, the per-vendor adapters, `PacketSniffer`, `SupportRegistry`,
// `TopicMapping`, and `make_adapter` document in their headers. The
// orchestrator layer (nebuladec_bag) plans to dispatch one Decoder per
// lidar topic onto a thread pool, so the contract must hold when many
// Decoders are constructed and fed concurrently from different threads.
//
// These tests do not exercise correctness of point output; the
// per-vendor adapter tests cover that on the single-threaded path.
// They focus on:
//   * Constructing N adapters concurrently across vendors does not race.
//   * Feeding N independent Decoders concurrently does not corrupt
//     identity, lose packets, or trigger TSan / ASan reports under CI.
//   * The shared `SupportRegistry` singleton and stateless
//     `PacketSniffer` are safe to share across threads.

#include "nebuladec_adapters/decoder.hpp"

#include <nebuladec_core/identity.hpp>
#include <nebuladec_core/packet_sniffer.hpp>
#include <nebuladec_core/support_registry.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <optional>
#include <thread>
#include <vector>

namespace nebuladec
{
namespace
{

// Synthetic packets mirroring the helpers in test_decoder_integration.cpp.
// Kept private to this TU so the two test executables remain decoupled.
std::vector<std::uint8_t> seyond_stub_packet()
{
  std::vector<std::uint8_t> pkt(512, 0);
  pkt[0] = 0x6A;
  pkt[1] = 0x17;
  pkt[38] = 0x01;
  return pkt;
}

std::vector<std::uint8_t> hesai_pandar40p_packet()
{
  std::vector<std::uint8_t> pkt(1256, 0);
  pkt[0] = 0xEE;
  pkt[1] = 0xFF;
  pkt[6] = 40;
  pkt[7] = 10;
  pkt[10] = 0x37;
  return pkt;
}

std::vector<std::uint8_t> velodyne_vlp16_packet()
{
  std::vector<std::uint8_t> pkt(1206, 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xEE;
  pkt[1204] = 55;
  pkt[1205] = 0x22;
  return pkt;
}

constexpr int k_threads = 4;
constexpr int k_packets_per_thread = 200;

}  // namespace

// Sniffer is documented as stateless; sharing one instance across threads
// must be safe. Catches accidental introduction of internal mutable state.
TEST(DecoderConcurrency, SinglePacketSnifferSharedAcrossThreads)
{
  PacketSniffer sniffer;
  const auto pkt = seyond_stub_packet();
  std::vector<std::thread> workers;
  std::atomic<int> hits{0};
  workers.reserve(k_threads);
  for (int t = 0; t < k_threads; ++t) {
    workers.emplace_back([&]() {
      for (int i = 0; i < k_packets_per_thread; ++i) {
        const auto id = sniffer.identify(pkt);
        if (id && id->vendor == Vendor::SEYOND) {
          hits.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  EXPECT_EQ(hits.load(), k_threads * k_packets_per_thread);
}

// The SupportRegistry singleton is documented as immutable after first
// access. Concurrent first-access must not race (Meyers-singleton C++11
// guarantee) and concurrent reads must return identical answers.
TEST(DecoderConcurrency, SupportRegistrySharedAcrossThreads)
{
  std::vector<std::thread> workers;
  std::atomic<int> supported{0};
  workers.reserve(k_threads);
  for (int t = 0; t < k_threads; ++t) {
    workers.emplace_back([&]() {
      for (int i = 0; i < k_packets_per_thread; ++i) {
        if (SupportRegistry::instance().is_vendor_supported(Vendor::SEYOND)) {
          supported.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  EXPECT_EQ(supported.load(), k_threads * k_packets_per_thread);
}

// Each thread owns its own Decoder; the contract is that distinct
// instances do not interfere. Identity must resolve correctly per
// stream and the call must not race on the shared support registry.
TEST(DecoderConcurrency, PerThreadSeyondDecodersAreIndependent)
{
  std::vector<std::thread> workers;
  std::vector<std::optional<Identity>> ids(k_threads);
  workers.reserve(k_threads);
  const auto pkt = seyond_stub_packet();
  for (int t = 0; t < k_threads; ++t) {
    workers.emplace_back([&, t]() {
      Decoder decoder;
      for (int i = 0; i < k_packets_per_thread; ++i) {
        (void)decoder.feed(pkt, static_cast<double>(i) * 0.001);
      }
      ids[t] = decoder.identity();
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  for (int t = 0; t < k_threads; ++t) {
    ASSERT_TRUE(ids[t].has_value()) << "thread " << t;
    EXPECT_EQ(ids[t]->vendor, Vendor::SEYOND) << "thread " << t;
  }
}

// Mix vendors across threads to exercise concurrent creation of
// different adapter types (each loads a different calibration file at
// construction). Catches races in shared package-share-dir lookup and
// in upstream driver static initialisation.
TEST(DecoderConcurrency, MixedVendorDecodersAreIndependent)
{
  const std::vector<std::vector<std::uint8_t>> packets = {
    seyond_stub_packet(), hesai_pandar40p_packet(), velodyne_vlp16_packet(), seyond_stub_packet()};
  ASSERT_EQ(packets.size(), static_cast<std::size_t>(k_threads));
  const std::vector<Vendor> expected_vendors = {
    Vendor::SEYOND, Vendor::HESAI, Vendor::VELODYNE, Vendor::SEYOND};

  std::vector<std::thread> workers;
  std::vector<std::optional<Identity>> ids(k_threads);
  workers.reserve(k_threads);
  for (int t = 0; t < k_threads; ++t) {
    workers.emplace_back([&, t]() {
      Decoder decoder;
      for (int i = 0; i < k_packets_per_thread; ++i) {
        (void)decoder.feed(packets[t], static_cast<double>(i) * 0.001);
      }
      ids[t] = decoder.identity();
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  for (int t = 0; t < k_threads; ++t) {
    ASSERT_TRUE(ids[t].has_value()) << "thread " << t;
    EXPECT_EQ(ids[t]->vendor, expected_vendors[t]) << "thread " << t;
  }
}

}  // namespace nebuladec
