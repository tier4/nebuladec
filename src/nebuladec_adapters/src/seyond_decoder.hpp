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

#ifndef SEYOND_DECODER_HPP_
#define SEYOND_DECODER_HPP_

// NOTE: This header lives under `src/` (not the public `include/` tree)
// so it stays an implementation detail of nebuladec_adapters. The wrapped
// type `nebula::drivers::SeyondDecoder` shares its class name; consumers
// outside this package never need to disambiguate -- `SeyondAdapter` is
// the only public surface.
//
// This class composes `nebula::drivers::SeyondDecoder` and learns the
// `angle_hv_table` calibration directly from the UDP stream. Nebula's
// upstream decoder rejects angle_hv_table packets (type=100/101/103/104)
// as `supported_layout = false`, which causes offline replay without an
// external `calibration_file` to silently fall back to degraded decoding
// (compact formats collapse 8 channels into one). This wrapper:
//
//   1. Inspects every incoming packet before forwarding it.
//   2. When an angle_hv_table packet appears, strips the 70-byte
//      `SeyondDataPacket` header and feeds the body into a fresh
//      `nebula::drivers::SeyondCalibrationData`.
//   3. Rebuilds the inner decoder *once* with the populated calibration.
//
// Subsequent angle_hv_table packets are silently ignored -- the table
// does not change mid-stream.

#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebuladec::adapters
{

class SeyondDecoder
{
public:
  using pointcloud_callback_t = nebula::drivers::SeyondDecoder::pointcloud_callback_t;

  SeyondDecoder(
    nebula::drivers::SeyondSensorConfiguration config, pointcloud_callback_t pointcloud_cb);

  SeyondDecoder(const SeyondDecoder &) = delete;
  SeyondDecoder & operator=(const SeyondDecoder &) = delete;
  SeyondDecoder(SeyondDecoder &&) = delete;
  SeyondDecoder & operator=(SeyondDecoder &&) = delete;
  ~SeyondDecoder() = default;

  /// @brief Unpack a raw Seyond packet.
  ///
  /// If the packet is an angle_hv_table packet, this wrapper consumes
  /// it: the inner decoder is rebuilt (once) with the recovered
  /// calibration, and the result mirrors Nebula's rejection path --
  /// `{ts_from_header, 0, false}`. All other packets are forwarded to
  /// the inner decoder unchanged.
  nebula::drivers::SeyondPacketDecodeResult unpack(const std::vector<std::uint8_t> & packet);

private:
  nebula::drivers::SeyondSensorConfiguration config_;
  pointcloud_callback_t callback_;
  std::unique_ptr<nebula::drivers::SeyondDecoder> inner_;
  bool angle_hv_applied_{false};
};

}  // namespace nebuladec::adapters

#endif  // SEYOND_DECODER_HPP_
