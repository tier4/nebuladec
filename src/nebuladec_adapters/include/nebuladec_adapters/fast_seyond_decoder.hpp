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

#ifndef NEBULADEC_ADAPTERS__FAST_SEYOND_DECODER_HPP_
#define NEBULADEC_ADAPTERS__FAST_SEYOND_DECODER_HPP_

// Drop-in optimized re-implementation of `nebula::drivers::SeyondDecoder`
// for the FALCON_K and ROBIN_W (non-compact) hot paths.
//
// Why this exists: profiling `nebuladec convert` on a real Seyond capture
// showed `SeyondDecoder::unpack` consumes ~29% of wall time, dominated by
// per-point double-precision trig, per-scan PointCloud allocations with no
// reserve, and redundant atan2/sqrt work in the point-emit helper. Rather
// than fork upstream Nebula to fix this, we implement a parallel decoder
// here that:
//
//   * reuses Nebula's public packet/calibration/config types,
//   * preserves the SeyondDecoder::unpack contract (same signature, same
//     return value, same pointcloud-callback semantics),
//   * preserves point output up to floating-point precision,
//   * skips work the upstream decoder does generically but that this code
//     path can hoist or elide (full-circle FOV, double->float casts,
//     un-reserved PointCloud allocations, separate sin/cos calls, ...).
//
// FastSeyondDecoder is intentionally limited in scope. `supports()` returns
// true only for the two sensor models exercised in the workflow this
// decoder was written for. SeyondAdapter falls back to the upstream
// nebula::drivers::SeyondDecoder for every other model and for the
// compact-packet payload variants of Robin W, which are not yet covered.

#include <nebula_core_common/point_types.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>

#include <cstdint>
#include <functional>
#include <vector>

namespace nebuladec::adapters
{

class FastSeyondDecoder
{
public:
  using pointcloud_callback_t =
    std::function<void(nebula::drivers::NebulaPointCloudPtr, std::uint64_t)>;

  FastSeyondDecoder(
    const nebula::drivers::SeyondSensorConfiguration & config, pointcloud_callback_t pointcloud_cb,
    const nebula::drivers::SeyondCalibrationData & calibration =
      nebula::drivers::SeyondCalibrationData{});

  /// @return true if this decoder handles the given model. The caller
  /// (typically SeyondAdapter) is expected to construct the upstream
  /// nebula decoder instead when this returns false.
  static bool supports(nebula::drivers::SeyondSensorModel model) noexcept;

  /// Bit-for-bit compatible with the upstream SeyondDecoder::unpack
  /// contract: returns sensor timestamp / points unpacked / scan-complete.
  nebula::drivers::SeyondPacketDecodeResult unpack(const std::vector<std::uint8_t> & packet_data);

private:
  void parse_falcon_k(const void * packet_bytes);
  void parse_robin_w_en(const void * packet_bytes);

  void emit_point(
    float x, float y, float z, std::uint8_t intensity, std::uint16_t channel,
    std::uint32_t timestamp_ns);

  void ensure_scan_cloud();

  nebula::drivers::SeyondSensorConfiguration config_;
  nebula::drivers::SeyondCalibrationData calibration_;
  pointcloud_callback_t pointcloud_callback_;

  nebula::drivers::NebulaPointCloudPtr current_scan_cloud_;
  std::uint64_t current_scan_frame_idx_{0};
  std::uint64_t current_scan_start_timestamp_ns_{0};
  std::uint64_t last_packet_start_timestamp_ns_{0};
  bool has_current_scan_frame_{false};

  // Hoisted FOV gating: SeyondAdapter configures full-circle FOV
  // ({0.0F, 0.0F}) for offline decode, so the per-point angle_is_between
  // branches in the upstream decoder are always false-positive guards.
  // Caching the predicate lets the hot loop skip the check entirely while
  // still honoring a non-default config.
  bool azimuth_full_circle_{true};
  bool elevation_full_circle_{true};

  // Robin W calibration is layout-dependent on the angle table. Cached
  // once so the inner loop never re-validates the table size.
  bool robin_w_use_calibration_{false};

  // Initial reserve on every new scan cloud; an under-estimate just
  // triggers vector growth, never overflow.
  std::size_t scan_reserve_capacity_{0};
};

}  // namespace nebuladec::adapters

#endif  // NEBULADEC_ADAPTERS__FAST_SEYOND_DECODER_HPP_
