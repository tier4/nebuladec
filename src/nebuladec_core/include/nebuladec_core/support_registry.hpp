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

#ifndef NEBULADEC_CORE__SUPPORT_REGISTRY_HPP_
#define NEBULADEC_CORE__SUPPORT_REGISTRY_HPP_

#include "nebuladec_core/identity.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace nebuladec
{

/// @brief Outcome of a support check against a sniffed `Identity`.
///
/// The four levels separate two orthogonal failure axes so callers can
/// report user-friendly diagnostics:
///
///   * Vendor-level: is the vendor itself in scope for nebuladec? (e.g.
///     Continental radar is out of scope because no PointCloud2 adapter
///     exists for it.)
///   * Model-level: does nebuladec's adapter set cover the specific
///     sensor model carried in the Identity? (e.g. a brand-new Hesai
///     model that the sniffer cannot yet identify produces
///     `SensorModel::UNKNOWN` and therefore fails this check.)
enum class SupportLevel : std::uint8_t {
  /// The vendor and model are both covered by a PointCloud2 adapter.
  Supported,
  /// The vendor is known, but this specific model is either unknown to
  /// the sniffer (SensorModel::UNKNOWN) or outside the list of models
  /// nebuladec decodes.
  ModelNotSupported,
  /// The vendor itself has no PointCloud2 adapter (e.g. Continental
  /// radar). Models are irrelevant at this level.
  VendorNotSupported,
  /// The sniffer could not even resolve a vendor (`Identity` missing or
  /// `vendor == UNKNOWN`).
  VendorUnknown,
};

/// @brief Human-readable answer accompanying a `SupportLevel`.
struct SupportDecision
{
  SupportLevel level{SupportLevel::VendorUnknown};
  /// Short explanation suitable for CLI messages. Empty when
  /// `level == Supported`.
  std::string reason;
};

/// @brief Single source of truth for "what vendors and models can this
/// build of nebuladec decode into PointCloud2?".
///
/// The registry is declarative: each (vendor, model) pair listed here is
/// the contract that downstream code may assume. Adapter factories and
/// dry-run reporters both consult this table rather than maintaining
/// their own copy of the support policy. When the adapter set grows to
/// cover a new model, update the table here and every consumer follows.
class SupportRegistry
{
public:
  /// Global instance. The registry is read-only after construction.
  static const SupportRegistry & instance();

  /// Main query: classify the given (possibly unset) Identity.
  SupportDecision check(const std::optional<Identity> & identity) const;

  /// Vendor-level predicate.
  bool is_vendor_supported(Vendor vendor) const;

  /// Model-level predicate (presumes vendor is already supported).
  bool is_model_supported(const Identity & identity) const;

  /// Vendors that have at least one supported model. Useful for CLI
  /// status commands that want to list what nebuladec can decode.
  const std::vector<Vendor> & supported_vendors() const { return supported_vendor_list_; }

private:
  SupportRegistry();

  struct VendorSupport
  {
    Vendor vendor;
    /// Models decoded through `SensorModel`. Empty when the vendor
    /// identifies models through a separate enum (e.g. Seyond).
    std::vector<nebula::drivers::SensorModel> sensor_models;
    /// Models decoded through `SeyondSensorModel`. Only populated for
    /// Vendor::SEYOND; other vendors leave this empty.
    std::vector<nebula::drivers::SeyondSensorModel> seyond_models;
  };

  std::vector<VendorSupport> vendors_;
  std::vector<Vendor> supported_vendor_list_;
};

}  // namespace nebuladec

#endif  // NEBULADEC_CORE__SUPPORT_REGISTRY_HPP_
