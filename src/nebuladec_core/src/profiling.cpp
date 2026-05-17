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

#include "nebuladec_core/profiling.hpp"

#if NEBULADEC_PROFILE

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace nebuladec::profiling
{

namespace
{

class Registry
{
public:
  Registry() = default;
  // The destructor dumps the table. Doing it here (rather than in a
  // separate Dumper static) avoids the static-destruction-order fiasco:
  // a separate Dumper that lazy-constructs the Registry on first use
  // would always be destroyed *after* the registry, leaving the dumper
  // walking freed entries.
  ~Registry()
  {
    const std::lock_guard<std::mutex> lock(mtx_);
    if (entries_.empty()) {
      return;
    }
    std::vector<std::pair<std::string, Counter *>> snapshot;
    snapshot.reserve(entries_.size());
    for (auto & e : entries_) {
      snapshot.emplace_back(e.first, e.second.get());
    }
    std::sort(snapshot.begin(), snapshot.end(), [](const auto & a, const auto & b) {
      return a.second->ns.load() > b.second->ns.load();
    });
    std::fprintf(stderr, "\n=== nebuladec profiling ===\n");
    std::fprintf(stderr, "%-40s %14s %18s %14s\n", "label", "calls", "total_ns", "ns_per_call");
    for (const auto & [label, c] : snapshot) {
      const std::uint64_t ns = c->ns.load();
      const std::uint64_t calls = c->calls.load();
      const std::uint64_t per = calls == 0 ? std::uint64_t{0} : ns / calls;
      std::fprintf(
        stderr, "%-40s %14" PRIu64 " %18" PRIu64 " %14" PRIu64 "\n", label.c_str(), calls, ns, per);
    }
    std::fprintf(stderr, "===========================\n");
  }

  // CG C.21 Rule of Five: holding a mutex is non-copyable / non-movable;
  // make that explicit so future maintainers cannot accidentally rely on
  // an implicitly deleted operation.
  Registry(const Registry &) = delete;
  Registry & operator=(const Registry &) = delete;
  Registry(Registry &&) = delete;
  Registry & operator=(Registry &&) = delete;

  [[nodiscard]] Counter & lookup_or_insert(std::string_view label)
  {
    const std::lock_guard<std::mutex> lock(mtx_);
    for (auto & e : entries_) {
      if (e.first == label) {
        return *e.second;
      }
    }
    entries_.emplace_back(std::string{label}, std::make_unique<Counter>());
    return *entries_.back().second;
  }

private:
  std::mutex mtx_;
  std::vector<std::pair<std::string, std::unique_ptr<Counter>>> entries_;
};

Registry & registry()
{
  static Registry r;
  return r;
}

}  // namespace

Counter & counter_for(std::string_view label)
{
  return registry().lookup_or_insert(label);
}

}  // namespace nebuladec::profiling

#endif  // NEBULADEC_PROFILE
