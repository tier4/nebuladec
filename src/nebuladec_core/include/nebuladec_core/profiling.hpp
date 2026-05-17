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

#ifndef NEBULADEC_CORE__PROFILING_HPP_
#define NEBULADEC_CORE__PROFILING_HPP_

// Opt-in micro-profiler. Enabled at build time with -DNEBULADEC_PROFILE=1.
// When disabled the macros expand to nothing so Release builds are unchanged.
// When enabled, NEBULADEC_PROFILE_SCOPE("label") accumulates wall-clock
// nanoseconds and call counts per label into a process-global registry that
// is dumped to stderr from a static destructor at process exit.
//
// Intended only for one-off perf investigations; not a public API.

#ifndef NEBULADEC_PROFILE
#define NEBULADEC_PROFILE 0
#endif

#if NEBULADEC_PROFILE

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string_view>

namespace nebuladec::profiling
{

struct Counter
{
  std::atomic<std::uint64_t> ns{0};
  std::atomic<std::uint64_t> calls{0};
};

/// Register (once per label) and return a stable reference to the counter
/// for that label. Thread-safe: registration happens under a mutex on
/// first call. The returned reference stays valid for the entire lifetime
/// of the process-global registry (i.e. effectively forever for any
/// caller running before main()'s end).
[[nodiscard]] Counter & counter_for(std::string_view label);

/// RAII scope timer. Construction captures the start time; destruction
/// adds the elapsed nanoseconds and a call to the referenced Counter.
/// Non-copyable and non-movable by design: timers are strictly local to
/// the scope in which they are declared, and the static-storage Counter
/// they reference is owned by the global registry, not by the timer.
class ScopedTimer
{
public:
  explicit ScopedTimer(Counter & c) noexcept : counter_(c), start_(std::chrono::steady_clock::now())
  {
  }
  ~ScopedTimer()
  {
    const auto end = std::chrono::steady_clock::now();
    const auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count();
    counter_.ns.fetch_add(static_cast<std::uint64_t>(delta), std::memory_order_relaxed);
    counter_.calls.fetch_add(1, std::memory_order_relaxed);
  }
  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer & operator=(const ScopedTimer &) = delete;
  ScopedTimer(ScopedTimer &&) = delete;
  ScopedTimer & operator=(ScopedTimer &&) = delete;

private:
  Counter & counter_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace nebuladec::profiling

// Macros below are intentional: NEBULADEC_PROFILE_SCOPE must elide to a
// no-op when NEBULADEC_PROFILE=0 (which a constexpr/template helper
// cannot guarantee without leaving call-site cost), and it must define
// a unique per-line static Counter reference so multiple SCOPE calls in
// the same function do not collide. clang-tidy's
// cppcoreguidelines-macro-usage check is suppressed for these
// definitions only.
// NOLINTBEGIN
#define NEBULADEC_PROFILE_CONCAT_INNER(a, b) a##b
#define NEBULADEC_PROFILE_CONCAT(a, b) NEBULADEC_PROFILE_CONCAT_INNER(a, b)
#define NEBULADEC_PROFILE_COUNTER_NAME NEBULADEC_PROFILE_CONCAT(_nebuladec_prof_c_, __LINE__)
#define NEBULADEC_PROFILE_TIMER_NAME NEBULADEC_PROFILE_CONCAT(_nebuladec_prof_t_, __LINE__)
#define NEBULADEC_PROFILE_SCOPE(label)                                      \
  static ::nebuladec::profiling::Counter & NEBULADEC_PROFILE_COUNTER_NAME = \
    ::nebuladec::profiling::counter_for(label);                             \
  ::nebuladec::profiling::ScopedTimer NEBULADEC_PROFILE_TIMER_NAME          \
  {                                                                         \
    NEBULADEC_PROFILE_COUNTER_NAME                                          \
  }
// NOLINTEND

#else  // NEBULADEC_PROFILE

// NOLINTNEXTLINE
#define NEBULADEC_PROFILE_SCOPE(label) ((void)0)

#endif  // NEBULADEC_PROFILE

#endif  // NEBULADEC_CORE__PROFILING_HPP_
