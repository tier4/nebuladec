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

#ifndef CONVERT_PARALLEL_HPP_
#define CONVERT_PARALLEL_HPP_

#include "nebuladec_bag/bag_io.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <utility>

namespace nebuladec::bag
{

/// @brief Reader-side throttled progress reporter.
///
/// Counts every decoded-topic message the reader hands to a worker
/// (parallel path) or feeds to a decoder inline (sequential path), then
/// forwards a `ProgressEvent` to `on_progress` at most every ~50 ms.
/// `finalize()` always emits the final snapshot, even if it would
/// otherwise be throttled, so the bar always lands on 100%.
///
/// All public methods are thread-safe and no-op when `on_progress` is
/// empty -- `advance()` collapses to a single function-pointer check +
/// early return in that case. Callback exceptions are swallowed so a
/// UI bug cannot abort the convert pipeline.
class ProgressReporter
{
public:
  ProgressReporter(
    std::function<void(const ProgressEvent &)> on_progress, std::size_t total,
    std::chrono::milliseconds interval = std::chrono::milliseconds{50})
  : on_progress_(std::move(on_progress)),
    total_(total),
    interval_(interval),
    last_emit_(std::chrono::steady_clock::now())
  {
  }

  ProgressReporter(const ProgressReporter &) = delete;
  ProgressReporter & operator=(const ProgressReporter &) = delete;
  ProgressReporter(ProgressReporter &&) = delete;
  ProgressReporter & operator=(ProgressReporter &&) = delete;
  ~ProgressReporter() = default;

  void advance() noexcept
  {
    if (!on_progress_) {
      return;
    }
    const auto done = done_.fetch_add(1, std::memory_order_acq_rel) + 1U;
    const auto now = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lk(m_, std::try_to_lock);
    if (!lk.owns_lock()) {
      return;  // another caller is already emitting; skip this tick
    }
    if (now - last_emit_ < interval_) {
      return;
    }
    last_emit_ = now;
    emit_locked(done);
  }

  void finalize() noexcept
  {
    if (!on_progress_) {
      return;
    }
    std::lock_guard<std::mutex> lk(m_);
    emit_locked(done_.load(std::memory_order_acquire));
  }

private:
  void emit_locked(std::size_t done) noexcept
  {
    try {
      on_progress_(ProgressEvent{done, total_});
    } catch (...) {
      // Swallow: a broken UI callback must not poison the pipeline.
    }
  }

  std::function<void(const ProgressEvent &)> on_progress_;
  const std::size_t total_;
  const std::chrono::milliseconds interval_;
  std::atomic<std::size_t> done_{0};
  std::mutex m_;
  std::chrono::steady_clock::time_point last_emit_;
};

/// Tuning knobs for the parallel pipeline.
///
/// `k_wait_backoff` is the maximum time a cv-wait in
/// `BoundedQueue::push`/`pop` sleeps before re-checking its predicate.
/// It exists as a defensive cap so an abort can be observed promptly
/// even if a notify was missed.
constexpr std::size_t k_default_input_queue_capacity = 256;
constexpr std::size_t k_default_output_queue_capacity = 256;
constexpr std::chrono::milliseconds k_wait_backoff{10};

/// @brief Cross-thread abort signal with first-capture exception_ptr.
///
/// Any pipeline thread that catches an exception calls `set(...)`.
/// Threads waiting in `BoundedQueue::push/pop` poll `aborted()` while
/// waiting on their cv (via the `k_wait_backoff` timeout), so they
/// unblock within ~10 ms of an abort. `take()` returns the first
/// captured exception so the driver can rethrow it after joining all
/// worker threads.
class AbortFlag
{
public:
  AbortFlag() = default;
  ~AbortFlag() = default;
  AbortFlag(const AbortFlag &) = delete;
  AbortFlag & operator=(const AbortFlag &) = delete;
  AbortFlag(AbortFlag &&) = delete;
  AbortFlag & operator=(AbortFlag &&) = delete;

  [[nodiscard]] bool aborted() const noexcept { return aborted_.load(std::memory_order_acquire); }

  void set(std::exception_ptr ep) noexcept
  {
    bool expected = false;
    if (aborted_.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel, std::memory_order_relaxed)) {
      std::lock_guard<std::mutex> lk(m_);
      ep_ = std::move(ep);
    }
  }

  std::exception_ptr take() noexcept
  {
    std::lock_guard<std::mutex> lk(m_);
    return std::move(ep_);
  }

private:
  std::atomic<bool> aborted_{false};
  mutable std::mutex m_;
  std::exception_ptr ep_;
};

/// @brief Bounded thread-safe blocking queue used between pipeline
/// stages.
///
/// Multiple producers may push; one or more consumers may pop. Both
/// block when the queue is full / empty and unblock on `close()` or
/// abort. The `k_wait_backoff` timeout caps how long a missed
/// notification can stall shutdown.
template <class T>
class BoundedQueue
{
public:
  BoundedQueue(std::size_t capacity, AbortFlag & abort) : capacity_(capacity), abort_(abort) {}
  ~BoundedQueue() = default;

  BoundedQueue(const BoundedQueue &) = delete;
  BoundedQueue & operator=(const BoundedQueue &) = delete;
  BoundedQueue(BoundedQueue &&) = delete;
  BoundedQueue & operator=(BoundedQueue &&) = delete;

  /// Block until space is available or the queue is closed/aborted.
  /// Returns `true` when the item was accepted, `false` when refused.
  bool push(T item)
  {
    {
      std::unique_lock<std::mutex> lk(m_);
      while (q_.size() >= capacity_ && !closed_ && !abort_.aborted()) {
        cv_not_full_.wait_for(lk, k_wait_backoff);
      }
      if (closed_ || abort_.aborted()) {
        return false;
      }
      q_.push_back(std::move(item));
    }
    cv_not_empty_.notify_one();
    return true;
  }

  /// Block until an item is available, the queue is closed and
  /// drained, or aborted. Returns `true` on successful pop, `false`
  /// on EOF or abort.
  bool pop(T & out)
  {
    std::unique_lock<std::mutex> lk(m_);
    while (q_.empty() && !closed_ && !abort_.aborted()) {
      cv_not_empty_.wait_for(lk, k_wait_backoff);
    }
    if (!q_.empty()) {
      // Prefer pop over abort/EOF when items remain so shutdown drains
      // gracefully.
      out = std::move(q_.front());
      q_.pop_front();
      cv_not_full_.notify_one();
      return true;
    }
    return false;  // EOF (closed + drained) or aborted
  }

  /// Mark closed. Wakes all waiters so they observe EOF / abort.
  void close()
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      closed_ = true;
    }
    cv_not_empty_.notify_all();
    cv_not_full_.notify_all();
  }

private:
  const std::size_t capacity_;
  AbortFlag & abort_;
  mutable std::mutex m_;
  std::condition_variable cv_not_full_;
  std::condition_variable cv_not_empty_;
  std::deque<T> q_;
  bool closed_{false};
};

}  // namespace nebuladec::bag

#endif  // CONVERT_PARALLEL_HPP_
