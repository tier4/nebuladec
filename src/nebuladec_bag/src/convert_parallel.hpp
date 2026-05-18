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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

namespace nebuladec::bag
{

/// Tuning knobs for the parallel pipeline. Centralised here so callers
/// can refer to a single named source of truth instead of sprinkling
/// magic numbers across the convert driver and queue templates.
///
/// `k_wait_backoff` is the maximum time a cv-wait in
/// `BoundedQueue::push`/`pop` (and the merger's `abort.wait_for`)
/// sleeps before re-checking the predicate. It exists purely as a
/// defensive cap against missed notifications -- in practice every
/// state change calls `notify_all`/`notify_one`, so the loop almost
/// always wakes earlier.
constexpr std::size_t k_default_input_queue_capacity = 256;
constexpr std::size_t k_default_output_queue_capacity = 64;
constexpr std::chrono::milliseconds k_wait_backoff{10};

/// @brief Cross-thread abort signal with first-capture exception_ptr.
///
/// Any pipeline thread that catches an exception calls `set(...)`. The
/// flag is sticky (once aborted, stays aborted). Threads waiting in
/// `BoundedQueue::push/pop` or `k_way_merge_drive` wake up promptly
/// because both poll `aborted()` while waiting on their cv. `take()`
/// returns the first captured exception so the main thread can rethrow
/// it after joining all worker threads.
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
    notify_all();
  }

  std::exception_ptr take() noexcept
  {
    std::lock_guard<std::mutex> lk(m_);
    return std::move(ep_);
  }

  /// Wake all threads waiting on this flag's internal cv. Producers
  /// (`BoundedQueue::push/close`, `Watermark::advance`) call this when
  /// they make progress so the merger can re-scan promptly.
  void notify_all() noexcept
  {
    std::lock_guard<std::mutex> lk(m_);
    cv_.notify_all();
  }

  /// Sleep up to `timeout`, returning early when `notify_all()` fires.
  /// Used by `k_way_merge_drive` to wait for source-state changes
  /// without busy-looping.
  template <class Rep, class Period>
  void wait_for(std::chrono::duration<Rep, Period> timeout) noexcept
  {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait_for(lk, timeout);
  }

private:
  std::atomic<bool> aborted_{false};
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::exception_ptr ep_;
};

/// @brief Monotonic lower-bound watermark for an output queue.
///
/// Stored as an atomic `int64_t` so the merger reads without locking.
/// Mutators (`advance`, `close`) notify the supplied `AbortFlag &` so
/// the merger thread (which sleeps on the abort cv) wakes up to
/// re-scan whenever a watermark moves. `kClosed` is the sentinel used
/// after a source signals EOF.
class Watermark
{
public:
  static constexpr std::int64_t k_initial = std::numeric_limits<std::int64_t>::min();
  static constexpr std::int64_t k_closed = std::numeric_limits<std::int64_t>::max();

  Watermark() = default;
  explicit Watermark(AbortFlag & abort) : abort_(&abort) {}

  [[nodiscard]] std::int64_t load() const noexcept { return v_.load(std::memory_order_acquire); }

  /// Monotonic advance: ignored when `to <= current`.
  void advance(std::int64_t to) noexcept
  {
    std::int64_t cur = v_.load(std::memory_order_relaxed);
    while (to > cur) {
      if (v_.compare_exchange_weak(cur, to, std::memory_order_acq_rel, std::memory_order_relaxed)) {
        break;
      }
    }
    if (abort_ != nullptr) {
      abort_->notify_all();
    }
  }

  /// Mark this source as finished. Equivalent to `advance(k_closed)`.
  void close() noexcept { advance(k_closed); }

private:
  std::atomic<std::int64_t> v_{k_initial};
  AbortFlag * abort_{nullptr};
};

/// @brief Bounded thread-safe blocking queue used between pipeline
/// stages.
///
/// One BoundedQueue per topic. Producers block on `push()` when full;
/// consumers block on `pop()` when empty. Both unblock promptly on
/// `close()` or abort (waits use a short timeout so `abort.aborted()`
/// is rechecked at least every ~10 ms even without an explicit
/// notify).
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
      // CP.42: the surrounding loop is the wait predicate. The
      // `k_wait_backoff` timeout caps how long a missed notification
      // can stall shutdown (abort sets `aborted()` true but does not
      // directly notify this queue's cv).
      while (q_.size() >= capacity_ && !closed_ && !abort_.aborted()) {
        cv_not_full_.wait_for(lk, k_wait_backoff);
      }
      if (closed_ || abort_.aborted()) {
        return false;
      }
      q_.push_back(std::move(item));
    }
    cv_not_empty_.notify_one();
    abort_.notify_all();  // wake the merger
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
      // Prefer pop over abort/EOF when the queue still has items so
      // shutdown drains gracefully.
      out = std::move(q_.front());
      q_.pop_front();
      cv_not_full_.notify_one();
      return true;
    }
    return false;  // EOF (closed + drained) or aborted
  }

  /// Non-blocking pop.
  bool try_pop(T & out)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) {
      return false;
    }
    out = std::move(q_.front());
    q_.pop_front();
    cv_not_full_.notify_one();
    return true;
  }

  /// Non-blocking peek (copies the head's value).
  bool peek(T & out) const
  {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) {
      return false;
    }
    out = q_.front();
    return true;
  }

  /// Mark closed.
  void close()
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      closed_ = true;
    }
    cv_not_empty_.notify_all();
    cv_not_full_.notify_all();
    abort_.notify_all();
  }

  [[nodiscard]] bool is_closed() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return closed_;
  }

  [[nodiscard]] bool empty_and_closed() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return q_.empty() && closed_;
  }

  [[nodiscard]] std::size_t size() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return q_.size();
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

/// @brief One participant in the K-way merge.
///
/// Modeled as four callbacks so the merger stays bag-agnostic and can
/// be unit-tested with synthetic sources (see
/// `test/test_convert_parallel_kway_merge.cpp`). The driver providing
/// these callbacks is responsible for thread-safe access to the
/// underlying queue + watermark.
struct OutputSource
{
  std::function<bool(std::int64_t & out_stamp)> peek;
  std::function<bool()> is_eof;
  std::function<std::int64_t()> watermark;
  std::function<void()> consume_head;
};

/// Drive a K-way merge over `sources`, popping the smallest-stamp
/// head and routing it through `consume_head`. Loops until every
/// source is empty + EOF (clean exit) or `abort.aborted()` becomes
/// true (error path).
///
/// Watermark contract: when no non-empty source has a stamp `<=` the
/// minimum watermark of all empty-but-not-EOF sources, the merger
/// waits on `abort.wait_for(...)` until a producer notifies.
void k_way_merge_drive(std::vector<OutputSource> & sources, AbortFlag & abort);

}  // namespace nebuladec::bag

#endif  // CONVERT_PARALLEL_HPP_
