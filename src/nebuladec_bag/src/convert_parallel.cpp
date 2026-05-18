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

#include "convert_parallel.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace nebuladec::bag
{

void k_way_merge_drive(std::vector<OutputSource> & sources, AbortFlag & abort)
{
  while (!abort.aborted()) {
    // Scan every source once per iteration. K is typically <= ~8
    // (handful of LiDAR topics + 1 passthrough) so a linear scan is
    // cheaper than a heap.
    int best_idx = -1;
    std::int64_t best_stamp = std::numeric_limits<std::int64_t>::max();
    std::int64_t empty_min_watermark = std::numeric_limits<std::int64_t>::max();
    bool any_pending = false;

    for (std::size_t i = 0; i < sources.size(); ++i) {
      auto & src = sources[i];
      std::int64_t head_stamp = 0;
      if (src.peek(head_stamp)) {
        any_pending = true;
        if (head_stamp < best_stamp) {
          best_stamp = head_stamp;
          best_idx = static_cast<int>(i);
        }
      } else if (!src.is_eof()) {
        // Empty but not EOF: this source could still produce items.
        // Its watermark bounds the merger.
        any_pending = true;
        const auto wm = src.watermark();
        if (wm < empty_min_watermark) {
          empty_min_watermark = wm;
        }
      }
    }

    if (!any_pending) {
      // Every source is empty + EOF. Clean exit.
      return;
    }

    if (best_idx >= 0 && best_stamp <= empty_min_watermark) {
      // Safe to consume: no empty non-EOF source could yet produce a
      // stamp earlier than `best_stamp`.
      sources[static_cast<std::size_t>(best_idx)].consume_head();
      continue;
    }

    // Either:
    //   (a) every source is empty (best_idx == -1) and at least one
    //       is not EOF -- wait for a push or a close, OR
    //   (b) the smallest available head is bounded by an empty
    //       source's watermark -- wait for that watermark to advance
    //       or for the empty source to receive an item / close.
    //
    // Producers (BoundedQueue push/close, Watermark advance) notify
    // abort's cv on every state change so wait_for returns promptly.
    // `k_wait_backoff` is a defensive backstop in case a notification
    // is missed.
    abort.wait_for(k_wait_backoff);
  }
}

}  // namespace nebuladec::bag
