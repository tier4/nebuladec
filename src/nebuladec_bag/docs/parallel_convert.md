# Parallel `convert()` design

**Status:** Proposed (design memo — not yet implemented)

This memo captures the design discussion for parallelizing
`nebuladec::bag::convert()`. The current implementation is a single
thread that drives `rosbag2_cpp::Reader::has_next()` in a loop, calls
`Decoder::feed()` for every packet, and writes both decoded
`PointCloud2` clouds and passthrough messages through a single
`rosbag2_cpp::Writer` on the same thread.

See `src/bag_io.cpp:1074-1106` for the present main loop.

## Goals

- Decode multiple LiDAR topics concurrently when the bag carries
  several packet topics.
- Preserve the existing output guarantees:
  - Same set of output topics and message contents.
  - Same `log_time` ordering across the output bag, so `ros2 bag play`
    behaves identically.
  - Identical `ConvertResult` accounting (packets per topic,
    `clouds_written`, identities, passthrough list).
- Stay within the per-instance thread-safety contract documented in
  [`nebuladec_adapters/README.md`](../../nebuladec_adapters/README.md#thread-safety-contract):
  one `Decoder` per stream, no shared mutation.

## Non-goals

- Splitting a single topic across multiple workers. `Decoder::feed()`
  must be called in monotonic packet order on a single thread per
  instance.
- Parallelizing the writer. `rosbag2_cpp::Writer` and
  `mcap::McapWriter` are single-thread-write APIs; serializing through
  one writer thread is intentional.
- Reordering messages across `log_time`. The output bag must remain a
  byte-faithful, time-ordered superset of the inputs (clouds
  substituted for packets where mapped).

## Architecture

Three-stage pipeline with a worker pool in the middle stage.

```text
[Reader thread] ─┬─→ [worker_A] ─→ out_queue_A ─┐
                 ├─→ [worker_B] ─→ out_queue_B ─┤  K-way merge
                 ├─→ [worker_C] ─→ out_queue_C ─┤  (stamp-ordered)
                 └────────────────→ out_queue_PT ┘     │
                       (passthrough, no decode)        ▼
                                                  [Writer thread]
```

- **Reader thread (1):** Pulls bag messages from `rosbag2_cpp::Reader`
  in `log_time` order. For each packet topic, it pushes
  `(stamp_ns, raw_bytes)` into that topic's per-topic input queue.
  Passthrough messages are pushed directly into a shared passthrough
  output queue. The reader also updates a per-queue **watermark**
  (`= last dispatched log_time`) so the merger knows how far each
  source has progressed even when no output has been produced yet.

- **Worker pool (N threads):** Each topic is owned by exactly one
  worker for the duration of `convert()`. The worker pops packet
  messages from its topic's input queue, calls
  `packet_source->extract()`, feeds bytes via `Decoder::feed()`, and
  pushes resulting `(stamp_ns, NebulaPointCloudPtr)` tuples into the
  topic's output queue. On EOF it also calls `Decoder::flush()`. The
  worker is naturally monotonic in `stamp_ns` because its input is.

- **Writer thread (1):** Performs a K-way merge over all output queues
  (per-topic decoded clouds + passthrough), popping the smallest-stamp
  head and writing it via `rosbag2_cpp::Writer` (or
  `mcap::McapWriter` on the schema-forwarding fast path). When a queue
  is empty, the writer uses that queue's watermark as a lower-bound
  guarantee so progress on other queues is not blocked.

### Why per-source FIFO + K-way merge (not a single priority queue)

- Each producer (reader for passthrough, worker for a topic) is
  monotonic in `stamp_ns` because its input is. Per-source FIFO
  preserves that ordering for free.
- K ≤ ~8 in practice (typical bag has a handful of LiDAR topics plus
  passthrough). A linear scan over K heads to pick the min is cheap
  per message and avoids the `O(log N)` insertion cost of a global
  priority queue.
- Watermarks naturally attach per-source; a single global queue would
  still need per-source watermarks to avoid head-of-line blocking, so
  it would not simplify the design.

### Watermarks

Without watermarks, the merger would stall whenever a topic's worker
buffered packets but had not yet emitted a cloud (Hesai 10 Hz ≈ 100 ms
of buffering before the first scan completes). The watermark scheme:

- **Reader updates** the input-queue watermark to the dispatched
  packet's `log_time` at dispatch time. This propagates as the
  output-queue watermark once the worker has consumed up to that
  packet, OR — equivalently and simpler — the reader directly bumps
  the **output-queue** watermark since the worker can never produce a
  cloud with `log_time` earlier than the latest packet it received.
- **Writer rule:** to pop the smallest head among non-empty queues, it
  must verify that no empty queue has `watermark < candidate_stamp`.
  If such a queue exists and is not yet EOF, the writer waits on a
  condition variable.
- **EOF:** when the reader finishes, it signals EOF on every input
  queue, workers flush their decoders, and each output queue is then
  marked EOF with `watermark = INT64_MAX`. The merger drains until all
  queues are simultaneously empty + EOF.

### Ordering key

The merger orders by **input-bag `log_time`**, matching the current
`write_serialized(..., stamp_ns, stamp_ns)` semantics in `bag_io.cpp`.
For decoded clouds, the per-cloud `stamp_ns` is the `log_time` of the
last packet that completed the scan — not the sensor-derived sample
time inside `NebulaPoint`. This preserves the existing output bag's
log-time monotonicity, which `ros2 bag play` relies on.

### Backpressure

- Output queues are **bounded** (suggested default: 64 entries). When
  a queue is full, the producer (worker or reader) blocks on push.
- This bounds memory in passthrough-heavy bags where the writer might
  be slower than the reader (e.g., MCAP with zstd compression on
  spinning rust).
- The input queues per topic should also be bounded (suggested: 256
  packets) to bound memory when one worker is slower than the
  reader.

### Schema-forwarding fast path

Compatible with parallelization unchanged. The writer thread chooses
between `rosbag2_cpp::Writer` and `mcap::McapWriter` exactly as today
(see `bag_io.cpp` end-to-end flow step 5); the merger feeds whichever
writer was selected. Schema registration happens once on the writer
thread before the merge loop starts.

## Speedup model

Total wall-clock time approximates the slowest pipeline stage:

```text
T_total ≈ max(T_read, T_decode_parallel, T_write)
```

Where `T_decode_parallel ≈ sum_over_topics(T_decode_topic) /
min(N, K)`, K being the number of packet topics with non-trivial
volume.

Two independent gains stack:

| Mechanism                 | What it does                             | When it matters                  |
| ------------------------- | ---------------------------------------- | -------------------------------- |
| Pipeline split (3 stages) | Overlaps read I/O, decode CPU, write I/O | Always — even at N=1             |
| Worker pool (N threads)   | Shrinks the decode stage by up to K×     | When decode is the slowest stage |

Empirically expected regimes:

| Workload                                        | Slowest stage   | Dominant gain             |
| ----------------------------------------------- | --------------- | ------------------------- |
| Many heavy LiDAR topics (Hesai AT128, VLS128)   | T_decode        | Worker pool (N×)          |
| Few packet topics + large raw-image passthrough | T_write         | Pipeline split only (~2×) |
| Slow input storage (HDD / NFS)                  | T_read          | Pipeline split only       |
| Balanced                                        | All three close | Both stack                |

## `--jobs N` semantics (proposed CLI surface)

- `--jobs 1`: legacy behavior — single thread does read, decode, write.
  Kept as the default for byte-for-byte regression assurance during
  rollout.
- `--jobs N` (N ≥ 2): pipeline always splits into reader + writer
  threads; worker pool size = `min(N, K, hardware_concurrency)`. K is
  the count of packet topics discovered during `inspect()`.
- `--jobs 0` (or `auto`): pick `N = min(K, hardware_concurrency)`
  after `inspect()`.

The CLI plumbing lives in `nebuladec_cli`; `bag::convert` takes the
parallel-mode parameter via `ConvertOptions`.

## Edge cases and gotchas

- **Single-topic bags.** N > 1 has no decode-parallelism benefit but
  the pipeline split alone is still a small win (reader I/O overlaps
  with decoder CPU + writer I/O). Acceptable to keep the pipeline
  split on; the cost is two extra threads mostly idle.
- **`Decoder::flush()` ordering.** Workers must flush before signaling
  EOF on their output queue so the trailing scan participates in the
  merge with the correct watermark (`= INT64_MAX` after flush).
- **`make_adapter` race.** The first call per adapter inside
  `Decoder::feed()` loads calibration and resolves the
  package-share-dir. This is exercised by
  `test_decoder_concurrency.cpp`; no extra locking required.
- **Passthrough zero-copy.** `rosbag2_storage::SerializedBagMessage`
  carries a `shared_ptr<rcl_serialized_message_t>`. The passthrough
  queue must hold the `shared_ptr` directly (move on push, move on
  pop) — never copy the buffer.
- **Reader thread shutdown.** If the writer dies (disk full, write
  error), the reader and workers must notice promptly. Use a shared
  abort flag checked at each queue push/pop.
- **Determinism.** Output `ConvertResult` counts (`packets`,
  `clouds_written`) must match the sequential version exactly. Per-
  worker counters are aggregated on the writer thread after EOF, not
  during the merge.
- **Memory of passthrough-heavy bags.** With bounded queues and
  zero-copy passthrough, steady-state memory is `O(queue_capacity ×
message_size)` per queue. Empirically dominated by the largest
  passthrough payload (e.g. uncompressed image frames).

## Risks

- **Test coverage.** Existing `test_bag_io_*` tests are sequential.
  New tests should cover: (a) byte-for-byte equivalence between
  `--jobs 1` and `--jobs N` outputs for a representative multi-topic
  bag, (b) deadlock-free shutdown when the writer errors mid-stream,
  (c) correctness under a bag where one topic's decoder lags far
  behind others (watermark exercise).
- **MCAP compression interaction.** `rosbag2_storage_mcap` may use an
  internal compression thread pool. The writer thread becoming a
  serialization point means MCAP-internal parallelism is the next
  bottleneck once the pipeline is in place; profile before adding
  further writer-side parallelism.
- **API surface.** `ConvertOptions` gains a field; downstream callers
  (CLI, any embedders) must opt in. Default stays at `jobs = 1` to
  preserve current behavior.

## Implementation outline (when picked up)

1. Introduce `ConvertOptions::jobs` (default 1) and thread it through
   `convert()`.
2. Extract the current main loop body into a "sequential driver"
   function so the existing path is preserved verbatim when `jobs ==
1`.
3. Add a "parallel driver" with the three-stage pipeline behind the
   same `convert()` entry point.
4. Per-topic worker = small RAII type owning `(PacketSource, Decoder,
input_queue, output_queue, stats counters)`. Reuses the existing
   `TopicState` fields where possible.
5. K-way merge implemented as a free function over a span of output
   queues; reused for unit tests independent of the bag plumbing.
6. New tests: byte-for-byte equivalence harness; concurrency stress
   adapted from `nebuladec_adapters/test/test_decoder_concurrency.cpp`.
7. Document the `--jobs` flag in `nebuladec_cli` once the feature
   lands.
