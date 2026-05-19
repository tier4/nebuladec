# nebuladec_bag

A library that takes a ROS 2 bag as input, decodes Nebula-family LiDAR raw
packet topics into `sensor_msgs/msg/PointCloud2`, and writes a new bag.
Non-target topics (TF, IMU, cameras, unsupported LiDARs, etc.) are passed
through verbatim. The `convert` subcommand of `nebuladec_cli` is the direct
consumer.

## Public headers (`include/nebuladec_bag/`)

### `bag_io.hpp` — main API

Four free functions in the namespace `nebuladec::bag`:

| API                                 | Role                                                                                                                                                                                                                               |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `detect_input(path)`                | Auto-detects the storage plugin and layout from a path and returns an `InputSpec` (`uri`, `storage_id`, `is_directory`). Looks at the `.mcap` / `.db3` extension, or `storage_identifier` from `metadata.yaml` inside a directory. |
| `inspect(input_path)`               | Opens the bag, sniffs the first packet on every Nebula packet topic to resolve vendor / model, and returns an `InspectSummary` (per-topic `Identity` and `has_messages`).                                                          |
| `plan_convert(input_path, mapping)` | Dry-run. Resolves every packet topic against the `TopicMapping` and returns a `ConvertPlanEntry` (status is `"ok"`, `"skipped"`, or `"error"`).                                                                                    |
| `convert(options)`                  | The full pipeline: read → identify → decode → write `PointCloud2` → pass through everything else. Returns a `ConvertResult`.                                                                                                       |

Supporting value types: `InputSpec`, `TopicInspectResult`, `InspectSummary`,
`ConvertOptions`, `TopicConvertResult`, `ConvertResult`, `ConvertPlanEntry`,
`ProgressEvent`.

#### `ConvertOptions::on_progress`

`ConvertOptions` carries an optional `std::function<void(const ProgressEvent &)> on_progress` sink. When set, `convert()` invokes it from the reader stage to report cumulative decode progress across every decoded topic in the bag (passthrough traffic is excluded). `ProgressEvent::messages_total` is fixed at `convert()` entry from bag metadata; `messages_done` is monotonic.

- Calls are throttled to at most one every ~50 ms.
- A final call always fires after the writer scope closes, so the consumer always sees a snapshot with `messages_done == messages_total`.
- Callback exceptions are swallowed — a broken UI cannot abort decoding.
- Leave empty (default) to disable; the per-message branch collapses to a single function-pointer check.

The CLI uses this hook to drive an `indicators::BlockProgressBar`; see [`nebuladec_cli/README.md`](../nebuladec_cli/README.md#progress-bar) for the user-facing UX and the `--no-progress` flag.

#### `ConvertOptions::mcap`

`ConvertOptions::mcap` (of type `McapWriteOptions`) selects compression and chunk size for the output MCAP. Output mirrors the input storage plugin, so these fields are only honoured when the input bag is MCAP; on sqlite3 input the library logs a single WARN-level message via `RCUTILS_LOG_WARN_NAMED("nebuladec_bag", ...)` and treats the options as unset.

- `compression`: `McapCompression::{kAuto, kNone, kLz4, kZstd}` (default `kAuto` = writer plugin default, which is `Zstd`).
- `compression_level`: `McapCompressionLevel::{kAuto, kFastest, kFast, kDefault, kSlow, kSlowest}` (default `kAuto`). Ignored when `compression == kNone`.
- `chunk_size_bytes`: `std::uint64_t`. `0` means "writer plugin default" (~768 KiB).

Internally `convert()` writes a short-lived storage-config YAML under `/tmp` and points `rosbag2_storage::StorageOptions::storage_config_uri` at it for the `rosbag2_cpp::Writer` path. The bare-file MCAP path (used when the input bag carries embedded schemas) forwards the same options to `McapDefinitionWriter` which applies them directly to `mcap::McapWriterOptions`. See [`nebuladec_cli/README.md`](../nebuladec_cli/README.md#mcap-tuning) for the user-facing flags and a benchmark.

### `message_definition.hpp` — embedded schema forwarding

Defines `MessageDefinition` (POD: `type_name`, `encoding`, `text`),
`MessageDefinitionSource` (abstract source of definitions per storage
plugin), `MessageDefinitionRegistry` (type-name keyed view), the factory
`make_definition_source(spec)`, and the convenience
`load_definition_registry(spec)`.

The motivation is the `oxts_msgs/msg/Ncom`-style failure mode: when a
rosbag2 MCAP file is converted on a host that does not have the input
bag's message-type packages installed, `rosbag2_storage_mcap` writes an
empty Schema record to the output and emits `definition file(s) missing
for ...` at ERROR level. With the registry populated from the input
bag's embedded Schema records, `bag::convert()` switches to a
definition-aware writer that forwards the Schema byte-for-byte. See
_End-to-end flow_ below.

### `point_cloud2.hpp`

Just one function: `to_point_cloud2(cloud, stamp, frame_id)`. Converts a
`nebula::drivers::NebulaPointCloud` into `sensor_msgs::msg::PointCloud2`. The
field descriptors are generated from `NebulaPoint::fields()` (x, y, z,
intensity, return_type, channel, azimuth, elevation, distance, time_stamp).

## Implementation (`src/`)

- **`bag_io.cpp`** — Implements the four public functions.
  - `inspect`: for a bare `.db3` file it bypasses `rosbag2_cpp::Reader`'s expensive full-table scan and queries SQLite3 directly, fetching one `SELECT MIN(id)` per packet topic. For MCAP and directory bags it uses `rosbag2_cpp::Reader` with a `StorageFilter`.
  - `convert`: runs `inspect()` first to settle identities, then opens a Reader and Writer. It creates `PointCloud2` output topics for matched/supported packet topics and passthrough topics for everything else. After the main loop it calls `Decoder::flush()` so the trailing scan is not dropped. For bare-file inputs it produces a bare-file output by writing to a scratch directory and renaming the storage file.
  - `SqliteGuard` / `SqliteStmtGuard` are RAII wrappers around sqlite3 handles.

- **`packet_source.hpp`** (private) — Declares `PacketBytes` (payload + `stamp_ns`), the abstract `PacketSource`, and the free functions `make_packet_source(type_name)`, `is_packet_type(type_name)`, `vendor_from_message_type(type_name)`.

- **`packet_source.cpp`** — Four `PacketSource` implementations.
  - `NebulaPacketsSource` — `nebula_msgs/msg/NebulaPackets`.
  - `PandarScanSource` — `pandar_msgs/msg/PandarScan` (respects `p.size` to trim oversized arrays).
  - `VelodyneScanSource` — `velodyne_msgs/msg/VelodyneScan`.
  - `RobosenseScanSource` — `robosense_msgs/msg/RobosenseScan`.
  - `vendor_from_message_type`: Pandar → HESAI, Velodyne → VELODYNE, Robosense → ROBOSENSE, NebulaPackets → UNKNOWN (Seyond and Continental share that type, so payload sniffing is required).

- **`point_cloud2.cpp`** — `memcpy`s the `NebulaPoint` buffer directly and builds the `PointCloud2` with `point_step = sizeof(NebulaPoint)`.

## End-to-end flow

1. **Resolve the input** — `detect_input(path)` picks the storage plugin and layout.
2. **Identify sensors** — `inspect()` creates a `PacketSource` per packet topic, reads the first message, and runs `PacketSniffer::identify()` + `Decoder::feed()` to settle each `Identity`.
3. **Plan** — `plan_convert()` runs `TopicMapping::resolve()` per topic and classifies it as `ok`, `skipped`, or `error`.
4. **Decode** — `convert()` reads each packet message in the main loop, feeds bytes to `Decoder::feed()`, and converts returned clouds via `to_point_cloud2()`. Unmatched / unsupported topics are written through unchanged. After the loop, `Decoder::flush()` drains the trailing scan.
5. **Write the output** — Output mirrors the input's storage plugin and layout. For bare-file inputs the writer creates a scratch directory and renames the final storage file into place.
   - **Schema-forwarding fast path (MCAP → MCAP, bare file).** When the input bag carries embedded `Schema` records and the output is a bare-file MCAP, `convert()` skips `rosbag2_cpp::Writer` and writes directly via `mcap::McapWriter` (`McapDefinitionWriter`). Schema records are sourced in priority order: (a) input bag registry, (b) local ament index via the recursive `.msg` loader in `ament_message_loader.cpp`, (c) soft fail (empty Schema + warning, matching `rosbag2_storage_mcap`'s default). The other three cases (sqlite3 output, MCAP directory output, MCAP→MCAP where the registry is empty) keep the original `rosbag2_cpp::Writer` flow byte-for-byte.

## Performance: 3-stage pipeline for `convert()`

`convert()` runs as a **3-stage pipeline by default**: one reader thread,
N decoder workers, and one writer thread, with bounded queues between
stages. The output bag carries the same topics, timestamps, and
payloads as a single-threaded run — `ros2 bag play` behaves identically.

### Before — single-threaded (legacy, still available via `--sequential`)

```text
[Input bag]
    │
    ▼
 ┌──read──┐  ┌─decode─┐  ┌─write──┐
 │ packet │→ │ packet │→ │ cloud  │→  [Output bag]
 └────────┘  └────────┘  └────────┘
        ▲         ▲          ▲
        └─── ONE thread does all three, taking turns ───┘
```

One thread reads, decodes, writes, then moves on. While the decoder is
busy, file I/O sits idle. While write I/O is slow (MCAP + zstd to slow
storage), decode sits idle.

### After — 3-stage pipeline (default)

```text
                       ┌─ Worker A (topic /lidar_front) ──► decoded clouds ┐
[Input bag]            │                                                    │
    │                  ├─ Worker B (topic /lidar_left)  ──► decoded clouds ┤      ┌──────────┐
    ▼                  │                                                    ├─────►│  Writer  │
[Reader thread] ───────┼─ Worker C (topic /lidar_right) ──► decoded clouds ┤      │  thread  │
 (single I/O ingest)   │                                                    │      │  (FIFO   │
                       └──── pass-through (TF, IMU, camera, ...) ──────────┘      │  drain)  │
                                                                                   └────┬─────┘
                                                                                        ▼
                                                                                  [Output bag]
                       ◄──── all stages run concurrently ────►
              (all four arrows feed one shared bounded write queue)
```

Each stage runs on its own thread:

- **1 reader thread** pulls messages from the input bag in `log_time`
  order. Decoded-topic packets are routed into the input queue of the
  worker that owns the source topic — **one shared queue per worker,
  fed in arrival order across all topics that worker is assigned** —
  while pass-through messages skip the workers and go straight onto
  the shared write queue. (One queue per worker, not one per topic:
  with per-topic queues a `workers < K` config deadlocked because the
  worker drained one topic to EOF while the reader filled another
  topic's queue to its cap and blocked.)
- **N worker threads** (`--workers N`, default `min(cores, K)`) decode
  LiDAR topics concurrently. Each worker holds its own `Decoder` per
  assigned topic and feeds packets in monotonic order, satisfying the
  per-instance threading contract of `nebuladec_adapters`. When
  `workers < K`, each worker handles **K / workers topics** evenly
  (the worker count is snapped down to the largest divisor of K so
  the split is exact). Decoded clouds are pushed onto the same shared
  write queue the reader writes pass-through to.
- **1 writer thread** drains the shared bounded queue FIFO. The reader
  and workers are decoupled producers, so a temporarily idle topic
  never holds back the others — the writer only ever waits on disk
  I/O, not on a cross-topic ordering invariant. The output bag holds
  the same multiset of `(topic, log_time, payload)` records as the
  legacy single-threaded path; `ros2 bag play` (which is `log_time`
  -driven) treats both bags identically.

### Why this is faster

Two independent gains stack:

```text
Time ────────────►

Sequential:
  Read    ▓░░░░░░░░░░░░░░░░░
  Decode  ░▓▓▓░░░░░░░░░░░░░░
  Write   ░░░░▓░░░░░░░░░░░░░
          └── one batch ──┘  then repeat
          Total wall-clock = sum of all 3 stages

Pipelined:
  Read    ▓▓▓▓▓▓▓▓▓░░░
  Decode  ░▓▓▓▓▓▓▓▓▓▓░       (per-topic parallelism shrinks this stage)
  Write   ░░▓▓▓▓▓▓▓▓▓▓
          Total wall-clock ≈ MAX of the 3 stages, not their sum
```

| Mechanism                 | What it overlaps                  | When it matters                  |
| ------------------------- | --------------------------------- | -------------------------------- |
| Pipeline split (3 stages) | read I/O ↔ decode CPU ↔ write I/O | Always — even with 1 worker      |
| Worker pool (N threads)   | decode of N topics concurrently   | When decode is the slowest stage |

### How parallelism is chosen

```text
                                        ┌──► SEQUENTIAL (legacy thread)
       --sequential set? ──yes──────────┘
            │
            no
            ▼
       cores < 3? ──────────yes─────────┐
            │                            │
            no                           ├──► SEQUENTIAL (auto-fallback)
            ▼                            │
       K == 0 LiDAR topics? ─yes─────────┘    (K = decoded packet
            │                                  topics found by inspect())
            no
            ▼
       workers_request = (--workers N if given) else min(cores, K)
       workers = min(workers_request, K)
       if workers < K:
         workers = largest divisor of K that is ≤ workers
            │
            ▼
       PIPELINE with `workers` worker threads,
       each owning K / workers topics
```

The divisor clamp keeps load **even across workers** when `workers < K`:

| K   | `--workers` requested | Effective workers | Topics per worker            |
| --- | --------------------- | ----------------- | ---------------------------- |
| 8   | (default, 8+ cores)   | 8                 | 1                            |
| 8   | 5                     | 4                 | 2                            |
| 8   | 3                     | 2                 | 4                            |
| 6   | 5                     | 3                 | 2                            |
| 7   | 5                     | 1                 | 7 (K is prime → degenerates) |

When K is prime and `--workers < K`, the divisor clamp degenerates to 1
worker. The CLI surfaces this in `--help`; the workaround is to pass
`--workers K` (matching the topic count exactly).

### CLI flags

| Flag           | Effect                                                                                                                      |
| -------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `--workers N`  | Override the decoder worker count. Capped to K and snapped down to the largest divisor of K (see table above).              |
| `--sequential` | Force the legacy single-threaded path. Mutually exclusive with `--workers`. Useful for byte-for-byte regression comparison. |

When neither flag is given, `convert()` runs the pipeline with
`min(hardware_concurrency(), K)` workers (further adjusted by the
divisor rule when needed).

### Plain-language summary

- The output bag is the same multiset of `(topic, log_time, payload)`
  records whether you use `--sequential` or the default pipeline.
- More LiDAR topics in the bag → bigger speedup. A bag with one heavy
  LiDAR still benefits from the read/decode/write overlap, but not
  from the worker pool.
- Hosts with fewer than 3 hardware threads, or bags with no LiDAR
  topics to decode, fall back to the sequential path automatically.

## Topic naming

There is no fixed naming convention — the package follows whatever the
user-supplied `TopicMapping` YAML says. Examples used in the tests:

| Input (packets)     | Output (PointCloud2) |
| ------------------- | -------------------- |
| `/pandar_packets`   | `/pandar_points`     |
| `/velodyne_packets` | `/velodyne_points`   |

`frame_id` is also defined per mapping rule (the tests use `"lidar"`).

The recognized ROS message types are:

- `nebula_msgs/msg/NebulaPackets` (Seyond LiDAR / Continental radar)
- `pandar_msgs/msg/PandarScan` (Hesai)
- `velodyne_msgs/msg/VelodyneScan` (Velodyne)
- `robosense_msgs/msg/RobosenseScan` (Robosense — identified but not decoded to PointCloud2)

## Tests (`test/`)

- **`test_detect_input.cpp`** — Unit tests for `detect_input()`. Builds fixtures via `mkdtemp` and checks MCAP / SQLite3 directory detection, bare `.mcap` and `.db3` files, and exceptions for missing `metadata.yaml`, unknown extensions, and non-existent paths.
- **`test_point_cloud2.cpp`** — `to_point_cloud2()` with empty clouds, two-point round trips (memcpy back and check values), and matching field names / offsets / counts against `NebulaPoint::fields()`.
- **`test_convert_hesai_qt128.cpp`** — Regression test for the Hesai cloud-ownership bug. `HesaiDecoder` clears its frame buffer after the scan callback, so the adapter must own a copy. Uses a Nebula-provided golden bag under `../dependencies/nebula/.../qt128/.../*.db3`, baked in via `NEBULADEC_BAG_TEST_QT128_BAG`. Built conditionally.
- **`test_convert_velodyne_vlp16.cpp`** — Regression test for the Velodyne trailing-scan loss. `VelodyneDriver` only surfaces the previous scan on azimuth wrap, so without `flush()` the last scan is dropped. Uses a VLP16 golden bag baked in via `NEBULADEC_BAG_TEST_VLP16_BAG`. Built conditionally.
- **`test_message_definition_source.cpp`** — Unit tests for the MCAP and SQLite3 sources of embedded definitions. Synthesizes MCAP and `metadata.yaml` fixtures in-process; covers byte-for-byte round trip, empty-bag behaviour, missing-file error, and the Iron-style `message_definition` block parser.
- **`test_convert_embedded_definition.cpp`** — End-to-end regression for the schema-forwarding fast path. Builds a one-topic MCAP whose type package is not installed in the build environment, runs `bag::convert()`, and asserts the output bag's Schema record carries the same `.msg` text.

## Dependencies

`package.xml`:

- `depend`: `libsqlite3-dev`, `mcap_vendor`, `nebula_msgs`, `nebuladec_adapters`, `pandar_msgs`, `rclcpp`, `robosense_msgs`, `rosbag2_cpp`, `rosbag2_storage`, `sensor_msgs`, `velodyne_msgs`, `yaml-cpp`
- `exec_depend`: `rosbag2_storage_mcap` (runtime plugin for MCAP storage)
- `test_depend`: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`
- `buildtool_depend`: `ament_cmake_auto`

## Build artifacts

- Shared library `nebuladec_bag`. Linked explicitly against `SQLite::SQLite3` and `mcap_vendor::mcap`.
- Test binaries: `test_detect_input` and `test_point_cloud2` are always built. `test_convert_hesai_qt128` and `test_convert_velodyne_vlp16` register only when the corresponding golden bag is present.
- `ament_cmake_uncrustify` is suppressed — formatting is enforced by clang-format via pre-commit.
