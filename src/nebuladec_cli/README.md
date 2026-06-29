# nebuladec_cli

The `nebuladec` executable. The only runnable package in the `nebuladec`
workspace — a thin CLI wrapper around the `nebuladec_bag` API.

## Executable

- Name: `nebuladec`
- Entry point: `src/main.cpp`
- Argument parser: CLI11 (bundled at `include/third_party/CLI11.hpp`)
- Exactly one subcommand is required (`app.require_subcommand(1)`)
- Exit codes: `0` (success), `64` (usage error), `70` (runtime error)

## `convert` subcommand

```text
nebuladec convert <input> -o <output> -c <config.yaml>
                  [--dry-run] [-j <N> | --sequential]
```

| Option               | Description                                                                                                                                       |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `input`              | Path to the input ROS 2 bag (positional, required).                                                                                               |
| `-o, --output`       | Output bag path. Required unless `--dry-run` is set.                                                                                              |
| `-c, --config`       | YAML mapping config. Required for real conversion. Optional with `--dry-run`.                                                                     |
| `--dry-run`          | Print a plan without writing any bag.                                                                                                             |
| `-j, --workers N`    | Decoder worker pool size for the parallel pipeline. `0` (default) = auto. See below.                                                              |
| `--sequential`       | Force the legacy single-threaded code path. Mutually exclusive with `--workers`.                                                                  |
| `--no-progress`      | Suppress the decode progress bar. Bar is auto-hidden on non-TTY stdout anyway.                                                                    |
| `--mcap-compression` | MCAP output compression: `none` \| `lz4[:LEVEL]` \| `zstd[:LEVEL]` where LEVEL is `fastest`/`fast`/`default`/`slow`/`slowest`. See _MCAP tuning_. |
| `--mcap-chunk-size`  | MCAP output chunk size in bytes; integer with optional `K`/`M`/`G` suffix (binary SI). See _MCAP tuning_.                                         |

### Behavior

- **`--dry-run` alone (no config)** — Inspect-style report: a `topic | vendor | model` table.
- **`--dry-run --config`** — Full resolution plan: an `in_topic | vendor | model | out_topic | frame_id | decodable` table plus a `resolved / skipped / errors` summary line. Exits non-zero if any errors are present.
- **Without `--dry-run`** — Both `--output` and `--config` are required. On success, prints a per-topic results table with input topic, output topic, frame id, resolved identity, processed packet count, and emitted cloud count. Topics with no matching rule are passed through verbatim.
- **`--workers` / `--sequential` with `--dry-run`** — Ignored (warning emitted on stderr); dry-run never enters the decode pipeline.

### Performance: parallel pipeline

`convert` runs as a **3-stage pipeline by default** (one reader thread, N decoder workers, one writer thread). The output bag is the same multiset of `(topic, log_time, payload)` records as the legacy single-threaded path — `ros2 bag play` behaves identically — but the wall-clock is dominated by the slowest stage rather than the sum of all three.

- `--workers N` overrides the auto-chosen worker count.
  - Capped to `K` (the number of decoded LiDAR topics found by `inspect()`).
  - Values below `K` are snapped down to the largest divisor of `K` so each worker can own the same number of topics. Example: with `K = 8` and `--workers 5`, the effective count is `4` (each worker handles 2 topics).
  - When `K` is prime and `--workers < K`, the effective count degenerates to `1`; pass `--workers K` (or omit the flag) for full parallelism.
- `--sequential` forces the legacy single-threaded path. Useful for byte-for-byte regression comparison and on hosts with fewer than 3 hardware threads.
- The pipeline auto-falls-back to the sequential path when `std::thread::hardware_concurrency() < 3` or when the bag has no decodable LiDAR topics.

See [`nebuladec_bag/README.md`](../nebuladec_bag/README.md#performance-3-stage-pipeline-for-convert) for the architectural diagrams.

### MCAP tuning

`convert` drives the output storage plugin from the output path's extension (`.mcap` -> mcap, `.db3` -> sqlite3) for bare-file outputs, so these flags only take effect when the **output** bag is MCAP — including cross-format conversions like `.db3 -> .mcap`. On sqlite3 output the library emits a single warning and ignores them.

- `--mcap-compression VALUE` — `none` | `lz4[:LEVEL]` | `zstd[:LEVEL]` (LEVEL is `fastest`/`fast`/`default`/`slow`/`slowest`). Default: writer plugin default (`zstd:default`).
- `--mcap-chunk-size BYTES` — integer bytes, optional `K`/`M`/`G` suffix (binary SI: `K=1024`). Default: writer plugin default (~768 KiB).

Choosing a value is a wall-clock vs file-size trade. Measured on a 2.5 GB bag (600 messages) writing to a local NVMe (lower `real` is faster, output sizes shown for context):

| `--mcap-compression`       | real (avg, 3 runs) | output size | vs default |
| -------------------------- | ------------------ | ----------- | ---------- |
| (default = `zstd:default`) | 7.04 s             | 4045 MB     | 1.00×      |
| `zstd:fast`                | 4.69 s             | 4272 MB     | **1.50×**  |
| `lz4:fastest`              | 4.95 s             | 4273 MB     | 1.42×      |
| `none`                     | 3.08 s             | 4388 MB     | **2.29×**  |

Notes:

- LiDAR pointclouds compress poorly (output is mostly floats); the size delta between `none` and `zstd:default` is only ~8 % on this workload, so loosening compression buys speed cheaply.
- Avoid bare `--mcap-compression lz4` — libmcap's lz4 _default_ level is anomalously slow (~9× slower than `zstd:default` in the same run). Use `lz4:fastest` instead.
- `--mcap-chunk-size 16M` was a wash on this workload (writer-bound dominated by compression cost). Larger chunks can still help on slow remote storage where fewer write syscalls matter more.

### Progress bar

`nebuladec convert` renders one overall progress bar while it decodes. The bar shows percent complete, the elapsed wall-clock time at **millisecond precision** (e.g. `1.827s`), and a `<done> / <total> messages` postfix. The total counts only the bag-metadata message count of the input topics that will actually be decoded (passthrough traffic is excluded).

- Shown by default when stdout is a TTY **and** the bag carries at least one decoded LiDAR topic.
- Automatically hidden when stdout is not a TTY (CI logs, pipes, `tee` to a file).
- `--no-progress` forces the bar off even on a TTY. Useful when you want a clean redirected log or when running under another wrapper that already shows its own progress UI.
- Ignored under `--dry-run` (no decoding happens).

The bar is driven by the `ConvertOptions::on_progress` callback exposed from `nebuladec_bag`; the CLI is the only consumer today, but library users can pass their own sink instead.

## Bundled third-party headers (`include/third_party/`)

| Header           | Use                                                                                                                                                      |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `CLI11.hpp`      | CLI11 v2.4.2 (github.com/CLIUtils/CLI11, BSD-3). Used from `main.cpp` via `add_option`, `add_flag`, etc.                                                 |
| `tabulate.hpp`   | p-ranav/tabulate. `tabulate::Table` is used to render the dry-run and result summary ASCII tables.                                                       |
| `indicators.hpp` | p-ranav/indicators (MIT). `indicators::BlockProgressBar` drives the `convert` decode progress bar; the CLI keeps a single overall bar across all topics. |

As the `CMakeLists.txt` comments explain, the headers are vendored so
IntelliSense works before the first build and so there is no extra rosdep
entry to maintain.

## Config YAML

Example: `config/x2.yaml`

```yaml
mapping:
  - in_topic: /sensing/lidar/<position>/<vendor>_packets
    out_topic: /sensing/lidar/<position>/<vendor>_points
    frame_id: <position>/lidar
```

Schema:

- Top-level key `mapping`: a YAML sequence of rule objects.
- Each rule has three string fields:
  - `in_topic` — Input topic pattern. May be absolute (starts with `/`) or relative. Each `<name>` placeholder matches a single path segment (regex `[^/]+`). A repeated placeholder name within the same pattern acts as a backreference.
  - `out_topic` — Output template. Captured values from `<name>` placeholders are substituted in.
  - `frame_id` — `frame_id` template. Substituted the same way.
- Rules are evaluated first-match-wins. If two rules match the same topic that's an ambiguous config and is rejected at runtime.

## Dependencies

`package.xml`:

- `buildtool_depend`: `ament_cmake_auto`
- `depend`: `nebuladec_bag`, `nebuladec_core`
- No third-party rosdep entries — CLI11, tabulate, and indicators are vendored as single headers.

## Build artifacts

- Executable `nebuladec` (from `src/main.cpp`).
- C++17. Include path `${CMAKE_CURRENT_SOURCE_DIR}/include` (SYSTEM, PRIVATE).
- Links `nebuladec_bag` and `nebuladec_core` via `ament_target_dependencies`.
- Installed into `bin`.
- To avoid false positives from the vendored headers, the `ament_cmake_uncrustify`, `ament_cmake_copyright`, `ament_cmake_cpplint`, and `ament_cmake_cppcheck` test-time linters are suppressed.

## Example invocations

```bash
# Just list the packet topics in a bag (no config required)
nebuladec convert my_bag --dry-run

# Preview what the mapping would produce
nebuladec convert my_bag --dry-run -c config/x2.yaml

# Real conversion: decode packets to PointCloud2; pass everything else through
nebuladec convert my_bag -o my_bag_decoded -c config/x2.yaml
```
