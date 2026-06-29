# nebuladec

A Nebula-based offline toolset for decoding LiDAR raw packets recorded in a
ROS 2 bag into `sensor_msgs/msg/PointCloud2`. The per-vendor decoding is
delegated to [TIER IV's Nebula](https://github.com/tier4/nebula); this
workspace adds a thin layer that makes those drivers easy to use in a
"bag in, bag out" workflow.

## What it does

- Takes a ROS 2 bag (`.db3`, `.mcap`, or a directory bag) that contains LiDAR packet topics.
- Sniffs one packet per topic to identify the vendor, model, and return mode.
- Decodes supported sensors into `PointCloud2` and writes the result to a new bag.
- Passes everything else (TF, IMU, cameras, unsupported LiDARs, etc.) through verbatim.
- Lets a YAML mapping rewrite input topics into output topic names and `frame_id` values.
- Supports a dry-run mode that prints the plan without writing any bag.
- Runs as a 3-stage pipeline by default (reader → decoder worker pool → shared FIFO write queue → writer) for concurrent decoding of multiple LiDAR topics; auto-falls-back to a single-threaded path on hosts with fewer than 3 hardware threads. See [`src/nebuladec_bag/README.md`](src/nebuladec_bag/README.md#performance-3-stage-pipeline-for-convert) for the architecture and [`src/nebuladec_cli/README.md`](src/nebuladec_cli/README.md#performance-parallel-pipeline) for the `--workers` / `--sequential` CLI flags.

## Supported sensors

| Vendor                                                                      | Packet identification | PointCloud2 decoding |
| --------------------------------------------------------------------------- | --------------------- | -------------------- |
| Hesai (Pandar40P / 64 / QT64 / XT16 / XT32 / XT32M / QT128 / AT128 / OT128) | ✓                     | ✓                    |
| Velodyne (VLP16 / VLP32 / HDL32 / VLS128)                                   | ✓                     | ✓                    |
| Robosense (Bpearl V3, Helios)                                               | ✓                     | ✗ (passthrough only) |
| Continental (ARS548, SRR520)                                                | ✓                     | ✗ (passthrough only) |

## Repository layout

```text
nebuladec/
├── src/
│   ├── nebuladec_core/      # Vendor identification, support gating, topic mapping
│   ├── nebuladec_adapters/  # Wraps Nebula vendor decoders behind a common interface
│   ├── nebuladec_bag/       # ROS 2 bag I/O glued to the decoders
│   ├── nebuladec_cli/       # The `nebuladec` executable
│   └── dependencies/        # External sources pulled in by vcs (git-ignored)
├── config/                  # Example mapping YAMLs
│   └── x2.yaml
├── build_depends.repos      # Pins for the external sources
├── setup.sh                 # Runs vcs import + rosdep install
└── build.sh                 # colcon build wrapper
```

The package dependency stack is roughly **`cli` → `bag` → `adapters` → `core`**.
Per-package details:

- [`src/nebuladec_core/README.md`](src/nebuladec_core/README.md) — Vendor identification, support gating, and topic mapping core.
- [`src/nebuladec_adapters/README.md`](src/nebuladec_adapters/README.md) — Per-vendor decoder adapters and the `Decoder` facade.
- [`src/nebuladec_bag/README.md`](src/nebuladec_bag/README.md) — ROS 2 bag I/O and the decode pipeline.
- [`src/nebuladec_cli/README.md`](src/nebuladec_cli/README.md) — The `nebuladec` executable and its CLI surface.

## Requirements

- ROS 2 (set `ROS_DISTRO`, e.g. Humble or Jazzy).
- `vcs` (`python3-vcstool`).
- `rosdep` (initialized with `sudo rosdep init && rosdep update`).
- `colcon`.
- A C++17 compiler.

## Setup

```bash
# Source ROS first
source /opt/ros/${ROS_DISTRO}/setup.bash

# Fetch external sources, then resolve ROS package dependencies
./setup.sh
```

`setup.sh` does two things:

1. Runs `vcs import` against `build_depends.repos` to fetch the pinned external sources (Nebula, sync_tooling_msgs, agnocast) into `src/dependencies/` (the directory is git-ignored and is recreated on every fresh checkout).
2. Runs `rosdep install` across the whole workspace (the imported sources plus the in-tree packages).

## Build

```bash
./build.sh                          # default: Release + Make + nproc/2 workers
./build.sh -c                       # clean build (removes install/, build/, log/)
./build.sh --build-type debug       # Debug
./build.sh --build-type info        # RelWithDebInfo
./build.sh --builder ninja          # use Ninja (must be installed)
./build.sh -j 8                     # explicit parallelism
./build.sh -h                       # show full usage
```

`build.sh` requires `ROS_DISTRO` to be set, so
`source /opt/ros/${ROS_DISTRO}/setup.bash` before running it.

Under the hood it runs
`colcon build --symlink-install --packages-up-to nebuladec_cli` and afterwards
merges the per-package `compile_commands.json` files into a single
`build/compile_commands.json` — tools like clang-tidy expect a single
workspace-level compilation database.

After a `-c` clean build, the script automatically strips stale
`${WORKSPACE}/install/...` entries from `AMENT_PREFIX_PATH`,
`CMAKE_PREFIX_PATH`, and `COLCON_PREFIX_PATH`, so a previously sourced
`install/setup.bash` does not leave colcon warning about missing paths.

## Running

```bash
source install/setup.bash

# Inspect what packet topics exist in the bag
nebuladec convert path/to/input_bag --dry-run

# Preview what conversion would produce once a mapping is applied
nebuladec convert path/to/input_bag --dry-run -c config/x2.yaml

# Real conversion: decode packets to PointCloud2; pass everything else through.
# The 3-stage parallel pipeline runs by default with min(num_cores, num_lidar_topics) workers.
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml

# Override the decoder worker count (default: auto).
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml --workers 4

# Force the legacy single-threaded path (e.g. byte-for-byte regression comparison).
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml --sequential

# Suppress the live progress bar (e.g. CI logs, pipes -- auto-hidden on non-TTY too).
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml --no-progress

# Cross-format conversion: the output storage plugin is driven by the output
# path's extension (.mcap / .db3) for bare-file outputs, so .db3 -> .mcap and
# .mcap -> .db3 round-trips work transparently.
nebuladec convert path/to/input.db3 -o path/to/output.mcap -c config/x2.yaml
nebuladec convert path/to/input.mcap -o path/to/output.db3 -c config/x2.yaml

# MCAP writer tuning (output must be .mcap; ignored with a warning on .db3 output).
# Loosen compression for ~1.5-2.3x faster wall-clock at +5-9% file size.
nebuladec convert path/to/input.mcap -o path/to/output.mcap -c config/x2.yaml --mcap-compression zstd:fast
nebuladec convert path/to/input.mcap -o path/to/output.mcap -c config/x2.yaml --mcap-compression none
```

See [`nebuladec_cli/README.md`](src/nebuladec_cli/README.md#mcap-tuning) for the full `--mcap-compression` / `--mcap-chunk-size` matrix and a benchmark on a 2.5 GB bag.

### Mapping YAML

Example (`config/x2.yaml`):

```yaml
mapping:
  - in_topic: /sensing/lidar/<position>/<vendor>_packets
    out_topic: /sensing/lidar/<position>/<vendor>_points
    frame_id: <position>/lidar
```

`<name>` matches a single path segment (`[^/]+`) and reusing the same name
within a pattern acts as a backreference. The full schema is documented in
[`nebuladec_cli/README.md`](src/nebuladec_cli/README.md).

## Tests

```bash
colcon test --packages-up-to nebuladec_cli
colcon test-result --verbose
```

Some `nebuladec_bag` regression tests (Hesai QT128 and Velodyne VLP16) only
register when the corresponding Nebula-provided golden bag is present under
`src/dependencies/nebula/...`.

## License

See [`LICENSE`](LICENSE) for the license of this repository. The bundled
external sources (CLI11, tabulate, and the repositories under
`src/dependencies/`) each carry their own licenses.
