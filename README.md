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
├── pixi.toml                # pixi manifest: ROS distros, dependencies, build tasks
├── pixi.lock                # Locked, reproducible dependency set (committed)
├── scripts/                 # Helper scripts invoked by the pixi tasks
└── third_party/             # Vendored deps not packaged on conda (png++, compat shims)
```

The package dependency stack is roughly **`cli` → `bag` → `adapters` → `core`**.
Per-package details:

- [`src/nebuladec_core/README.md`](src/nebuladec_core/README.md) — Vendor identification, support gating, and topic mapping core.
- [`src/nebuladec_adapters/README.md`](src/nebuladec_adapters/README.md) — Per-vendor decoder adapters and the `Decoder` facade.
- [`src/nebuladec_bag/README.md`](src/nebuladec_bag/README.md) — ROS 2 bag I/O and the decode pipeline.
- [`src/nebuladec_cli/README.md`](src/nebuladec_cli/README.md) — The `nebuladec` executable and its CLI surface.

## Requirements

[pixi](https://pixi.sh) is the only prerequisite. It provisions ROS 2 (via
[RoboStack](https://robostack.github.io/) conda packages), the C++ toolchain,
`colcon`, and `vcs` into a project-local environment — no system ROS, `apt`, or
`rosdep` setup is required.

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

The ROS distro is chosen per pixi environment. Two are available:

| Environment | ROS 2 distro | RoboStack channel             |
| ----------- | ------------ | ----------------------------- |
| `humble`    | Humble       | `prefix.dev/robostack-humble` |
| `jazzy`     | Jazzy        | `prefix.dev/robostack-jazzy`  |

`humble` is the default environment (used when `-e` is omitted).

> **ROS 2 Lyrical** is not provided yet: the RoboStack environment installs, but
> Nebula's sources do not compile on Lyrical — Nebula uses the
> `ament_target_dependencies` CMake macro that Lyrical's `ament_cmake` removed.
> It will be added once Nebula supports Lyrical.

## Setup & build

Pick a distro and build. The first build for a distro downloads the RoboStack
ROS environment (multi-GB) and imports the external sources; later builds are
incremental.

```bash
pixi run -e humble build      # ROS 2 Humble (also the default environment)
pixi run -e jazzy build       # ROS 2 Jazzy

pixi run build                # no -e: uses the default environment (Humble)
```

Each `build` runs two steps in order:

1. `import-deps` — `vcs import` against `build_depends.repos` to fetch the
   pinned external sources (Nebula, sync_tooling_msgs, agnocast) into
   `src/dependencies/` (git-ignored, recreated on every fresh checkout).
2. `colcon build --symlink-install --packages-up-to nebuladec_cli` inside the
   selected ROS environment, then merges the per-package
   `compile_commands.json` files into a single `build/compile_commands.json` —
   editors and language servers (clangd, IDEs) expect a single workspace-level
   compilation database.

Other tasks (each accepts `-e <distro>`):

```bash
pixi run build-debug          # Debug build (CMAKE_BUILD_TYPE=Debug)
pixi run import-deps          # fetch the external sources only
pixi run clean                # remove build/ install/ log/
```

## Running

After a build, run `nebuladec` inside the matching pixi environment. Either drop
into a shell that already has ROS and the freshly built binary on `PATH`:

```bash
pixi shell -e humble
nebuladec convert path/to/input_bag --dry-run
```

or invoke a single command without an interactive shell:

```bash
pixi run -e humble nebuladec convert path/to/input_bag --dry-run
```

The examples below assume you are inside `pixi shell` (otherwise prefix each
with `pixi run -e <distro>`):

```bash
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
pixi run -e humble test       # build (if needed) + colcon test + results
```

`test` first builds the product (via `build`), then builds and runs the tests
for the four in-tree `nebuladec_*` packages (`colcon test` + `colcon
test-result --verbose`). Nebula's own test suites are not built (they do not
compile under the conda toolchain). Use `-e jazzy` to test against Jazzy.

Some `nebuladec_bag` regression tests (Hesai QT128 and Velodyne VLP16) only
register when the corresponding Nebula-provided golden bag is present under
`src/dependencies/nebula/...`.

## License

See [`LICENSE`](LICENSE) for the license of this repository. The bundled
external sources (CLI11, tabulate, the vendored png++ headers under
[`third_party/pngpp/`](third_party/pngpp/README.md), and the repositories under
`src/dependencies/`) each carry their own licenses.
