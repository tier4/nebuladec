# nebuladec

Offline toolset for decoding LiDAR raw packets in a ROS 2 bag into
`sensor_msgs/msg/PointCloud2`. Decoding is delegated to
[TIER IV's Nebula](https://github.com/tier4/nebula); this workspace wraps it in
a "bag in, bag out" CLI.

## Features

- Decode LiDAR packet topics from `.db3`, `.mcap`, or directory bags.
- Auto-identify vendor, model, and return mode from a sample packet.
- Write `PointCloud2` to a new bag; pass everything else through verbatim.
- Rewrite input topics / `frame_id` via YAML mapping.
- Dry-run mode to preview the conversion plan.

## Supported sensors

| Vendor                                                                      | Identify | Decode      |
| --------------------------------------------------------------------------- | -------- | ----------- |
| Hesai (Pandar40P / 64 / QT64 / XT16 / XT32 / XT32M / QT128 / AT128 / OT128) | ✓        | ✓           |
| Velodyne (VLP16 / VLP32 / HDL32 / VLS128)                                   | ✓        | ✓           |
| Robosense (Bpearl V3, Helios)                                               | ✓        | passthrough |
| Continental (ARS548, SRR520)                                                | ✓        | passthrough |

## Install

[pixi](https://pixi.sh) is the only prerequisite. It provisions ROS 2, the C++
toolchain, `colcon`, and `vcs` locally.

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Available environments:

| Environment | ROS 2 distro |
| ----------- | ------------ |
| `humble`    | Humble       |
| `jazzy`     | Jazzy        |

`humble` is the default.

## Build

```bash
pixi run build                # default: Humble
pixi run -e jazzy build       # Jazzy
```

The first build downloads the ROS environment and imports external sources;
later builds are incremental.

Other tasks:

```bash
pixi run build-debug          # Debug build
pixi run clean                # Remove build/ install/ log/
pixi run -e humble install    # Install a prefix-free `nebuladec` launcher
pixi run uninstall            # Remove the launcher
```

## Usage

Run via pixi or from an installed launcher:

```bash
pixi run -e humble nebuladec convert path/to/input_bag --dry-run
```

```bash
pixi shell -e humble
nebuladec convert path/to/input_bag --dry-run
```

Common examples:

```bash
# Inspect packet topics
nebuladec convert path/to/input_bag --dry-run

# Preview with mapping
nebuladec convert path/to/input_bag --dry-run -c config/x2.yaml

# Convert to a new bag
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml

# Adjust decoder parallelism (default: auto)
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml --workers 4

# Force single-threaded mode
nebuladec convert path/to/input_bag -o path/to/output_bag -c config/x2.yaml --sequential

# Cross-format: .db3 <-> .mcap
nebuladec convert path/to/input.db3 -o path/to/output.mcap -c config/x2.yaml
nebuladec convert path/to/input.mcap -o path/to/output.db3 -c config/x2.yaml
```

## Mapping YAML

Example (`config/x2.yaml`):

```yaml
mapping:
  - in_topic: /sensing/lidar/<position>/<vendor>_packets
    out_topic: /sensing/lidar/<position>/<vendor>_points
    frame_id: <position>/lidar
```

`<name>` matches one path segment and reusing it acts as a backreference. See
[`src/nebuladec_cli/README.md`](src/nebuladec_cli/README.md) for the full
schema.

## Tests

```bash
pixi run -e humble test
```

## License

See [`LICENSE`](LICENSE). Bundled external sources carry their own licenses.
