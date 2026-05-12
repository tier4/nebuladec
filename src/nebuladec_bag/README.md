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
`ConvertOptions`, `TopicConvertResult`, `ConvertResult`, `ConvertPlanEntry`.

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

## Dependencies

`package.xml`:

- `depend`: `libsqlite3-dev`, `nebula_msgs`, `nebuladec_adapters`, `pandar_msgs`, `rclcpp`, `robosense_msgs`, `rosbag2_cpp`, `rosbag2_storage`, `sensor_msgs`, `velodyne_msgs`
- `exec_depend`: `rosbag2_storage_mcap` (runtime plugin for MCAP storage)
- `test_depend`: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`
- `buildtool_depend`: `ament_cmake_auto`

## Build artifacts

- Shared library `nebuladec_bag` (three source files). Linked explicitly against `SQLite::SQLite3`.
- Test binaries: `test_detect_input` and `test_point_cloud2` are always built. `test_convert_hesai_qt128` and `test_convert_velodyne_vlp16` register only when the corresponding golden bag is present.
- `ament_cmake_uncrustify` is suppressed — formatting is enforced by clang-format via pre-commit.
