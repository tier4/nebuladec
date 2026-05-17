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
nebuladec convert <input> -o <output> -c <config.yaml> [--dry-run]
```

| Option         | Description                                                                   |
| -------------- | ----------------------------------------------------------------------------- |
| `input`        | Path to the input ROS 2 bag (positional, required).                           |
| `-o, --output` | Output bag path. Required unless `--dry-run` is set.                          |
| `-c, --config` | YAML mapping config. Required for real conversion. Optional with `--dry-run`. |
| `--dry-run`    | Print a plan without writing any bag.                                         |

### Behavior

- **`--dry-run` alone (no config)** — Inspect-style report: a `topic | vendor | model` table.
- **`--dry-run --config`** — Full resolution plan: an `in_topic | vendor | model | out_topic | frame_id | decodable` table plus a `resolved / skipped / errors` summary line. Exits non-zero if any errors are present.
- **Without `--dry-run`** — Both `--output` and `--config` are required. On success, prints a per-topic results table with input topic, output topic, frame id, resolved identity, processed packet count, and emitted cloud count. Topics with no matching rule are passed through verbatim.

## Bundled third-party headers (`include/third_party/`)

| Header         | Use                                                                                                      |
| -------------- | -------------------------------------------------------------------------------------------------------- |
| `CLI11.hpp`    | CLI11 v2.4.2 (github.com/CLIUtils/CLI11, BSD-3). Used from `main.cpp` via `add_option`, `add_flag`, etc. |
| `tabulate.hpp` | p-ranav/tabulate. `tabulate::Table` is used to render the dry-run and result summary ASCII tables.       |

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
- No third-party rosdep entries — CLI11 and tabulate are vendored as single headers.

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
