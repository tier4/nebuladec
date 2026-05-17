# nebuladec_core

The vendor-agnostic foundation library of `nebuladec`. It provides the core
types and functions for identifying a LiDAR sensor from raw packet bytes,
deciding whether that sensor is supported, and resolving input/output topic
mapping rules.

## Overview

This package only contains the logic that answers "which vendor and model is
this packet?" plus the associated metadata (support state, topic mapping
rules, and the abstract decoder interface). It deliberately does not pull in
any Nebula decoders — point-cloud reconstruction belongs to
`nebuladec_adapters` and above.

## Public headers (`include/nebuladec_core/`)

| Header                 | Purpose                                                                                                                                                                                                                                                      |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `identity.hpp`         | The `Vendor` enum (HESAI / VELODYNE / ROBOSENSE / SEYOND / CONTINENTAL) and the `Identity` struct that carries the identification result (vendor, `SensorModel`, `ReturnMode`, optional `SeyondSensorModel`, confidence). Also declares `to_string(Vendor)`. |
| `any_decoder.hpp`      | The pure-virtual `AnyDecoder` interface: `feed()` (raw bytes → optional point cloud) and `flush()` (drain the trailing scan). Implementations live in `nebuladec_adapters`.                                                                                  |
| `packet_sniffer.hpp`   | The stateless `PacketSniffer::identify(data, size, vendor_hint)`. Infers vendor, model, and return mode from a single packet.                                                                                                                                |
| `support_registry.hpp` | The Meyers singleton `SupportRegistry`: `check()`, `is_vendor_supported()`, `is_model_supported()`, `supported_vendors()`. Defines the `SupportLevel` enum and `SupportDecision`.                                                                            |
| `topic_mapping.hpp`    | The YAML-driven `TopicMapping`. Resolves an input topic to an output topic and `frame_id` via `<placeholder>` patterns. Exposes `from_yaml_file()`, `from_yaml_string()`, `resolve()`, `MappingRule`, and `MappingMatch`.                                    |

## Implementation (`src/`)

- `packet_sniffer.cpp` — Per-vendor sniff routines (Hesai SOP, Velodyne block magic + fixed size, the Robosense 4/8-byte magic, Seyond magic + data-item-type, the Continental ARS548/SRR520 header + size pairs).
- `support_registry.cpp` — Hardcoded registry entries for HESAI (9 models), VELODYNE (4 models), and SEYOND (4 models). ROBOSENSE and CONTINENTAL are identified by the sniffer but intentionally absent — there is no PointCloud2 adapter for them.
- `topic_mapping.cpp` — YAML parsing, template tokenization, regex compilation for `<name>` placeholders (with backreferences for repeated placeholder names and prefix preservation for relative rules), and ambiguous-rule detection.
- `profiling.cpp` — Implementation of the opt-in micro-profiler declared in `profiling.hpp`. Compiled in but inert unless `NEBULADEC_PROFILE=1` is defined; see _Profiling_ below.

## Tests (`test/`)

- `test_packet_sniffer.cpp` — Identification of every supported model (Hesai / Velodyne / Robosense / Seyond / Continental), rejection of empty / corrupt / undersized / size-mismatched packets, Seyond status-packet filtering, and `vendor_hint` restriction.
- `test_support_registry.cpp` — Asserts that only HESAI / VELODYNE / SEYOND are supported, every `SupportLevel` return is correct, and `supported_vendors()` has the expected size.
- `test_topic_mapping.cpp` — Parsing of absolute and relative rules, error paths (missing fields, invalid placeholders, mixing absolute and relative rules), resolution logic, ambiguity raising, and `nullopt` on no-match.

Cross-package concurrent-use stress tests live in
`nebuladec_adapters/test/test_decoder_concurrency.cpp`; they exercise
the thread-safety contracts documented on `PacketSniffer`,
`SupportRegistry`, and `TopicMapping` here.

## Dependencies

`package.xml`:

- `depend`: `nebula_core_common`, `nebula_seyond_common`, `yaml_cpp_vendor`
- `buildtool_depend`: `ament_cmake_auto`
- `test_depend`: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`

## Build artifacts

- Shared library `nebuladec_core` (four source files).
- Public include directory exported as `include/` (both build- and install-interface in ament).
- Explicit link against `yaml-cpp` (required on ROS 2 Jazzy).
- CMake option `NEBULADEC_PROFILE` (default `OFF`). When `ON`, the
  micro-profiler defined in `profiling.hpp` is active and the same
  option must also be passed to dependent packages (see _Profiling_).

## Profiling

`nebuladec_core` ships an opt-in micro-profiler (`profiling.hpp` /
`profiling.cpp`) intended for one-off perf investigations, not as a
public API. It is disabled at build time by default; the
`NEBULADEC_PROFILE_SCOPE("label")` macro expands to `((void)0)` and adds
zero runtime cost.

Enable it with:

```bash
colcon build --packages-select nebuladec_core nebuladec_adapters \
  nebuladec_bag nebuladec_cli \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -D NEBULADEC_PROFILE=ON
```

When enabled, each instrumented scope accumulates wall-clock
nanoseconds and call counts into a process-global registry that is
dumped to `stderr` from a static destructor at process exit. Current
scopes:

- `decoder_feed_total`, `decoder_feed_sniff` (in `Decoder::feed`)
- `<vendor>_adapter_feed_total` and the wrapped upstream driver call
  (`accelerated_<vendor>_*_unpack` / `<vendor>_decoder_unpack`) per
  adapter
- For Hesai, three additional scopes inside
  `AcceleratedHesaiDecoder::unpack`:
  `accelerated_hesai_angle_correct`,
  `accelerated_hesai_scan_cutter_step`, and
  `accelerated_hesai_convert_returns`

ament does not propagate `target_compile_definitions PUBLIC` across
packages, so `nebuladec_adapters` independently honours the same
option — both must be rebuilt together with `-D NEBULADEC_PROFILE=ON`.

## Consuming the library

Downstream packages add `<depend>nebuladec_core</depend>` to their `package.xml`
and use it via `ament_auto_find_build_dependencies()`. Adapter implementations
inherit from `AnyDecoder` and consult `SupportRegistry::instance()` to gate
the models they handle.

```cpp
#include <nebuladec_core/packet_sniffer.hpp>
#include <nebuladec_core/support_registry.hpp>

nebuladec::PacketSniffer sniffer;
auto identity = sniffer.identify(bytes, size);
auto decision = nebuladec::SupportRegistry::instance().check(identity);
```
