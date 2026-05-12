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

## Tests (`test/`)

- `test_packet_sniffer.cpp` — Identification of every supported model (Hesai / Velodyne / Robosense / Seyond / Continental), rejection of empty / corrupt / undersized / size-mismatched packets, Seyond status-packet filtering, and `vendor_hint` restriction.
- `test_support_registry.cpp` — Asserts that only HESAI / VELODYNE / SEYOND are supported, every `SupportLevel` return is correct, and `supported_vendors()` has the expected size.
- `test_topic_mapping.cpp` — Parsing of absolute and relative rules, error paths (missing fields, invalid placeholders, mixing absolute and relative rules), resolution logic, ambiguity raising, and `nullopt` on no-match.

## Dependencies

`package.xml`:

- `depend`: `nebula_core_common`, `nebula_seyond_common`, `yaml_cpp_vendor`
- `buildtool_depend`: `ament_cmake_auto`
- `test_depend`: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`

## Build artifacts

- Shared library `nebuladec_core` (three source files).
- Public include directory exported as `include/` (both build- and install-interface in ament).
- Explicit link against `yaml-cpp` (required on ROS 2 Jazzy).

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
