# nebuladec_adapters

A thin adapter layer that wraps Nebula's vendor-specific decoders (Hesai,
Velodyne, Seyond) behind the `AnyDecoder` interface from `nebuladec_core`.
The top-level `Decoder` facade auto-identifies the vendor from a stream of
raw packets, lazily instantiates the matching adapter, caches it, and
returns point clouds.

## Public headers (`include/nebuladec_adapters/`)

| Header                 | Purpose                                                                                                                                                                               |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `decoder.hpp`          | The SDK facade for this package. Declares `class Decoder` (auto-routing, vendor sniffing, min-point filter, `flush()`) and the free factory function `make_adapter(const Identity&)`. |
| `hesai_adapter.hpp`    | `HesaiAdapter` — wraps `nebula::drivers::HesaiDriver`.                                                                                                                                |
| `velodyne_adapter.hpp` | `VelodyneAdapter` — wraps `nebula::drivers::VelodyneDriver`.                                                                                                                          |
| `seyond_adapter.hpp`   | `SeyondAdapter` — wraps `nebula::drivers::SeyondDecoder`.                                                                                                                             |

All three adapters inherit from `nebuladec_core::AnyDecoder` and implement
`feed`, `flush`, and `identity`.

## Support matrix

| Adapter           | Wrapped class                     | Vendors decoded                                                     |
| ----------------- | --------------------------------- | ------------------------------------------------------------------- |
| `HesaiAdapter`    | `nebula::drivers::HesaiDriver`    | Hesai (Pandar40P, PandarXT32, Pandar64, etc.)                       |
| `VelodyneAdapter` | `nebula::drivers::VelodyneDriver` | Velodyne (VLP16, VLP32, VLS128, etc.)                               |
| `SeyondAdapter`   | `nebula::drivers::SeyondDecoder`  | Seyond                                                              |
| _(no adapter)_    | —                                 | Robosense — identified by the sniffer, not decoded to PointCloud2   |
| _(no adapter)_    | —                                 | Continental — identified by the sniffer, not decoded to PointCloud2 |

## The factory and the `Decoder` facade

- `make_adapter(const Identity&)` switches on `identity.vendor` and returns
  the matching adapter, or `nullptr` for unsupported / unknown vendors. There
  is no dynamic registry — the switch statement _is_ the registry.
- `Decoder` owns a `PacketSniffer` and lazily calls `make_adapter` on the
  first identifiable packet, then caches the adapter.
- `set_vendor_hint(Vendor)` pre-biases the sniffer when the caller already
  knows the vendor (e.g. derived from a ROS 2 message type).
- `set_min_points` / its getter let the caller drop scans with too few
  points so noise scans never reach downstream consumers.

### Thread-safety contract

A single `Decoder` instance is **not** thread-safe — `feed()` mutates
the cached adapter and identity state on every call. **Distinct
instances, however, are independent and may be driven concurrently from
different threads.** Orchestrators that decode several lidar topics in
parallel (e.g. `nebuladec_bag` with a per-topic worker pool) should
create one `Decoder` per stream and dispatch packets accordingly. The
shared singletons `PacketSniffer` and `SupportRegistry::instance()` are
themselves safe to share across threads. See
`test/test_decoder_concurrency.cpp` for the stress-tested guarantees.

## Implementation (`src/`)

- `decoder.cpp` — The `Decoder` class (`feed/flush/identity/set_vendor_hint/set_min_points`) and the `make_adapter` factory.
- `hesai_adapter.cpp`, `velodyne_adapter.cpp`, `seyond_adapter.cpp` — Per-vendor adapter shims.

## Tests (`test/`)

- `test_decoder.cpp` — `Decoder` identity lifecycle, garbage-packet tolerance, Seyond routing, `min_points` getter/setter, and `make_adapter` behavior for Seyond and unknown vendors.
- `test_hesai_adapter.cpp` — `is_ready()` for Pandar40P and PandarXT32, rejection of UNKNOWN model, empty-packet safety, and a `make_adapter` round-trip.
- `test_velodyne_adapter.cpp` — `is_ready()` for VLP16 and VLS128, rejection of UNKNOWN model, and empty-packet safety.
- `test_decoder_integration.cpp` — End-to-end Sniffer → `make_adapter` → `AnyDecoder::feed`. Realistic byte sequences for Hesai Pandar40P, Velodyne VLP16, Seyond, and Robosense (Helios, BpearlV3). Confirms Robosense and garbage produce no point clouds.
- `test_decoder_concurrency.cpp` — Stress tests for the per-instance thread-safety contract: a shared `PacketSniffer` and `SupportRegistry::instance()` driven from 4 threads, 4 independent per-thread Seyond `Decoder`s, and 4 mixed-vendor (Seyond / Hesai / Velodyne) `Decoder`s constructed and fed concurrently. Catches races in calibration loading and in the package-share-dir lookup that `make_adapter` performs.

## Dependencies

`package.xml`:

- `buildtool_depend`: `ament_cmake_auto`
- `depend`: `ament_index_cpp`, `nebuladec_core`,
  `nebula_hesai_common`, `nebula_hesai_decoders`,
  `nebula_seyond_common`, `nebula_seyond_decoders`,
  `nebula_velodyne_common`, `nebula_velodyne_decoders`
- `test_depend`: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`

## Build artifacts

- Shared library `nebuladec_adapters` (four source files). Public includes exported via `include/`.
- Test executables: `test_decoder`, `test_hesai_adapter`, `test_velodyne_adapter`, `test_decoder_integration`, `test_decoder_concurrency`.
- `ament_cmake_uncrustify` is suppressed — formatting is enforced by clang-format via pre-commit.
- CMake option `NEBULADEC_PROFILE` (default `OFF`) mirrors the same
  option on `nebuladec_core`. When `ON`, the adapter's `feed` paths and
  the wrapped upstream `parse_cloud_packet` / `unpack` calls are wrapped
  in `NEBULADEC_PROFILE_SCOPE` markers; see `nebuladec_core/README.md`
  for the full description.

## Consuming the adapters

The primary consumer is `nebuladec_bag`. It looks up the vendor from the ROS 2
message type, calls `set_vendor_hint` on a `Decoder`, feeds raw packet bytes
in, and converts returned clouds to `PointCloud2`.

```cpp
#include <nebuladec_adapters/decoder.hpp>

nebuladec::Decoder decoder;
decoder.set_vendor_hint(nebuladec::Vendor::HESAI);

for (const auto & packet : packets) {
  if (auto cloud = decoder.feed(packet.data, packet.size, packet.stamp_ns)) {
    // convert to PointCloud2 and write out
  }
}
if (auto tail = decoder.flush()) {
  // emit the trailing scan
}
```

`nebuladec_cli` uses the adapters only transitively through `nebuladec_bag` —
it does not link this package directly.
