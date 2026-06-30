# Vendored png++ 0.2.10

png++ is a header-only C++ wrapper around libpng. Nebula's decoders pull it in
transitively: `nebula_core_decoders/point_filters/downsample_mask.hpp`
`#include <png++/...>`, and that header reaches `nebuladec_adapters` through the
Hesai decoder (`hesai_decoder.hpp`). png++ is **not** packaged on conda-forge or
RoboStack, and pixi does not use rosdep, so the headers are vendored here.

## Provenance

- Upstream: <https://download.savannah.nongnu.org/releases/pngpp/png++-0.2.10.tar.gz>
- Version: 0.2.10
- Tarball SHA-256: `998af216ab16ebb88543fbaa2dbb9175855e944775b66f2996fc945c8444eee1`
- License: BSD-3-Clause (see `COPYING`)

Only the public headers (`include/png++/*.hpp`) and `COPYING` are vendored; the
release's build, test, example, and documentation files are omitted. libpng
itself is **not** vendored — it comes from the conda environment via
`find_package(PNG)`.

## How it is wired in

`scripts/pixi-activate-build-compat.sh` (registered in `pixi.toml`
`[activation]`) adds `include/` to `CPLUS_INCLUDE_PATH` so `<png++/...>` resolves
during the build. Using `CPLUS_INCLUDE_PATH` (rather than `CPATH`) makes GCC
treat the directory like `-isystem`, so png++'s own warnings are not promoted to
errors under Nebula's `-Werror`.

This tree is excluded from the style hooks (`.pre-commit-config.yaml`) and the
spell checker (`.cspell.json`), like the other vendored third-party sources.
