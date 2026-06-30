#!/usr/bin/env bash
# pixi-activate-build-compat.sh - Sourced by pixi on environment activation
# (see [activation] in pixi.toml).
#
# Compile-time shims that let Nebula build under the conda / RoboStack toolchain.
# Neither is needed at runtime; both are harmless when already satisfied.
#
#   1. png++ headers: Nebula's decoders #include <png++/...> (a header-only
#      wrapper around libpng). png++ is not packaged on conda-forge or RoboStack
#      and pixi does not use rosdep, so the headers are vendored in
#      third_party/pngpp and exposed via CPLUS_INCLUDE_PATH. GCC treats that like
#      -isystem, so png++'s own warnings are not promoted to errors under
#      Nebula's -Werror. libpng itself comes from the conda environment via
#      find_package(PNG).
#
#   2. M_PIf: Nebula uses the GNU single-precision constant M_PIf, which the
#      conda sysroot glibc <math.h> does not define. The guarded shim
#      third_party/compat/nebula_glibc_compat.hpp defines it when missing and is
#      force-included into every C++ translation unit via CXXFLAGS.
#
# This script is sourced, not executed: do not call `exit`.

_root="${PIXI_PROJECT_ROOT:-.}"

_pngpp_inc="${_root}/third_party/pngpp/include"
if [ -d "${_pngpp_inc}" ]; then
    export CPLUS_INCLUDE_PATH="${_pngpp_inc}${CPLUS_INCLUDE_PATH:+:${CPLUS_INCLUDE_PATH}}"
fi

_compat_hdr="${_root}/third_party/compat/nebula_glibc_compat.hpp"
if [ -f "${_compat_hdr}" ]; then
    export CXXFLAGS="${CXXFLAGS:+${CXXFLAGS} }-include ${_compat_hdr}"
fi

unset _root _pngpp_inc _compat_hdr
