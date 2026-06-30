#pragma once
// Compatibility shim for building Nebula under the conda / RoboStack toolchain.
//
// Nebula's decoders use the GNU single-precision math constant M_PIf, which the
// conda sysroot's glibc <math.h> does not define (it ships only the
// bit-width-named variants such as M_PIf32). <cmath> is included first so that a
// libc which *does* define M_PIf wins; the guard then fills it in only when the
// toolchain leaves it undefined. Force-included into every C++ translation unit
// via CXXFLAGS by scripts/pixi-activate-build-compat.sh.
#include <cmath>

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif
