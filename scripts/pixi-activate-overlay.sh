#!/usr/bin/env bash
# pixi-activate-overlay.sh - Sourced by pixi on environment activation
# (see [activation] in pixi.toml).
#
# Layers this workspace's colcon overlay (install/setup.sh) on top of the
# RoboStack ROS environment that pixi has just activated, so the freshly built
# `nebuladec` binary and its libraries are on PATH within `pixi shell` and
# `pixi run`. The guard makes activation a no-op before the first build, when
# install/ does not yet exist.
#
# This script is sourced, not executed: do not call `exit`.

# PIXI_PROJECT_ROOT is exported by pixi; fall back to the current directory so
# the script also works if sourced manually.
_nebuladec_root="${PIXI_PROJECT_ROOT:-.}"

if [ -f "${_nebuladec_root}/install/setup.sh" ]; then
    # shellcheck disable=SC1091  # path is resolved at activation time
    . "${_nebuladec_root}/install/setup.sh"
fi

unset _nebuladec_root
