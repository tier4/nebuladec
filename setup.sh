#!/usr/bin/env bash
# setup.sh - Install third-party sources and ROS package dependencies
# required to build nebuladec.
#
# Run after cloning the repository. Performs two steps:
#   1. Import third-party sources listed in build_depends.repos into
#      src/dependencies/ via `vcs import` (Nebula, sync_tooling_msgs,
#      agnocast). The directory is git-ignored so it is recreated on
#      every fresh checkout.
#   2. Resolve ROS package dependencies via `rosdep install` over the
#      whole workspace (the imported sources plus the in-tree packages
#      under src/nebuladec_*).
#
# Despite the filename, this script is meant to be executed, not sourced.
#
# Requires:
#   - ROS_DISTRO sourced (e.g. `source /opt/ros/humble/setup.bash`)
#   - vcs (python3-vcstool)
#   - rosdep (initialised with `sudo rosdep init && rosdep update`)

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOS_FILE="${SCRIPT_DIR}/build_depends.repos"

if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "[setup.sh] ROS_DISTRO is not set. Source your ROS environment first." >&2
    exit 1
fi

if ! command -v vcs >/dev/null 2>&1; then
    echo "[setup.sh] 'vcs' is not installed or not on PATH. Install python3-vcstool." >&2
    exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
    echo "[setup.sh] 'rosdep' is not installed or not on PATH." >&2
    exit 1
fi

if [[ ! -f ${REPOS_FILE} ]]; then
    echo "[setup.sh] Cannot find ${REPOS_FILE}." >&2
    exit 1
fi

echo "[setup.sh] Importing third-party sources from $(basename "${REPOS_FILE}")"
mkdir -p "${SCRIPT_DIR}/src"
vcs import --recursive "${SCRIPT_DIR}/src" <"${REPOS_FILE}"

echo "[setup.sh] Installing ROS package dependencies via rosdep (distro=${ROS_DISTRO})"
rosdep install \
    --from-paths "${SCRIPT_DIR}/src" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r -y

echo "[setup.sh] Done. Build with ./build.sh"
