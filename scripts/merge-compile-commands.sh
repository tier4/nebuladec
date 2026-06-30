#!/usr/bin/env bash
# merge-compile-commands.sh - Aggregate the per-package compile_commands.json
# files colcon writes (build/<pkg>/compile_commands.json) into a single
# build/compile_commands.json.
#
# Merging them lets editors and language servers (clangd, IDEs) that expect a
# single workspace-level compilation database find entries for every translation
# unit in the workspace. Invoked by the `build` / `build-debug` pixi tasks after
# `colcon build ... -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

set -euo pipefail

# PIXI_PROJECT_ROOT is exported when run as a pixi task; fall back to the current
# directory so the script also works when invoked manually from the repo root.
build_root="${PIXI_PROJECT_ROOT:-.}/build"
merged="${build_root}/compile_commands.json"

shopt -s nullglob
files=("${build_root}"/*/compile_commands.json)
shopt -u nullglob

if [[ ${#files[@]} -eq 0 ]]; then
    exit 0
fi

# Concatenate the arrays from every per-package file. Each input contains a
# single top-level JSON array, so stripping the outer brackets with sed and
# joining the inner entries with commas yields a valid merged array without
# needing jq.
{
    printf '[\n'
    first=1
    for f in "${files[@]}"; do
        if [[ ${f} == "${merged}" ]]; then
            continue
        fi
        body=$(sed -e '1s/^[[:space:]]*\[//' -e '$s/\][[:space:]]*$//' "${f}")
        if [[ -z $(echo "${body}" | tr -d '[:space:]') ]]; then
            continue
        fi
        if [[ ${first} -eq 1 ]]; then
            first=0
        else
            printf ',\n'
        fi
        printf '%s' "${body}"
    done
    printf '\n]\n'
} >"${merged}.tmp"
mv "${merged}.tmp" "${merged}"
