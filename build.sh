#!/usr/bin/env bash
# Configure and build asv_sim. Pass --clean to wipe the build directory first.
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "--clean" ]]; then
  rm -rf build
fi

args=(-S . -B build -DCMAKE_BUILD_TYPE=Release)

# macOS only: optionally pin the SDK, e.g.
#   export ASV_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
if [[ "$(uname)" == "Darwin" && -n "${ASV_OSX_SYSROOT:-}" ]]; then
  args+=(-DCMAKE_OSX_SYSROOT="$ASV_OSX_SYSROOT")
fi

cmake "${args[@]}"
cmake --build build --config Release --parallel