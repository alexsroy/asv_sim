#!/usr/bin/env bash
# Run the unit tests (build first with ./build.sh).
set -euo pipefail
cd "$(dirname "$0")"

ctest --test-dir build -C Release --output-on-failure "$@"