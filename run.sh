#!/usr/bin/env bash
# Launch the viewer with a scripted target ship.
#
# Usage: ./run.sh [speed] [course]
#   speed   sim seconds per real second (default 5)
#   course  a .course file, or just its name, e.g. "patrol" for
#           scenario/patrol.course (default: crossing)
#           use "none" to run the ASV alone, with no target
set -euo pipefail
cd "$(dirname "$0")"

speed="${1:-5}"
course="${2:-crossing}"

args=("$speed")
if [[ "$course" != "none" ]]; then
  if [[ -f "$course" ]]; then
    path="$course"
  elif [[ -f "scenario/$course.course" ]]; then
    path="scenario/$course.course"
  else
    echo "Course '$course' not found. Available:" >&2
    ls scenario/*.course 2>/dev/null | sed 's|scenario/||; s|\.course$||; s|^|  |' >&2 || true
    exit 1
  fi
  args+=("$path")
fi

# Single-config builds (Linux/macOS) vs multi-config (Windows/MSVC)
for exe in build/viewer/sim_viewer build/viewer/Release/sim_viewer.exe; do
  if [[ -x "$exe" ]]; then
    exec "$exe" "${args[@]}"
  fi
done

echo "sim_viewer not found. Run ./build.sh first." >&2
exit 1