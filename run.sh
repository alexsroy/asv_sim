#!/usr/bin/env bash
# Launch the viewer with one or more scripted target ships.
#
# Usage: ./run.sh [speed] [course...]
#   speed    sim seconds per real second (default 5)
#   course   a .course file, or just its name, e.g. "patrol" for
#            scenario/patrol.course (default: crossing)
#            pass several names to spawn multiple targets
#            use "none" to run the ASV alone, with no target
#
# Examples:
#   ./run.sh 5
#   ./run.sh 5 crossing
#   ./run.sh 5 crossing parallel
#   ./run.sh 5 none
set -euo pipefail
cd "$(dirname "$0")"

speed="${1:-5}"
shift || true

resolve_course() {
  local course="$1"
  if [[ -f "$course" ]]; then
    printf '%s\n' "$course"
  elif [[ -f "scenario/$course.course" ]]; then
    printf '%s\n' "scenario/$course.course"
  else
    echo "Course '$course' not found. Available:" >&2
    ls scenario/*.course 2>/dev/null | sed 's|scenario/||; s|\.course$||; s|^|  |' >&2 || true
    exit 1
  fi
}

args=("$speed")
if [[ $# -eq 0 ]]; then
  args+=("$(resolve_course crossing)")
elif [[ $# -eq 1 && "$1" == "none" ]]; then
  :
else
  for course in "$@"; do
    args+=("$(resolve_course "$course")")
  done
fi

# Single-config builds (Linux/macOS) vs multi-config (Windows/MSVC)
for exe in build/viewer/sim_viewer build/viewer/Release/sim_viewer.exe; do
  if [[ -x "$exe" ]]; then
    exec "$exe" "${args[@]}"
  fi
done

echo "sim_viewer not found. Run ./build.sh first." >&2
exit 1
