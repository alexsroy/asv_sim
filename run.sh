
#!/usr/bin/env bash
# Launch the viewer. Extra arguments are passed through to sim_viewer.
set -euo pipefail
cd "$(dirname "$0")"
 
# Single-config builds (Linux/macOS) vs multi-config (Windows/MSVC)
for exe in build/viewer/sim_viewer build/viewer/Release/sim_viewer.exe; do
  if [[ -x "$exe" ]]; then
    exec "$exe" "$@"
  fi
done
 
echo "sim_viewer not found. Run ./build.sh first." >&2
exit 1
 