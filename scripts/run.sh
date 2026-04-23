#!/bin/bash
# Run the radar DSP binary inside a privileged Docker container.
# Privileged mode is required for SCHED_FIFO real-time scheduling.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

mkdir -p "$PROJECT_ROOT/output"

docker run --rm -it \
  --privileged \
  --ulimit rtprio=99 \
  -e TERM=xterm-256color \
  -v "$PROJECT_ROOT/output":/radar-dsp/output \
  radar-dsp \
  ./build/radar_dsp "$@"
