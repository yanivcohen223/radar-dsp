#!/bin/bash
# Build script — runs inside the Docker container
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"

echo "==> Configuring with CMake..."
cmake -B "$BUILD_DIR" \
      -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_PROFILING=ON \
      "$PROJECT_ROOT"

echo "==> Building..."
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

echo "==> Build complete: $BUILD_DIR/radar_dsp"
