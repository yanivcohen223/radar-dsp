#!/bin/bash
# Benchmark script — runs inside Docker container.
# Executes the pipeline in benchmark mode, then generates plots.
set -euo pipefail

OUTPUT_DIR="/radar-dsp/output"
mkdir -p "$OUTPUT_DIR"

echo "==> Running radar DSP benchmark..."
/radar-dsp/build/radar_dsp \
  --benchmark \
  --targets 5 \
  --snr 15 \
  --pulses 128 \
  --no-dashboard

echo "==> Generating plots..."
python3 /radar-dsp/tools/visualize.py "$OUTPUT_DIR"

echo "==> Benchmark complete. Results in $OUTPUT_DIR/"
ls -lh "$OUTPUT_DIR/"
