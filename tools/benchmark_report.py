#!/usr/bin/env python3
"""
benchmark_report.py — Post-process radar-dsp benchmark results.

Usage:
    python3 tools/benchmark_report.py [--json PATH] [--out DIR]

If --json is omitted the script finds the newest benchmark_*.json in
output/.  Generates:
  - output/report_TIMESTAMP.md    — Markdown summary table
  - output/latency_hist.png       — Per-stage latency histogram (p50/p95/p99/max)
  - output/jitter_timeline.png    — RT jitter timeline (when CSV present)
  - output/pd_snr.png             — Pd vs SNR curve (parametric from JSON)
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from datetime import datetime


# ─── Graceful matplotlib import ──────────────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print("[warn] matplotlib/numpy not found — skipping plots", file=sys.stderr)


# ─── Helpers ──────────────────────────────────────────────────────────────────

def find_latest_json(output_dir: str) -> str:
    pattern = os.path.join(output_dir, "benchmark_*.json")
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No benchmark JSON files found in {output_dir}")
    return files[-1]


def load_json(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def find_matching_csv(json_path: str, output_dir: str) -> str | None:
    # timing_TIMESTAMP.csv shares the same timestamp as benchmark_TIMESTAMP.json
    ts = os.path.basename(json_path).replace("benchmark_", "").replace(".json", "")
    csv_path = os.path.join(output_dir, f"timing_{ts}.csv")
    return csv_path if os.path.exists(csv_path) else None


# ─── Markdown report ─────────────────────────────────────────────────────────

def build_markdown(data: dict) -> str:
    meta = data.get("meta", {})
    det = data.get("detection", {})
    thr = data.get("throughput", {})
    wav = data.get("waveform", {})
    jit = data.get("rt_jitter_ns", {})
    stages = data.get("stage_latency_us", [])

    lines: list[str] = []
    lines.append("# Radar-DSP Benchmark Report\n")
    lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    lines.append("## Configuration\n")
    lines.append("| Parameter | Value |")
    lines.append("|-----------|-------|")
    lines.append(f"| Duration | {meta.get('duration_s', '?')} s |")
    lines.append(f"| Targets | {meta.get('num_targets', '?')} |")
    lines.append(f"| SNR | {meta.get('snr_db', '?')} dB |")
    lines.append(f"| Pulses/CPI | {meta.get('num_pulses', '?')} |")
    lines.append(f"| RT scheduling | {'SCHED_FIFO' if meta.get('rt_enabled') else 'SCHED_OTHER'} |")
    lines.append("")

    lines.append("## Waveform Quality\n")
    lines.append("| Metric | Value |")
    lines.append("|--------|-------|")
    lines.append(f"| Mean PSL | {wav.get('mean_psl_db', 0):.1f} dB |")
    lines.append(f"| Mean ISL | {wav.get('mean_isl_db', 0):.1f} dB |")
    lines.append("")

    lines.append("## Detection Performance\n")
    pd_pct = det.get("probability_of_detection", 0) * 100
    far = det.get("false_alarm_rate", 0)
    lines.append("| Metric | Value |")
    lines.append("|--------|-------|")
    lines.append(f"| Probability of Detection (Pd) | {pd_pct:.1f}% |")
    lines.append(f"| False Alarm Rate | {far:.2e} /cell |")
    lines.append(f"| True Detections | {det.get('true_detections', 0):,} |")
    lines.append(f"| False Detections | {det.get('false_detections', 0):,} |")
    lines.append(f"| Total CPIs | {det.get('total_cpis', 0):,} |")
    lines.append("")

    lines.append("## Throughput\n")
    lines.append("| Metric | Value |")
    lines.append("|--------|-------|")
    lines.append(f"| CPI Throughput | {thr.get('cpi_per_second', 0):.2f} CPI/s |")
    lines.append(f"| Elapsed Time | {thr.get('elapsed_s', 0):.1f} s |")
    lines.append("")

    lines.append("## Per-Stage Latency (µs)\n")
    lines.append("| Stage | Mean | p50 | p95 | p99 | Max | Samples |")
    lines.append("|-------|------|-----|-----|-----|-----|---------|")
    for s in stages:
        lines.append(
            f"| {s['stage']} "
            f"| {s['mean']:.1f} "
            f"| {s['p50']:.1f} "
            f"| {s['p95']:.1f} "
            f"| {s['p99']:.1f} "
            f"| {s['max']:.1f} "
            f"| {s['samples']:,} |"
        )
    lines.append("")

    if jit.get("samples", 0) > 0:
        lines.append("## RT Thread Jitter (ns)\n")
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")
        lines.append(f"| Min | {jit.get('min', 0):,} ns |")
        lines.append(f"| Mean | {jit.get('mean', 0):.0f} ns |")
        lines.append(f"| p99 | {jit.get('p99', 0):,} ns |")
        lines.append(f"| Max | {jit.get('max', 0):,} ns |")
        lines.append("")

    if HAS_PLOT:
        lines.append("## Plots\n")
        lines.append("![Latency Histogram](latency_hist.png)\n")
        lines.append("![Jitter Timeline](jitter_timeline.png)\n")

    return "\n".join(lines)


# ─── Plots ────────────────────────────────────────────────────────────────────

def plot_latency_histogram(stages: list[dict], out_path: str) -> None:
    if not HAS_PLOT or not stages:
        return

    names = [s["stage"].replace("_", "\n") for s in stages]
    p50  = [s["p50"]  for s in stages]
    p95  = [s["p95"]  for s in stages]
    p99  = [s["p99"]  for s in stages]
    maxv = [s["max"]  for s in stages]

    x = np.arange(len(names))
    width = 0.2

    fig, ax = plt.subplots(figsize=(max(8, len(names) * 1.5), 5))
    ax.bar(x - 1.5 * width, p50,  width, label="p50",  color="#4caf50")
    ax.bar(x - 0.5 * width, p95,  width, label="p95",  color="#2196f3")
    ax.bar(x + 0.5 * width, p99,  width, label="p99",  color="#ff9800")
    ax.bar(x + 1.5 * width, maxv, width, label="max",  color="#f44336")

    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("Latency (µs)")
    ax.set_title("Pipeline Stage Latency Distribution")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_jitter_timeline(csv_path: str, out_path: str) -> None:
    """Read timing CSV and render a jitter timeline for the RT thread."""
    if not HAS_PLOT or not csv_path or not os.path.exists(csv_path):
        return
    try:
        import csv as csv_mod
        rows = []
        with open(csv_path) as f:
            reader = csv_mod.DictReader(f)
            for row in reader:
                rows.append(row)
        if not rows:
            return

        # The CSV has aggregate stats; plot p99 as a bar chart per stage
        stages = [r["stage"] for r in rows]
        p99 = [float(r["p99_us"]) for r in rows]

        fig, ax = plt.subplots(figsize=(max(8, len(stages) * 1.2), 4))
        colors = ["#e53935" if v > 1000 else "#43a047" for v in p99]
        ax.barh(stages, p99, color=colors)
        ax.set_xlabel("p99 Latency (µs)")
        ax.set_title("Per-Stage p99 Latency (from CSV)")
        ax.axvline(x=1000, color="red", linestyle="--", alpha=0.5, label="1 ms")
        ax.legend()
        ax.grid(axis="x", alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f"  Saved: {out_path}")
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[warn] jitter timeline plot failed: {exc}", file=sys.stderr)


def plot_pd_vs_snr(pd_measured: float, snr_db: float, out_path: str) -> None:
    """Parametric Pd vs SNR curve (Albersheim approximation) + measured point."""
    if not HAS_PLOT:
        return

    # Albersheim approximation: Pd = 1 - exp(-exp(a + b * ln(SNR_linear)))
    # Use a simplified sigmoid model for illustration
    snr_range = np.linspace(-5, 30, 200)
    # Swerling-0 (non-fluctuating) threshold from Marcum Q-function approximation:
    # Pd ≈ 0.5 * erfc((threshold_factor - snr_lin) / sqrt(2))
    # We use the logistic curve as a visual stand-in
    pfa = 1e-6
    threshold_db = 10 * np.log10(-np.log(pfa))  # rough CF-CFAR threshold
    pd_curve = 1.0 / (1.0 + np.exp(-(snr_range - threshold_db) * 0.5))

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(snr_range, pd_curve * 100, "b-", linewidth=2, label="Theoretical (approx)")
    ax.scatter([snr_db], [pd_measured * 100], color="red", s=80, zorder=5,
               label=f"Measured ({pd_measured*100:.1f}% @ {snr_db:.0f} dB)")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("Probability of Detection (%)")
    ax.set_title("Pd vs SNR")
    ax.set_ylim(0, 105)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Radar-DSP benchmark report generator")
    parser.add_argument("--json", default=None, help="Path to benchmark JSON (default: latest)")
    parser.add_argument("--out", default="output", help="Output directory (default: output/)")
    args = parser.parse_args()

    output_dir = args.out
    json_path = args.json or find_latest_json(output_dir)
    print(f"Reading: {json_path}")

    data = load_json(json_path)
    csv_path = find_matching_csv(json_path, output_dir)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Markdown report
    md = build_markdown(data)
    md_path = os.path.join(output_dir, f"report_{ts}.md")
    with open(md_path, "w") as f:
        f.write(md)
    print(f"  Saved: {md_path}")

    if HAS_PLOT:
        stages = data.get("stage_latency_us", [])
        plot_latency_histogram(stages, os.path.join(output_dir, "latency_hist.png"))
        plot_jitter_timeline(csv_path, os.path.join(output_dir, "jitter_timeline.png"))

        det = data.get("detection", {})
        meta = data.get("meta", {})
        plot_pd_vs_snr(
            det.get("probability_of_detection", 0.0),
            meta.get("snr_db", 15.0),
            os.path.join(output_dir, "pd_snr.png"),
        )

    print("Done.")


if __name__ == "__main__":
    main()
