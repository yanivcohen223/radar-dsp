#!/usr/bin/env python3
"""
visualize.py — Post-run analysis and plotting for the Radar DSP pipeline.

Reads the latest perf_*.csv from the output directory and generates:
  1. range_doppler.png  — Range-Doppler heatmap (placeholder if no rd data)
  2. latency.png        — Per-stage latency bar chart (p50 / p95 / p99)
  3. jitter.png         — Jitter timeline per thread (placeholder)

Usage:
    python3 tools/visualize.py [output_dir]
"""

import sys
import os
import glob
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


def find_latest_csv(output_dir: str) -> str | None:
    pattern = os.path.join(output_dir, "perf_*.csv")
    files = sorted(glob.glob(pattern))
    return files[-1] if files else None


def read_perf_csv(path: str) -> list[dict]:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "stage": row["stage_name"],
                "mean":  float(row["mean_us"]),
                "p50":   float(row["p50_us"]),
                "p95":   float(row["p95_us"]),
                "p99":   float(row["p99_us"]),
                "max":   float(row["max_us"]),
                "count": int(row["count"]),
            })
    return rows


# ─── Plot 1: Range-Doppler heatmap ───────────────────────────────────────────

def plot_range_doppler(output_dir: str) -> None:
    """Generate a synthetic Range-Doppler heatmap for illustration."""
    num_doppler = 64
    num_range = 256
    prt_s = 200e-6
    sample_rate = 10e6
    chirp_bw = 10e6
    c = 3e8

    # Axes
    range_res = c / (2 * chirp_bw)  # ~15 m
    prf = 1 / prt_s
    lambda_eff = c / (chirp_bw / 2)
    vel_res = lambda_eff * prf / (2 * num_doppler)  # m/s per bin

    ranges = np.arange(num_range) * range_res
    velocities = (np.arange(num_doppler) - num_doppler // 2) * vel_res

    # Noise floor
    rng = np.random.default_rng(42)
    rd = rng.standard_normal((num_doppler, num_range)) * 2.0 - 20.0  # dB

    # Inject three synthetic targets
    targets = [
        (30, 50, 35),   # (range_bin, doppler_bin, peak_dB)
        (15, 100, 30),
        (45, 180, 28),
    ]
    for (dbin, rbin, peak) in targets:
        for dd in range(-3, 4):
            for dr in range(-3, 4):
                di = (dbin + dd + num_doppler) % num_doppler
                ri = rbin + dr
                if 0 <= ri < num_range:
                    amp = peak - 3.0 * (dd ** 2 + dr ** 2) ** 0.5
                    rd[di, ri] = max(rd[di, ri], amp)

    fig, ax = plt.subplots(figsize=(12, 5))
    im = ax.imshow(
        rd,
        aspect="auto",
        origin="lower",
        extent=[ranges[0], ranges[-1], velocities[0], velocities[-1]],
        cmap="inferno",
        vmin=-30,
        vmax=35,
    )
    plt.colorbar(im, ax=ax, label="Power [dBFS]")
    ax.set_xlabel("Range [m]")
    ax.set_ylabel("Radial Velocity [m/s]")
    ax.set_title("Range-Doppler Map (64 pulses, Hamming window)")
    ax.axhline(0, color="cyan", linewidth=0.5, linestyle="--", label="Zero Doppler")
    ax.legend(loc="upper right", fontsize=8)

    out_path = os.path.join(output_dir, "range_doppler.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ─── Plot 2: Per-stage latency bar chart ─────────────────────────────────────

def plot_latency(rows: list[dict], output_dir: str) -> None:
    if not rows:
        print("  No perf data — skipping latency plot")
        return

    stages = [r["stage"] for r in rows]
    p50 = [r["p50"] for r in rows]
    p95 = [r["p95"] for r in rows]
    p99 = [r["p99"] for r in rows]

    x = np.arange(len(stages))
    width = 0.25

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x - width, p50, width, label="p50", color="#4caf50")
    ax.bar(x,          p95, width, label="p95", color="#ff9800")
    ax.bar(x + width,  p99, width, label="p99", color="#f44336")

    ax.set_xticks(x)
    ax.set_xticklabels([s.replace("_", "\n") for s in stages], fontsize=9)
    ax.set_ylabel("Latency [µs]")
    ax.set_title("Per-Stage Pipeline Latency")
    ax.legend()
    ax.grid(axis="y", alpha=0.4)

    # Annotate p99 values
    for xi, val in zip(x + width, p99):
        ax.text(xi, val + 0.5, f"{val:.0f}", ha="center", va="bottom", fontsize=8)

    out_path = os.path.join(output_dir, "latency.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ─── Plot 3: Jitter timeline ─────────────────────────────────────────────────

def plot_jitter(output_dir: str) -> None:
    """Generate a synthetic jitter timeline for illustration."""
    rng = np.random.default_rng(7)
    n = 200
    t = np.arange(n)
    # Simulate jitter: mostly small, occasional spikes
    jitter = np.abs(rng.normal(0, 15, n))  # µs
    jitter[rng.integers(0, n, 5)] += rng.uniform(40, 80, 5)  # spikes

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, jitter, linewidth=0.8, color="#2196f3", label="Wake-time jitter")
    ax.axhline(np.percentile(jitter, 99), color="#f44336", linestyle="--",
               label=f"p99 = {np.percentile(jitter, 99):.1f} µs")
    ax.axhline(np.mean(jitter), color="#4caf50", linestyle=":",
               label=f"mean = {np.mean(jitter):.1f} µs")
    ax.set_xlabel("CPI index")
    ax.set_ylabel("Jitter [µs]")
    ax.set_title("RT Thread Wake-Time Jitter (SCHED_FIFO)")
    ax.legend()
    ax.grid(alpha=0.3)

    out_path = os.path.join(output_dir, "jitter.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "/radar-dsp/output"
    os.makedirs(output_dir, exist_ok=True)

    print(f"Generating plots in: {output_dir}")

    # Load perf CSV if available
    csv_path = find_latest_csv(output_dir)
    rows: list[dict] = []
    if csv_path:
        print(f"  Using: {csv_path}")
        rows = read_perf_csv(csv_path)
    else:
        print("  No perf CSV found — using synthetic data")
        rows = [
            {"stage": "1_adc",            "mean": 120, "p50": 118, "p95": 135, "p99": 148, "max": 210, "count": 100},
            {"stage": "2_pulse_compress", "mean": 850, "p50": 845, "p95": 920, "p99": 980, "max": 1200, "count": 100},
            {"stage": "3_range_doppler",  "mean": 1100, "p50": 1090, "p95": 1180, "p99": 1250, "max": 1600, "count": 100},
            {"stage": "4_cfar",           "mean": 500, "p50": 495, "p95": 560, "p99": 610, "max": 800, "count": 100},
            {"stage": "5_tracker",        "mean": 30, "p50": 28, "p95": 45, "p99": 55, "max": 90, "count": 100},
        ]

    plot_range_doppler(output_dir)
    plot_latency(rows, output_dir)
    plot_jitter(output_dir)

    print("Done.")


if __name__ == "__main__":
    main()
