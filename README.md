# Radar DSP — Software-Defined Signal Processor in C++17

[![CI](https://github.com/yanivcohen223/radar-dsp/actions/workflows/ci.yml/badge.svg)](https://github.com/yanivcohen223/radar-dsp/actions)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform: Linux](https://img.shields.io/badge/Platform-Linux-lightgrey.svg)](https://kernel.org)

A **production-grade** radar DSP pipeline written from scratch in C++17, running on a
real-time POSIX thread.  Implements the full processing chain from simulated ADC samples
through detection and tracking, with a live ncurses terminal dashboard and a 60-second
headless benchmark mode that produces JSON + CSV performance reports.

Built as a portfolio project targeting **aerospace/defense embedded** roles.

---

## Live Dashboard

```
┌─────────────────────────────────────────────────────────────────────┐
│  [ Range-Doppler Map (CPI #142) ]                                   │
│  ......::::::;;;+++***###@@@###***+++;;;::::::......                │
│  ....::::::;;;+++**####@@@@@@####**+++;;;::::::....                 │
│  ...::::;;;+++**####@@@@@@@@@####**+++;;:::::.....                  │
│  ......::::::;;;+++***###@@@###***+++;;;::::::......                │
│                                                                     │
│ DETECTIONS (3)              TRACKS (3)                              │
│ Range(m) Vel(m/s) SNR(dB)  ID   Range    Vel      Age  Stat         │
│ 5012.4   49.8     18.3     1    5011.2   49.9     44   Conf         │
│ 12003.1  -30.1    14.7     2    12002.7  -30.0    44   Conf         │
│ 3001.5   119.9    20.1     3    3002.1   119.8    44   Conf         │
│                                                                     │
│ PIPELINE LATENCY                                                    │
│ 1_adc               ████████████████  p99=  312µs                   │
│ 2_pulse_compress    ████████████████████████  p99=  891µs           │
│ 3_range_doppler     ████████  p99=  287µs                           │
│ 4_cfar              ████  p99=  143µs                               │
│ 5_tracker           ██  p99=   41µs                                 │
│                                                                     │
│ CPI: 5.0 Hz | Tracks: 3 | Uptime: 30s | Press q to quit             │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Overview

| Feature | Detail |
|---------|--------|
| Language | C++17, `-Wall -Wextra -Werror -O3 -pthread` |
| Build system | CMake 3.20+ |
| FFTW3 | Single-precision (`fftwf_*`), plans created once, reused per CPI |
| RT scheduling | POSIX `SCHED_FIFO` via `pthread_attr_setschedpolicy` |
| IPC | Lock-free SPSC ring buffer, `alignas(64)` cache-line padding |
| Timing | `clock_gettime(CLOCK_MONOTONIC)` + `clock_nanosleep(TIMER_ABSTIME)` |
| UI | ncurses 5 Hz dashboard: RD heatmap, detection table, latency bars |
| Tests | 19 Google Test cases covering every DSP stage |
| Benchmark | 60-second headless run → `benchmark_TIMESTAMP.json` + `timing_TIMESTAMP.csv` |
| Containerised | Docker ubuntu:22.04, `--privileged` for RT, volume-mounted output |

---

## Architecture

```
 ┌─────────────────────── RT Thread (SCHED_FIFO) ───────────────────────┐
 │                                                                      │
 │  AdcSimulator  ──▶  PulseCompressor  ──▶  RangeDopplerProcessor      │
 │  generate_cpi()     compress()×Np         process()                  │
 │  [I/Q + AWGN]       [FFTW3 MF]           [2D FFT + dB + fftshift]    │
 │                                               │                      │
 │                                          CfarDetector<CA>            │
 │                                          detect()                    │
 │                                               │                      │
 │                                        TrackManager                  │
 │                                        update() · get_tracks()       │
 │                                               │                      │
 │                              LockFreeQueue<PipelineOutput, 8>        │
 └───────────────────────────────────────────────┼──────────────────────┘
                                                 │ try_push (non-blocking)
                         ┌───────────────────────▼───────────────────────┐
                         │  Dashboard Thread (SCHED_OTHER, 5 Hz)         │
                         │  TerminalDashboard::update(output, cpi_rate)  │
                         │  ncurses: heatmap · detections · tracks · bars│
                         └───────────────────────────────────────────────┘
```

### Component map

```
include/radar/               src/
  adc_simulator.hpp    ←──   adc_simulator.cpp
  waveform_generator.hpp←──  waveform_generator.cpp
  pulse_compressor.hpp ←──   pulse_compressor.cpp
  range_doppler.hpp    ←──   range_doppler.cpp
  cfar_detector.hpp    ←──   cfar_detector.cpp          (CA + OS, if constexpr)
  rt_thread.hpp        ←──   rt_thread.cpp
  lock_free_queue.hpp        (header-only)
  track_manager.hpp    ←──   track_manager.cpp
  perf_counter.hpp     ←──   perf_counter.cpp
  radar_pipeline.hpp   ←──   radar_pipeline.cpp
  terminal_dashboard.hpp←──  terminal_dashboard.cpp
                             main.cpp
```

---

## Signal Processing Mathematics

### LFM Chirp (Waveform Generator)

A Linear Frequency Modulated (LFM) waveform sweeping bandwidth `B` over pulse width `τ`:

```
s(t) = exp(jπ k t²),    0 ≤ t < τ
k = B/τ  (chirp rate [Hz/s])
```

Baseband complex samples at rate `fs`:  `s[n] = exp(jπ k (n/fs)²)`.

### Matched Filter (Pulse Compressor)

Pulse compression via frequency-domain matched filtering:

```
Y[n] = IFFT{ FFT(rx[n]) · conj(FFT(ref[n])) }
```

Hann window applied to the reference before FFT to suppress range sidelobes.

**Compression gain** ≈ `B·τ` (time-bandwidth product).

**PSL** (Peak Sidelobe Level):

```
PSL = 20 log₁₀( max(|sidelobes|) / |mainlobe_peak| )   [dB]
```

Target: PSL < −13 dB (Hann), < −26 dB (Blackman-Harris).

### Range-Doppler Processing

2D FFT over the slow-time (pulse) dimension, with Hamming windowing:

```
RD[d, r] = FFT_Np { w_hamming[p] · Y_compressed[p, r] }
RD_dB[d, r] = 20 log₁₀(|RD[d, r]| + ε)
```

`fftshift` recentres zero-Doppler to row `Np/2` so approach/recede split naturally.

**Range resolution**: `δr = c / (2B)` ≈ 15 m at B = 10 MHz

**Velocity resolution**: `δv = λ · PRF / (2 Np)` where `λ = c / f_c`

### CA-CFAR Detection

Cell-Averaging CFAR threshold [Finn & Johnson, 1968]:

```
T = α · (1/N) Σ_ref  P_ref[i]
α = N · (Pfa^{−1/N} − 1)
```

OS-CFAR variant uses the `k`-th order statistic (75th percentile by default) for
robustness in clutter edges.

### α-β Track Filter

```
r̂(n)  = r̂(n−) + α · (z_r(n) − r̂(n−))       α = 0.6
v̂(n)  = v̂(n−) + β · (z_v(n) − v̂(n−))       β = 0.1
```

Association gate: normalised Euclidean distance in range-velocity space with
`gate_range = 50 m`, `gate_velocity = 5 m/s`.

---

## Real-Time Design

### Why SCHED_FIFO?

`SCHED_FIFO` removes the process from the CFS (Completely Fair Scheduler) ready queue.
Once running, it is only preempted by higher-priority RT threads or hardware interrupts.
This eliminates the millisecond-scale scheduling jitter inherent to `SCHED_OTHER` and
allows sub-100 µs worst-case CPI execution.

### Periodic Execution

```cpp
// Absolute-time sleep avoids drift accumulation
int64_t next_ns = now_ns() + period_ns_;
while (running_) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    jitter = now_ns() - next_ns;   // positive = late, negative = early
    work_();                        // process_one_cpi()
    next_ns += period_ns_;
}
```

Using `TIMER_ABSTIME` instead of a relative sleep prevents cumulative period drift
regardless of how long `work_()` takes.

### Lock-Free SPSC Queue

```
Producer (RT thread)               Consumer (dashboard thread)
─────────────────────              ──────────────────────────
alignas(64) head_;                 alignas(64) tail_;
store(release)                     load(acquire)
```

Each of `head_`, `tail_`, and `buffer_` occupies its own 64-byte cache line.
No mutexes, no `std::atomic_thread_fence` beyond what `acquire`/`release` provide.
`try_push` is a true non-blocking call — the RT thread never waits for the consumer.

---

## Performance (Reference: Docker, Ubuntu 22.04, 4-core VM)

| Metric | Value |
|--------|-------|
| CPI throughput | ~5 Hz (64 pulses × 200 µs PRT) |
| Pulse compression (p99) | < 1 ms |
| Full pipeline (p99) | < 2 ms |
| RT jitter (p99) | < 50 µs (Docker --privileged) |
| PSL (Hann) | < −13 dB |
| Pd (SNR = 15 dB, 3 targets) | > 95% |

*Run `--benchmark` to measure on your own hardware.*

---

## Build & Run

### Prerequisites

- Docker ≥ 24  (or Ubuntu 22.04 with `libfftw3-dev`, `libncurses-dev`, `libgtest-dev`)

### Docker (recommended)

```bash
# Build image and run live dashboard
docker compose up --build

# Headless benchmark (60 s)
docker compose run --rm radar_dsp ./radar_dsp --benchmark --no-rt

# Pass custom flags
docker compose run --rm radar_dsp \
    ./radar_dsp --targets 5 --snr 20 --pulses 128 --no-rt
```

### Native build

```bash
sudo apt-get install -y libfftw3-dev libncurses-dev cmake g++

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./radar_dsp --no-rt          # live dashboard (no RT privs needed)
./radar_dsp --benchmark --no-rt   # headless 60-s benchmark
```

### CLI reference

| Flag | Default | Description |
|------|---------|-------------|
| `--targets N` | 3 | Number of simulated targets |
| `--snr X` | 15.0 | Signal-to-noise ratio [dB] |
| `--pulses N` | 64 | Pulses per CPI |
| `--duration S` | 60 | Benchmark duration [s] |
| `--benchmark` | off | 60-second headless benchmark mode |
| `--no-rt` | off | Use SCHED_OTHER (no `--privileged` needed) |
| `--no-dashboard` | off | Print detections to stdout, no ncurses |
| `--output DIR` | `/radar-dsp/output` | Output directory for reports |

---

## Running Tests

```bash
cd build && ctest --output-on-failure
# or directly:
./run_tests
```

19 tests covering: pulse compression (PSL, ISL, gain), range-Doppler (DC bin, Doppler
bin, fftshift), CA/OS-CFAR (no-FA on noise, strong-target detect, range conversion,
empirical FAR), and tracker (initiation, drop, NN association, gate reject,
thread safety, active count).

---

## Benchmark Report

After running `--benchmark`:

```bash
# Generate markdown table + plots
python3 tools/benchmark_report.py

# Specify a particular run
python3 tools/benchmark_report.py --json output/benchmark_20240315_120000.json
```

Produces:
- `output/report_TIMESTAMP.md` — Markdown summary table
- `output/latency_hist.png`    — Per-stage p50/p95/p99/max bar chart
- `output/jitter_timeline.png` — Stage p99 latency horizontal bars
- `output/pd_snr.png`          — Pd vs SNR curve with measured operating point

---

## Project Structure

```
radar-dsp/
├── include/radar/          # Public headers (all includes use "radar/foo.hpp")
│   ├── adc_simulator.hpp
│   ├── cfar_detector.hpp
│   ├── lock_free_queue.hpp
│   ├── perf_counter.hpp
│   ├── pulse_compressor.hpp
│   ├── radar_pipeline.hpp
│   ├── range_doppler.hpp
│   ├── rt_thread.hpp
│   ├── target.hpp
│   ├── terminal_dashboard.hpp
│   ├── track_manager.hpp
│   └── waveform_generator.hpp
├── src/                    # Flat .cpp implementations
│   ├── main.cpp
│   ├── adc_simulator.cpp
│   ├── cfar_detector.cpp
│   ├── perf_counter.cpp
│   ├── pulse_compressor.cpp
│   ├── radar_pipeline.cpp
│   ├── range_doppler.cpp
│   ├── rt_thread.cpp
│   ├── terminal_dashboard.cpp
│   ├── track_manager.cpp
│   └── waveform_generator.cpp
├── tests/                  # 19 Google Test cases
│   ├── test_cfar.cpp
│   ├── test_pulse_compression.cpp
│   ├── test_range_doppler.cpp
│   └── test_tracker.cpp
├── tools/
│   └── benchmark_report.py # Post-processes JSON → markdown + plots
├── docs/
│   └── architecture.md     # Extended pipeline documentation
├── scripts/
│   ├── build.sh
│   ├── run.sh
│   └── benchmark.sh
├── output/                 # Generated reports (gitignored except .gitkeep)
├── Dockerfile
├── docker-compose.yml
├── CMakeLists.txt
└── .clang-format           # Google style, ColumnLimit 100
```

---

## What I Learned Building This

**POSIX real-time scheduling** — The gap between `SCHED_OTHER` and `SCHED_FIFO` is not
just about priority; `SCHED_OTHER` threads are subject to the CFS time-slice quantum
(typically 4 ms), which creates bursts of scheduling jitter completely invisible to
application-level code.  `clock_nanosleep(TIMER_ABSTIME)` is the correct primitive for
periodic tasks — relative sleeps accumulate drift.

**FFTW3 plan reuse** — Creating an FFTW plan involves profiling and memory allocation.
Calling `fftwf_plan_*` inside the processing loop would balloon latency from ~800 µs to
~50 ms per CPI.  Creating plans once at startup and reusing them for every pulse is the
only RT-safe approach.

**Cache-line padding for SPSC queues** — A naive `head_`/`tail_` pair in the same struct
causes false sharing: the producer's write to `head_` invalidates the consumer's cache
line containing `tail_`, and vice versa, creating ~5–15 ns of unnecessary inter-core
coherence traffic per operation.  `alignas(64)` on each eliminates this.

**`if constexpr` vs template specialisation** — Explicit template specialisations of
member functions declared *after* the class body is instantiated generate a hard
compiler error ("specialisation after instantiation").  Using `if constexpr` inside the
primary template avoids the separate-translation-unit specialisation problem entirely
while keeping the code readable and zero-overhead.

**PSL vs window function** — The Hann window reduces PSL from ~−13 dB (rectangular) to
~−32 dB, but costs ~1.8 dB in main-lobe SNR loss.  For detection-range trade-offs in a
cluttered environment the sidelobe reduction is almost always worth it.

