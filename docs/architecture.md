# Architecture: Software-Defined Radar DSP Pipeline

## Overview

The pipeline runs on a **single real-time POSIX thread** (SCHED_FIFO, configurable priority)
that fires once per CPI (Coherent Processing Interval).  Each CPI produces a complete
Range-Doppler map, a set of CFAR detections, and updated track states.

Results are published via a **lock-free SPSC ring buffer** to a dashboard thread running
at 5 Hz.  The two threads share no locks except through the ring buffer's acquire/release
memory ordering.

```
 ┌─────────────────────── RT Thread (SCHED_FIFO) ───────────────────────┐
 │                                                                        │
 │  ADC Simulator  ──▶  Pulse Compressor  ──▶  Range-Doppler  ──▶  CFAR │
 │       │                    │                     │               │    │
 │  generate_cpi()       compress()×Np          process()        detect()│
 │  [complex I/Q]        [FFTW3 MF]           [2D FFT + dB]    [CA/OS]  │
 │                                                                   │    │
 │                         Track Manager ◀─────────────────────────┘    │
 │                           update()                                     │
 │                              │                                         │
 │                     LockFreeQueue::try_push(PipelineOutput)            │
 └────────────────────────────────────────────────────────────────────────┘
                                │
               ┌────────────────▼─────────────────┐
               │  Dashboard Thread (SCHED_OTHER)   │
               │                                   │
               │  LockFreeQueue::try_pop()          │
               │  TerminalDashboard::update() 5 Hz  │
               │  ncurses: heatmap + tables + bars  │
               └───────────────────────────────────┘
```

---

## Stage 1 — ADC Simulator (`adc_simulator.cpp`)

Generates complex I/Q baseband samples for a full CPI of `Np × Nr` samples.

**Signal model** for target `k` at range `R_k`, velocity `v_k`:

```
s_k(t, p) = A_k · exp(jπ k_r t²) · exp(j2π f_d,k · p · T_prt)
```

where:
- `A_k`    = amplitude from RCS and range equation
- `k_r`    = chirp rate = B/τ
- `f_d,k`  = Doppler frequency = 2 v_k / λ
- `p`      = pulse index

Additive complex Gaussian noise is injected at the configured SNR.

---

## Stage 2 — Pulse Compressor (`pulse_compressor.cpp`)

Performs **matched filtering** in the frequency domain using FFTW3:

```
Y[n] = IFFT{ FFT(rx[n]) · conj(FFT(ref[n])) }
```

A **Hann window** is applied to the reference chirp before FFT to reduce range sidelobes.

**Metrics** computed on the first compressed pulse per CPI:
- **PSL** (Peak Sidelobe Level): ratio of largest sidelobe to mainlobe peak
- **ISL** (Integrated Sidelobe Level): ratio of total sidelobe energy to mainlobe energy

FFTW3 plans are created once at construction with `FFTW_ESTIMATE` and reused for all
`Np` pulses per CPI — safe for RT use (no heap allocation after init).

---

## Stage 3 — Range-Doppler Processor (`range_doppler.cpp`)

Converts the Np × Nr compressed pulse matrix into a 2D Range-Doppler map:

1. Apply **Hamming window** along slow-time axis (pulse dimension) to reduce Doppler sidelobes
2. FFT across `Np` pulses per range bin
3. Convert magnitude to dB: `20·log10(|X[d,r]| + ε)`
4. `fftshift` along Doppler axis so DC (zero-velocity) appears at row `Np/2`

Output: `rd_map_db[Nd × Nr]` in dB, Doppler-shifted.

---

## Stage 4 — CFAR Detector (`cfar_detector.cpp`)

Two variants selectable via template parameter `CfarType::CA` or `CfarType::OS`:

**CA-CFAR** (Cell-Averaging):

```
T = α · (1/N) · Σ reference_cells
α = N · (Pfa^{-1/N} - 1)          [Finn & Johnson, 1968]
```

**OS-CFAR** (Ordered-Statistics):

```
T = α · x_{(k)}    k = ⌊0.75·N⌋   (75th percentile)
```

Guard cells (`G`) on each side of the Cell Under Test (CUT) are excluded from
the reference window to avoid target self-masking.  Detection if `CUT > T`.

Output: `Detection{range_m, velocity_ms, snr_db, range_bin, doppler_bin}`.

---

## Stage 5 — Track Manager (`track_manager.cpp`)

**Association**: nearest-neighbour with normalized Euclidean gate:

```
d = sqrt( (Δr/r_gate)² + (Δv/v_gate)² )   →  accept if d < 1.0
```

**State update**: α-β filter:

```
range_est   += α · (meas_range - range_est)        α = 0.6
velocity_est += β · (meas_vel   - velocity_est)     β = 0.1
```

**Track life-cycle**:
```
Tentative  ──(hit_count ≥ 2)──▶  Confirmed
Confirmed  ──(miss)────────────▶  Coasted
Coasted    ──(hit)─────────────▶  Confirmed
Any        ──(miss_count ≥ 3)──▶  Dropped  ──▶  removed
```

---

## Lock-Free Queue (`lock_free_queue.hpp`)

Single-Producer Single-Consumer ring buffer of capacity `2^N`.

- `head_` (producer-owned) and `tail_` (consumer-owned) are each padded to a 64-byte
  cache line to eliminate false sharing.
- `try_push`: stores with `memory_order_release`, fails silently when full.
- `try_pop`: loads with `memory_order_acquire`, returns `std::optional<T>`.

---

## RT Thread (`rt_thread.cpp`)

Periodic task using `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`:

```
next_wake = now + period_ns
loop:
    clock_nanosleep(TIMER_ABSTIME, next_wake)
    jitter = now() - next_wake          # measure lateness
    work()                              # process_one_cpi()
    next_wake += period_ns
```

- Priority ≥ 1: requests `SCHED_FIFO` via `pthread_attr_setschedpolicy`.
- Priority = 0: falls through to `SCHED_OTHER` (no `--privileged` needed).
- CPU affinity: `pthread_setaffinity_np` pins the thread to a specific core.
- Jitter samples accumulate in a `std::vector<int64_t>`; statistics computed
  on demand via `jitter_stats()`.

---

## Performance Counter (`perf_counter.cpp`)

Each pipeline stage calls `begin(name)` / `end(name)` which record
`clock_gettime(CLOCK_MONOTONIC)` deltas into a 1000-sample ring buffer per stage.

`compute_stats()` returns sorted percentile statistics (p50/p95/p99/max) for the
ncurses latency bars and for benchmark JSON export.

When built with `-DENABLE_PROFILING=ON` the destructor auto-exports a CSV
(`output/perf_TIMESTAMP.csv`).

---

## Directory Layout

```
radar-dsp/
├── include/radar/          # All public headers (installed as "radar/foo.hpp")
├── src/                    # Flat .cpp implementations + main.cpp
├── tests/                  # Google Test unit tests (19 test cases)
├── tools/                  # Python post-processing scripts
├── docs/                   # Extended documentation
├── scripts/                # Docker helper scripts
└── output/                 # Generated CSVs, PNGs, benchmark JSON
```
