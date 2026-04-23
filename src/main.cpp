/**
 * @file main.cpp
 * @brief Entry point for the Software-Defined Radar DSP processor.
 *
 * CLI flags:
 *   --targets N     Number of simulated targets (default: 3)
 *   --snr X         Signal-to-noise ratio in dB (default: 15.0)
 *   --pulses N      Pulses per CPI (default: 64)
 *   --benchmark     Run in benchmark mode (no dashboard, exit after 100 CPIs)
 *   --no-rt         Disable SCHED_FIFO (useful outside Docker --privileged)
 *   --no-dashboard  Run without ncurses (print detections to stdout)
 *   --output DIR    Output directory for CSVs (default: /radar-dsp/output)
 */

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "dashboard/terminal_dashboard.hpp"
#include "pipeline/radar_pipeline.hpp"

// ─── Signal handling ─────────────────────────────────────────────────────────
static volatile sig_atomic_t g_shutdown = 0;
static void handle_signal(int /*sig*/) { g_shutdown = 1; }

// ─── CLI parsing ─────────────────────────────────────────────────────────────
struct CliArgs {
  int num_targets{3};
  float snr_db{15.0f};
  uint32_t num_pulses{64};
  bool benchmark{false};
  bool no_rt{false};
  bool no_dashboard{false};
  std::string output_dir{"/radar-dsp/output"};
};

static CliArgs parse_args(int argc, char** argv) {
  CliArgs a{};
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--targets") == 0 && i + 1 < argc) {
      a.num_targets = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
      a.snr_db = static_cast<float>(std::atof(argv[++i]));
    } else if (std::strcmp(argv[i], "--pulses") == 0 && i + 1 < argc) {
      a.num_pulses = static_cast<uint32_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--benchmark") == 0) {
      a.benchmark = true;
    } else if (std::strcmp(argv[i], "--no-rt") == 0) {
      a.no_rt = true;
    } else if (std::strcmp(argv[i], "--no-dashboard") == 0) {
      a.no_dashboard = true;
    } else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
      a.output_dir = argv[++i];
    }
  }
  return a;
}

// ─── Default simulated targets ───────────────────────────────────────────────
static std::vector<TargetConfig> make_targets(int n, float snr_db) {
  // Pre-defined targets at various ranges and velocities
  static const std::vector<TargetConfig> presets = {
      {5000.0f, 50.0f, 0.0f},
      {12000.0f, -30.0f, -3.0f},
      {3000.0f, 120.0f, 5.0f},
      {8000.0f, 0.0f, -6.0f},
      {15000.0f, -80.0f, 2.0f},
  };
  (void)snr_db;
  std::vector<TargetConfig> targets;
  for (int i = 0; i < n && i < static_cast<int>(presets.size()); ++i) {
    targets.push_back(presets[static_cast<std::size_t>(i)]);
  }
  return targets;
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const auto args = parse_args(argc, argv);

  // ── Build CPI config ──────────────────────────────────────────────────────
  CpiConfig cpi{};
  cpi.num_pulses = args.num_pulses;
  cpi.snr_db = args.snr_db;
  // range_bins set to power-of-two ≥ pulse_samples for efficient FFT
  cpi.range_bins = 1024;

  // ── Build pipeline ────────────────────────────────────────────────────────
  PipelineConfig pcfg{};
  pcfg.cpi = cpi;
  pcfg.enable_rt = !args.no_rt;
  pcfg.rt_priority = 50;
  pcfg.output_dir = args.output_dir;
  pcfg.benchmark_mode = args.benchmark;

  auto targets = make_targets(args.num_targets, args.snr_db);

  std::cout << "=== Radar DSP Processor ===\n"
            << "  Targets : " << targets.size() << "\n"
            << "  SNR     : " << args.snr_db << " dB\n"
            << "  Pulses  : " << cpi.num_pulses << "\n"
            << "  RT      : " << (!args.no_rt ? "SCHED_FIFO" : "disabled") << "\n"
            << "  Mode    : " << (args.benchmark ? "benchmark" : "live") << "\n"
            << std::flush;

  RadarPipeline pipeline(pcfg, targets);
  pipeline.start();

  // ── Benchmark mode ────────────────────────────────────────────────────────
  if (args.benchmark || args.no_dashboard) {
    constexpr uint64_t kBenchmarkCpis = 100;
    std::cout << "Running " << kBenchmarkCpis << " CPIs...\n";

    while (!g_shutdown) {
      auto out = pipeline.pop_output();
      if (out) {
        if (!args.no_dashboard) {
          std::printf("CPI %lu: %zu detections, %zu tracks\n",
                      static_cast<unsigned long>(out->cpi_index),
                      out->detections.size(), out->tracks.size());
        }
        if (out->cpi_index >= kBenchmarkCpis) break;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    pipeline.stop();
    pipeline.perf().export_csv();
    std::cout << "==> Performance CSV written to " << args.output_dir << "/\n";
    return 0;
  }

  // ── Live dashboard mode ───────────────────────────────────────────────────
  TerminalDashboard dashboard(&pipeline.perf());

  auto last_time = std::chrono::steady_clock::now();
  uint64_t last_cpi = 0;
  constexpr auto kRefreshPeriod = std::chrono::milliseconds(200);  // 5 Hz

  while (!g_shutdown && !dashboard.should_quit()) {
    auto now = std::chrono::steady_clock::now();
    if (now - last_time >= kRefreshPeriod) {
      const uint64_t current_cpi = pipeline.cpi_count();
      const double elapsed =
          std::chrono::duration<double>(now - last_time).count();
      const float cpi_rate =
          static_cast<float>(current_cpi - last_cpi) / static_cast<float>(elapsed);

      last_time = now;
      last_cpi = current_cpi;

      // Drain the output queue and show the latest
      std::optional<PipelineOutput> latest;
      while (auto out = pipeline.pop_output()) {
        latest = std::move(out);
      }
      if (latest) {
        dashboard.update(*latest, cpi_rate);
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  pipeline.stop();
  pipeline.perf().export_csv();
  return 0;
}
