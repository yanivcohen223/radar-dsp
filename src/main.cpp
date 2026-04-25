/**
 * @file main.cpp
 * @brief Entry point for the Software-Defined Radar DSP processor.
 *
 * CLI flags:
 *   --targets N     Number of simulated targets (default: 3)
 *   --snr X         Signal-to-noise ratio in dB (default: 15.0)
 *   --pulses N      Pulses per CPI (default: 64)
 *   --duration S    Benchmark duration in seconds (default: 60)
 *   --benchmark     Run 60-second headless benchmark, emit JSON + CSV
 *   --no-rt         Disable SCHED_FIFO (useful outside Docker --privileged)
 *   --no-dashboard  Run without ncurses, print detections to stdout
 *   --output DIR    Output directory for results (default: /radar-dsp/output)
 */

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "radar/radar_pipeline.hpp"
#include "radar/terminal_dashboard.hpp"

// ─── Signal handling ─────────────────────────────────────────────────────────
static volatile sig_atomic_t g_shutdown = 0;
static void handle_signal(int /*sig*/) { g_shutdown = 1; }

// ─── CLI parsing ─────────────────────────────────────────────────────────────
struct CliArgs {
  int num_targets{3};
  float snr_db{15.0f};
  uint32_t num_pulses{64};
  int duration_s{60};
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
    } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
      a.duration_s = std::atoi(argv[++i]);
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
  static const std::vector<TargetConfig> presets = {
      {5000.0f,  50.0f,   0.0f},
      {12000.0f, -30.0f, -3.0f},
      {3000.0f,  120.0f,  5.0f},
      {8000.0f,  0.0f,   -6.0f},
      {15000.0f, -80.0f,  2.0f},
  };
  (void)snr_db;
  std::vector<TargetConfig> targets;
  targets.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n && i < static_cast<int>(presets.size()); ++i) {
    targets.push_back(presets[static_cast<std::size_t>(i)]);
  }
  return targets;
}

// ─── Timestamp string ────────────────────────────────────────────────────────
static std::string timestamp_str() {
  std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
  return std::string(buf);
}

// ─── Check if a detection is near an injected target ─────────────────────────
static bool detection_matches_target(const Detection& det, const TargetConfig& tgt,
                                     float range_tol_m, float vel_tol_ms) {
  return std::fabs(det.range_m - tgt.range_m) <= range_tol_m &&
         std::fabs(det.velocity_ms - tgt.velocity_ms) <= vel_tol_ms;
}

// ─── Benchmark accumulator ────────────────────────────────────────────────────
struct BenchmarkAccum {
  // PSL / ISL (running average across CPIs)
  double psl_sum{0.0};
  double isl_sum{0.0};
  uint64_t psl_count{0};

  // Detection statistics
  uint64_t true_detections{0};   ///< Detections within gate of an injected target
  uint64_t false_detections{0};  ///< All other detections
  uint64_t total_target_opportunities{0};  ///< targets × CPIs processed

  // Timing
  std::chrono::steady_clock::time_point t_start;
  std::chrono::steady_clock::time_point t_end;
  uint64_t cpi_count{0};
};

// ─── Write benchmark JSON ─────────────────────────────────────────────────────
static void write_benchmark_json(const std::string& path,
                                 const BenchmarkAccum& acc,
                                 const std::vector<StageStats>& stage_stats,
                                 const JitterStats& jitter,
                                 const std::vector<TargetConfig>& targets,
                                 const CliArgs& args) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "Cannot open %s for writing\n", path.c_str());
    return;
  }

  const double elapsed_s =
      std::chrono::duration<double>(acc.t_end - acc.t_start).count();
  const double cpi_throughput = static_cast<double>(acc.cpi_count) / elapsed_s;
  const double pd = acc.total_target_opportunities > 0
                        ? static_cast<double>(acc.true_detections) /
                              static_cast<double>(acc.total_target_opportunities)
                        : 0.0;
  const double total_cells =
      static_cast<double>(acc.cpi_count) *
      static_cast<double>(args.num_pulses) *
      1024.0 *  // range_bins
      static_cast<double>(targets.size());
  const double far = total_cells > 0.0
                         ? static_cast<double>(acc.false_detections) / total_cells
                         : 0.0;
  const double mean_psl = acc.psl_count > 0 ? acc.psl_sum / acc.psl_count : 0.0;
  const double mean_isl = acc.psl_count > 0 ? acc.isl_sum / acc.psl_count : 0.0;

  std::fprintf(f, "{\n");
  std::fprintf(f, "  \"meta\": {\n");
  std::fprintf(f, "    \"duration_s\": %d,\n", args.duration_s);
  std::fprintf(f, "    \"num_targets\": %d,\n", args.num_targets);
  std::fprintf(f, "    \"snr_db\": %.1f,\n", static_cast<double>(args.snr_db));
  std::fprintf(f, "    \"num_pulses\": %u,\n", args.num_pulses);
  std::fprintf(f, "    \"rt_enabled\": %s\n", args.no_rt ? "false" : "true");
  std::fprintf(f, "  },\n");

  std::fprintf(f, "  \"waveform\": {\n");
  std::fprintf(f, "    \"mean_psl_db\": %.2f,\n", mean_psl);
  std::fprintf(f, "    \"mean_isl_db\": %.2f\n", mean_isl);
  std::fprintf(f, "  },\n");

  std::fprintf(f, "  \"detection\": {\n");
  std::fprintf(f, "    \"probability_of_detection\": %.4f,\n", pd);
  std::fprintf(f, "    \"false_alarm_rate\": %.2e,\n", far);
  std::fprintf(f, "    \"true_detections\": %llu,\n",
               static_cast<unsigned long long>(acc.true_detections));
  std::fprintf(f, "    \"false_detections\": %llu,\n",
               static_cast<unsigned long long>(acc.false_detections));
  std::fprintf(f, "    \"total_cpis\": %llu\n",
               static_cast<unsigned long long>(acc.cpi_count));
  std::fprintf(f, "  },\n");

  std::fprintf(f, "  \"throughput\": {\n");
  std::fprintf(f, "    \"cpi_per_second\": %.2f,\n", cpi_throughput);
  std::fprintf(f, "    \"elapsed_s\": %.3f\n", elapsed_s);
  std::fprintf(f, "  },\n");

  std::fprintf(f, "  \"stage_latency_us\": [\n");
  for (std::size_t i = 0; i < stage_stats.size(); ++i) {
    const auto& s = stage_stats[i];
    std::fprintf(f, "    {\n");
    std::fprintf(f, "      \"stage\": \"%s\",\n", s.name.c_str());
    std::fprintf(f, "      \"mean\": %.2f,\n", s.mean_us);
    std::fprintf(f, "      \"p50\": %.2f,\n", s.p50_us);
    std::fprintf(f, "      \"p95\": %.2f,\n", s.p95_us);
    std::fprintf(f, "      \"p99\": %.2f,\n", s.p99_us);
    std::fprintf(f, "      \"max\": %.2f,\n", s.max_us);
    std::fprintf(f, "      \"samples\": %zu\n", s.count);
    const bool last = (i + 1 == stage_stats.size());
    std::fprintf(f, "    }%s\n", last ? "" : ",");
  }
  std::fprintf(f, "  ],\n");

  std::fprintf(f, "  \"rt_jitter_ns\": {\n");
  std::fprintf(f, "    \"min\": %lld,\n", static_cast<long long>(jitter.min_ns));
  std::fprintf(f, "    \"mean\": %.1f,\n", jitter.mean_ns);
  std::fprintf(f, "    \"p99\": %lld,\n", static_cast<long long>(jitter.p99_ns));
  std::fprintf(f, "    \"max\": %lld,\n", static_cast<long long>(jitter.max_ns));
  std::fprintf(f, "    \"samples\": %zu\n", jitter.sample_count);
  std::fprintf(f, "  }\n");
  std::fprintf(f, "}\n");

  std::fclose(f);
}

// ─── Write timing CSV ─────────────────────────────────────────────────────────
static void write_timing_csv(const std::string& path,
                             const std::vector<StageStats>& stats) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "Cannot open %s for writing\n", path.c_str());
    return;
  }
  std::fprintf(f, "stage,mean_us,p50_us,p95_us,p99_us,max_us,samples\n");
  for (const auto& s : stats) {
    std::fprintf(f, "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%zu\n",
                 s.name.c_str(), s.mean_us, s.p50_us, s.p95_us, s.p99_us,
                 s.max_us, s.count);
  }
  std::fclose(f);
}

// ─── Benchmark mode ───────────────────────────────────────────────────────────
static int run_benchmark(RadarPipeline& pipeline,
                         const std::vector<TargetConfig>& targets,
                         const CliArgs& args) {
  // Range/velocity tolerances for Pd counting (one resolution cell)
  static constexpr float kC = 3.0e8f;
  static constexpr float kBw = 10e6f;
  const float range_res = kC / (2.0f * kBw);          // ~15 m
  const float fc_eff = kBw / 2.0f;
  const float lambda = kC / fc_eff;
  const float prf = 1.0f / 200e-6f;
  const float vel_res =
      lambda * prf / (2.0f * static_cast<float>(args.num_pulses));

  const float range_tol = 2.0f * range_res;
  const float vel_tol = 2.0f * vel_res;

  BenchmarkAccum acc{};
  acc.t_start = std::chrono::steady_clock::now();

  const auto deadline =
      acc.t_start + std::chrono::seconds(args.duration_s);

  std::printf("Benchmark: running for %d seconds...\n", args.duration_s);
  std::fflush(stdout);

  while (!g_shutdown) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    auto out = pipeline.pop_output();
    if (!out) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // PSL / ISL
    if (out->psl_db != 0.0f) {
      acc.psl_sum += static_cast<double>(out->psl_db);
      acc.isl_sum += static_cast<double>(out->isl_db);
      ++acc.psl_count;
    }

    // Pd / FAR accounting
    acc.total_target_opportunities += targets.size();
    std::vector<bool> det_is_true(out->detections.size(), false);

    for (const auto& tgt : targets) {
      for (std::size_t di = 0; di < out->detections.size(); ++di) {
        if (detection_matches_target(out->detections[di], tgt, range_tol, vel_tol)) {
          det_is_true[di] = true;
          ++acc.true_detections;
          break;
        }
      }
    }
    for (std::size_t di = 0; di < out->detections.size(); ++di) {
      if (!det_is_true[di]) ++acc.false_detections;
    }

    ++acc.cpi_count;

    // Progress tick every 10 CPIs
    if (acc.cpi_count % 10 == 0) {
      const double elapsed =
          std::chrono::duration<double>(now - acc.t_start).count();
      std::printf("  CPI %llu  (%.1f s elapsed, %.1f CPI/s)\r",
                  static_cast<unsigned long long>(acc.cpi_count),
                  elapsed,
                  static_cast<double>(acc.cpi_count) / elapsed);
      std::fflush(stdout);
    }
  }

  acc.t_end = std::chrono::steady_clock::now();
  pipeline.stop();

  std::printf("\n");

  const auto stage_stats = pipeline.perf().compute_stats();
  const auto jitter = pipeline.jitter_stats();

  // Write outputs
  std::filesystem::create_directories(args.output_dir);
  const std::string ts = timestamp_str();
  const std::string json_path = args.output_dir + "/benchmark_" + ts + ".json";
  const std::string csv_path = args.output_dir + "/timing_" + ts + ".csv";

  write_benchmark_json(json_path, acc, stage_stats, jitter, targets, args);
  write_timing_csv(csv_path, stage_stats);

  // Summary to stdout
  const double elapsed_s =
      std::chrono::duration<double>(acc.t_end - acc.t_start).count();
  const double pd = acc.total_target_opportunities > 0
                        ? static_cast<double>(acc.true_detections) /
                              static_cast<double>(acc.total_target_opportunities)
                        : 0.0;
  const double total_cells =
      static_cast<double>(acc.cpi_count) *
      static_cast<double>(args.num_pulses) * 1024.0 *
      static_cast<double>(targets.size());
  const double far = total_cells > 0.0
                         ? static_cast<double>(acc.false_detections) / total_cells
                         : 0.0;

  std::printf("\n=== Benchmark Results ===\n");
  std::printf("  CPIs processed : %llu\n",
              static_cast<unsigned long long>(acc.cpi_count));
  std::printf("  Throughput     : %.2f CPI/s\n",
              static_cast<double>(acc.cpi_count) / elapsed_s);
  std::printf("  Mean PSL       : %.1f dB\n",
              acc.psl_count > 0 ? acc.psl_sum / acc.psl_count : 0.0);
  std::printf("  Mean ISL       : %.1f dB\n",
              acc.psl_count > 0 ? acc.isl_sum / acc.psl_count : 0.0);
  std::printf("  Pd             : %.1f%%\n", pd * 100.0);
  std::printf("  FAR            : %.2e /cell\n", far);
  if (jitter.sample_count > 0) {
    std::printf("  Jitter p99     : %lld ns\n",
                static_cast<long long>(jitter.p99_ns));
  }
  std::printf("\nOutputs:\n");
  std::printf("  JSON  → %s\n", json_path.c_str());
  std::printf("  CSV   → %s\n", csv_path.c_str());

  return 0;
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
  cpi.range_bins = 1024;

  // ── Build pipeline config ─────────────────────────────────────────────────
  PipelineConfig pcfg{};
  pcfg.cpi = cpi;
  pcfg.enable_rt = !args.no_rt;
  pcfg.rt_priority = 50;
  pcfg.output_dir = args.output_dir;
  pcfg.benchmark_mode = args.benchmark;

  auto targets = make_targets(args.num_targets, args.snr_db);

  std::printf("=== Radar DSP Processor ===\n"
              "  Targets  : %d\n"
              "  SNR      : %.1f dB\n"
              "  Pulses   : %u\n"
              "  RT       : %s\n"
              "  Mode     : %s\n",
              static_cast<int>(targets.size()),
              static_cast<double>(args.snr_db),
              args.num_pulses,
              args.no_rt ? "SCHED_OTHER" : "SCHED_FIFO",
              args.benchmark ? "benchmark" : args.no_dashboard ? "headless" : "live");
  std::fflush(stdout);

  RadarPipeline pipeline(pcfg, targets);
  pipeline.start();

  // ── Benchmark mode ────────────────────────────────────────────────────────
  if (args.benchmark) {
    return run_benchmark(pipeline, targets, args);
  }

  // ── Headless mode (no ncurses) ────────────────────────────────────────────
  if (args.no_dashboard) {
    while (!g_shutdown) {
      auto out = pipeline.pop_output();
      if (out) {
        std::printf("CPI %llu: %zu detections, %zu tracks  PSL=%.1f dB\n",
                    static_cast<unsigned long long>(out->cpi_index),
                    out->detections.size(), out->tracks.size(),
                    static_cast<double>(out->psl_db));
        std::fflush(stdout);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    pipeline.stop();
    pipeline.perf().export_csv();
    return 0;
  }

  // ── Live ncurses dashboard ────────────────────────────────────────────────
  TerminalDashboard dashboard(&pipeline.perf());

  auto last_time = std::chrono::steady_clock::now();
  uint64_t last_cpi = 0;
  constexpr auto kRefreshPeriod = std::chrono::milliseconds(200);  // 5 Hz

  while (!g_shutdown && !dashboard.should_quit()) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_time >= kRefreshPeriod) {
      const uint64_t current_cpi = pipeline.cpi_count();
      const double elapsed =
          std::chrono::duration<double>(now - last_time).count();
      const float cpi_rate =
          static_cast<float>(current_cpi - last_cpi) / static_cast<float>(elapsed);
      last_time = now;
      last_cpi = current_cpi;

      // Drain queue, keep the latest frame
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
