#pragma once

/**
 * @file perf_counter.hpp
 * @brief Per-stage nanosecond-resolution performance counter.
 *
 * Uses clock_gettime(CLOCK_MONOTONIC) for measurements and maintains a
 * circular buffer of the last kBufSize samples per stage.  On destruction
 * (or explicit flush), statistics are exported to a timestamped CSV.
 */

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/// @brief Aggregated statistics for one pipeline stage.
struct StageStats {
  std::string name;
  double mean_us{0.0};
  double p50_us{0.0};
  double p95_us{0.0};
  double p99_us{0.0};
  double max_us{0.0};
  uint64_t count{0};
};

/**
 * @brief Thread-safe performance counter with circular sample buffer.
 *
 * Call begin(stage) / end(stage) around each pipeline stage.
 * Call export_csv(path) to write results.
 */
class PerfCounter {
 public:
  static constexpr std::size_t kBufSize = 1000;

  explicit PerfCounter(std::string output_dir = "/radar-dsp/output");
  ~PerfCounter();

  /// @brief Record start time for a named stage.
  void begin(const std::string& stage);

  /// @brief Record end time for a named stage (computes elapsed, stores in ring).
  void end(const std::string& stage);

  /// @brief Compute statistics for all stages and return them.
  [[nodiscard]] std::vector<StageStats> compute_stats() const;

  /// @brief Write CSV to output_dir/perf_<timestamp>.csv
  void export_csv() const;

 private:
  struct RingBuffer {
    std::array<int64_t, kBufSize> data{};
    std::size_t write_pos{0};
    std::size_t count{0};

    void push(int64_t ns) noexcept {
      data[write_pos] = ns;
      write_pos = (write_pos + 1) % kBufSize;
      if (count < kBufSize) ++count;
    }
  };

  struct StageData {
    int64_t start_ns{0};
    RingBuffer samples;
  };

  std::string output_dir_;
  mutable std::unordered_map<std::string, StageData> stages_;
};
