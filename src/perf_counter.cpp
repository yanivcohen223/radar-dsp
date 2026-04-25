#include "radar/perf_counter.hpp"

#include <time.h>

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <numeric>
#include <stdexcept>

static int64_t now_ns() {
  struct timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

PerfCounter::PerfCounter(std::string output_dir) : output_dir_(std::move(output_dir)) {}

PerfCounter::~PerfCounter() {
#ifdef ENABLE_PROFILING
  try {
    export_csv();
  } catch (...) {
    // Never throw from destructor
  }
#endif
}

void PerfCounter::begin(const std::string& stage) { stages_[stage].start_ns = now_ns(); }

void PerfCounter::end(const std::string& stage) {
  int64_t elapsed = now_ns() - stages_[stage].start_ns;
  stages_[stage].samples.push(elapsed);
}

std::vector<StageStats> PerfCounter::compute_stats() const {
  std::vector<StageStats> result;
  result.reserve(stages_.size());

  for (const auto& [name, sd] : stages_) {
    if (sd.samples.count == 0) continue;

    std::vector<double> us;
    us.reserve(sd.samples.count);
    for (std::size_t i = 0; i < sd.samples.count; ++i) {
      us.push_back(static_cast<double>(sd.samples.data[i]) / 1000.0);
    }
    std::sort(us.begin(), us.end());

    StageStats s{};
    s.name = name;
    s.count = sd.samples.count;
    s.mean_us = std::accumulate(us.begin(), us.end(), 0.0) / static_cast<double>(us.size());
    s.max_us = us.back();

    auto pct = [&](double p) {
      std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(us.size()));
      if (idx >= us.size()) idx = us.size() - 1;
      return us[idx];
    };
    s.p50_us = pct(0.50);
    s.p95_us = pct(0.95);
    s.p99_us = pct(0.99);

    result.push_back(std::move(s));
  }

  std::sort(result.begin(), result.end(),
            [](const StageStats& a, const StageStats& b) { return a.name < b.name; });
  return result;
}

void PerfCounter::export_csv() const {
  std::filesystem::create_directories(output_dir_);

  // Timestamp filename
  std::time_t t = std::time(nullptr);
  char ts_buf[32];
  std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", std::localtime(&t));

  std::string path = output_dir_ + "/perf_" + ts_buf + ".csv";
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) return;

  std::fprintf(f, "stage_name,mean_us,p50_us,p95_us,p99_us,max_us,count\n");
  for (const auto& s : compute_stats()) {
    std::fprintf(f, "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%lu\n", s.name.c_str(), s.mean_us, s.p50_us,
                 s.p95_us, s.p99_us, s.max_us, static_cast<unsigned long>(s.count));
  }
  std::fclose(f);
}
