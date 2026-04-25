#pragma once

/**
 * @file rt_thread.hpp
 * @brief Real-time thread wrapper using POSIX SCHED_FIFO scheduling.
 *
 * Wraps pthread to provide:
 *  - SCHED_FIFO with configurable priority (1–99)
 *  - CPU affinity via pthread_setaffinity_np
 *  - Periodic wake-up with jitter measurement
 *  - Graceful shutdown via std::atomic<bool>
 */

#include <pthread.h>
#include <sched.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

/// @brief Jitter statistics collected over the thread's lifetime.
struct JitterStats {
  int64_t min_ns{INT64_MAX};
  int64_t max_ns{0};
  double mean_ns{0.0};
  int64_t p99_ns{0};
  uint64_t sample_count{0};
};

/**
 * @brief Periodic real-time thread with SCHED_FIFO scheduling.
 *
 * Usage:
 * @code
 *   RtThread t("dsp", 10, 1, period_ns);   // priority=10, cpu=1
 *   t.start([&]{ process_one_cpi(); });
 *   // ... later ...
 *   t.stop();
 *   auto stats = t.jitter_stats();
 * @endcode
 */
class RtThread {
 public:
  /**
   * @param name       Thread name (shown in /proc/PID/task/TID/comm)
   * @param priority   SCHED_FIFO priority 1–99
   * @param cpu_affinity CPU index to pin thread to (-1 = no pinning)
   * @param period_ns  Wake-up period in nanoseconds
   */
  RtThread(std::string name, int priority, int cpu_affinity, int64_t period_ns);
  ~RtThread();

  // Non-copyable
  RtThread(const RtThread&) = delete;
  RtThread& operator=(const RtThread&) = delete;

  /// @brief Launch the thread, calling work() once per period.
  void start(std::function<void()> work);

  /// @brief Signal the thread to stop and join.
  void stop();

  /// @brief Returns true if the thread is currently running.
  [[nodiscard]] bool running() const noexcept { return running_.load(); }

  /// @brief Compute jitter statistics from recorded samples.
  [[nodiscard]] JitterStats jitter_stats() const;

  /// @brief Raw jitter samples (actual − expected wake time), nanoseconds.
  [[nodiscard]] const std::vector<int64_t>& jitter_samples() const { return jitter_samples_; }

 private:
  static void* thread_entry(void* arg);
  void run();

  std::string name_;
  int priority_;
  int cpu_affinity_;
  int64_t period_ns_;

  pthread_t thread_{};
  std::atomic<bool> running_{false};
  std::function<void()> work_;

  std::vector<int64_t> jitter_samples_;  // reserved upfront; written by RT thread only
};
