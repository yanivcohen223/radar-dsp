#include "radar/rt_thread.hpp"

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>

// ─── helpers ─────────────────────────────────────────────────────────────────

static inline int64_t timespec_to_ns(const struct timespec& ts) {
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

static inline struct timespec ns_to_timespec(int64_t ns) {
  struct timespec ts{};
  ts.tv_sec = static_cast<time_t>(ns / 1'000'000'000LL);
  ts.tv_nsec = static_cast<long>(ns % 1'000'000'000LL);
  return ts;
}

// ─── RtThread implementation ─────────────────────────────────────────────────

RtThread::RtThread(std::string name, int priority, int cpu_affinity, int64_t period_ns)
    : name_(std::move(name)),
      priority_(priority),
      cpu_affinity_(cpu_affinity),
      period_ns_(period_ns) {
  assert(priority >= 0 && priority <= 99);  // 0 = SCHED_OTHER, 1-99 = SCHED_FIFO
  jitter_samples_.reserve(100'000);
}

RtThread::~RtThread() { stop(); }

void RtThread::start(std::function<void()> work) {
  work_ = std::move(work);
  running_.store(true);

  pthread_attr_t attr;
  pthread_attr_init(&attr);

  if (priority_ >= 1) {
    // Request SCHED_FIFO — requires CAP_SYS_NICE (Docker --privileged).
    // Only attempted when a valid RT priority (1–99) is provided.
    struct sched_param sp{};
    sp.sched_priority = priority_;
    int rc = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (rc == 0) {
      rc = pthread_attr_setschedparam(&attr, &sp);
      if (rc == 0) {
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
      }
    }
    // If any RT attribute call fails, the thread inherits the caller's
    // scheduling policy (SCHED_OTHER) — graceful degradation.
  }
  // priority_ == 0 → use SCHED_OTHER (default), no extra attributes needed

  int rc = pthread_create(&thread_, &attr, &RtThread::thread_entry, this);
  pthread_attr_destroy(&attr);
  if (rc != 0) {
    running_.store(false);
    throw std::runtime_error("pthread_create failed: " + std::string(strerror(rc)));
  }

  pthread_setname_np(thread_, name_.substr(0, 15).c_str());
}

void RtThread::stop() {
  if (!running_.load()) return;
  running_.store(false);
  if (thread_) {
    pthread_join(thread_, nullptr);
    thread_ = 0;
  }
}

void* RtThread::thread_entry(void* arg) {
  auto* self = static_cast<RtThread*>(arg);
  self->run();
  return nullptr;
}

void RtThread::run() {
  // Pin to CPU if requested
  if (cpu_affinity_ >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<int>(cpu_affinity_), &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
  }

  struct timespec next{};
  clock_gettime(CLOCK_MONOTONIC, &next);

  while (running_.load(std::memory_order_relaxed)) {
    // Advance wake-up time by one period
    int64_t next_ns = timespec_to_ns(next) + period_ns_;
    next = ns_to_timespec(next_ns);

    // Sleep until next period boundary
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    // Measure actual wake time for jitter
    struct timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t jitter = timespec_to_ns(now) - next_ns;  // positive = late

    jitter_samples_.push_back(jitter);

    if (running_.load(std::memory_order_relaxed)) {
      work_();
    }
  }
}

JitterStats RtThread::jitter_stats() const {
  if (jitter_samples_.empty()) return {};

  std::vector<int64_t> sorted(jitter_samples_);
  std::sort(sorted.begin(), sorted.end());

  JitterStats s{};
  s.sample_count = sorted.size();
  s.min_ns = sorted.front();
  s.max_ns = sorted.back();

  // Mean
  double sum = 0.0;
  for (auto v : sorted) sum += static_cast<double>(v);
  s.mean_ns = sum / static_cast<double>(sorted.size());

  // p99
  std::size_t idx = static_cast<std::size_t>(0.99 * static_cast<double>(sorted.size()));
  if (idx >= sorted.size()) idx = sorted.size() - 1;
  s.p99_ns = sorted[idx];

  return s;
}
