#pragma once

/**
 * @file radar_pipeline.hpp
 * @brief End-to-end radar DSP pipeline coordinator.
 *
 * Wires all processing stages together:
 *   ADC Simulator → Pulse Compressor → Range-Doppler → CFAR → Track Manager
 *
 * Each CPI is processed on a dedicated RT thread.  Results are published via
 * lock-free queues to the dashboard thread.
 *
 * The pipeline runs until stop() is called.
 */

#include <atomic>
#include <functional>
#include <memory>
#include <vector>

#include "adc/adc_simulator.hpp"
#include "dsp/cfar_detector.hpp"
#include "dsp/pulse_compressor.hpp"
#include "dsp/range_doppler.hpp"
#include "lock_free_queue.hpp"
#include "metrics/perf_counter.hpp"
#include "scheduler/rt_thread.hpp"
#include "tracker/track_manager.hpp"
#include "waveform/waveform_generator.hpp"

/// @brief Published output of one processed CPI.
struct PipelineOutput {
  std::vector<float> rd_map_db;      ///< dB magnitude [Nd × Nr]
  std::vector<Detection> detections;
  std::vector<Track> tracks;
  uint32_t num_doppler{0};
  uint32_t num_range{0};
  uint64_t cpi_index{0};
};

/// @brief Configuration knobs for the full pipeline.
struct PipelineConfig {
  CpiConfig cpi{};
  int rt_priority{50};
  int cpu_affinity{-1};
  bool enable_rt{true};
  bool benchmark_mode{false};
  std::string output_dir{"/radar-dsp/output"};
};

/**
 * @brief Orchestrates the full radar processing pipeline.
 */
class RadarPipeline {
 public:
  explicit RadarPipeline(PipelineConfig cfg, std::vector<TargetConfig> targets);
  ~RadarPipeline();

  RadarPipeline(const RadarPipeline&) = delete;
  RadarPipeline& operator=(const RadarPipeline&) = delete;

  /// @brief Start the processing loop.
  void start();

  /// @brief Signal all threads to stop and join.
  void stop();

  /// @brief Non-blocking: pop the latest processed CPI output, if any.
  [[nodiscard]] std::optional<PipelineOutput> pop_output();

  /// @brief Access the performance counter for CSV export.
  [[nodiscard]] const PerfCounter& perf() const { return *perf_; }

  /// @brief CPI index counter.
  [[nodiscard]] uint64_t cpi_count() const noexcept { return cpi_count_.load(); }

 private:
  void process_one_cpi();

  PipelineConfig cfg_;

  // DSP stages
  std::unique_ptr<AdcSimulator> adc_;
  std::unique_ptr<WaveformGenerator> waveform_;
  std::unique_ptr<PulseCompressor> compressor_;
  std::unique_ptr<RangeDopplerProcessor> rd_proc_;
  std::unique_ptr<CfarDetector<CfarType::CA>> cfar_;
  std::unique_ptr<TrackManager> tracker_;
  std::unique_ptr<PerfCounter> perf_;

  // RT thread for pipeline processing
  std::unique_ptr<RtThread> rt_thread_;

  // Lock-free output queue (single producer = pipeline, single consumer = dashboard)
  static constexpr std::size_t kQueueDepth = 8;
  LockFreeQueue<PipelineOutput, kQueueDepth> output_queue_;

  std::atomic<uint64_t> cpi_count_{0};
};
