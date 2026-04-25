#include "radar/radar_pipeline.hpp"

#include <cmath>
#include <stdexcept>

static constexpr float kC = 3.0e8f;

RadarPipeline::RadarPipeline(PipelineConfig cfg, std::vector<TargetConfig> targets)
    : cfg_(cfg) {
  const auto& cpi = cfg_.cpi;

  // ── Waveform ──────────────────────────────────────────────────────────────
  ChirpParams chirp_params{cpi.chirp_bw_hz, cpi.pulse_width_s, cpi.sample_rate_hz};
  waveform_ = std::make_unique<WaveformGenerator>(chirp_params);

  // ── ADC Simulator ─────────────────────────────────────────────────────────
  adc_ = std::make_unique<AdcSimulator>(cpi, std::move(targets));

  // ── Pulse Compressor ──────────────────────────────────────────────────────
  const uint32_t fft_size = cpi.range_bins;
  auto ref = waveform_->chirp_reference();
  auto win = WaveformGenerator::hann_window(static_cast<uint32_t>(ref.size()));
  // Zero-pad window to fft_size for PulseCompressor
  win.resize(fft_size, 0.0f);
  compressor_ = std::make_unique<PulseCompressor>(fft_size, std::move(ref), std::move(win));

  // ── Range-Doppler Processor ───────────────────────────────────────────────
  rd_proc_ = std::make_unique<RangeDopplerProcessor>(cpi.num_pulses, cpi.range_bins, cpi.prt_s);

  // ── CFAR ──────────────────────────────────────────────────────────────────
  // Range resolution: c / (2 * B)
  const float range_res_m = kC / (2.0f * cpi.chirp_bw_hz);
  // Velocity resolution: lambda * PRF / (2 * Np)
  // lambda = c / fc_eff; fc_eff = bw/2 for our baseband model
  const float fc_eff = cpi.chirp_bw_hz / 2.0f;
  const float lambda = kC / fc_eff;
  const float prf = 1.0f / cpi.prt_s;
  const float vel_res = lambda * prf / (2.0f * static_cast<float>(cpi.num_pulses));

  cfar_ = std::make_unique<CfarDetector<CfarType::CA>>(
      /*guard=*/2, /*ref=*/8, /*pfa=*/1e-6f, range_res_m, vel_res);

  // ── Track Manager ─────────────────────────────────────────────────────────
  tracker_ = std::make_unique<TrackManager>();

  // ── Performance Counter ───────────────────────────────────────────────────
  perf_ = std::make_unique<PerfCounter>(cfg_.output_dir);

  // ── RT Thread ─────────────────────────────────────────────────────────────
  // Period = one CPI duration = num_pulses * prt_s
  const int64_t period_ns =
      static_cast<int64_t>(static_cast<float>(cpi.num_pulses) * cpi.prt_s * 1e9f);

  rt_thread_ = std::make_unique<RtThread>(
      "radar-pipeline",
      cfg_.enable_rt ? cfg_.rt_priority : 0,
      cfg_.cpu_affinity,
      period_ns);
}

RadarPipeline::~RadarPipeline() { stop(); }

void RadarPipeline::start() {
  rt_thread_->start([this]() { process_one_cpi(); });
}

void RadarPipeline::stop() {
  if (rt_thread_) rt_thread_->stop();
}

std::optional<PipelineOutput> RadarPipeline::pop_output() {
  return output_queue_.try_pop();
}

JitterStats RadarPipeline::jitter_stats() const {
  return rt_thread_ ? rt_thread_->jitter_stats() : JitterStats{};
}

void RadarPipeline::process_one_cpi() {
  const auto& cpi = cfg_.cpi;

  // ── Stage 1: ADC ─────────────────────────────────────────────────────────
  perf_->begin("1_adc");
  auto raw = adc_->generate_cpi();
  perf_->end("1_adc");

  // ── Stage 2: Pulse Compression ────────────────────────────────────────────
  perf_->begin("2_pulse_compress");
  std::vector<std::complex<float>> compressed(
      static_cast<std::size_t>(cpi.num_pulses) * cpi.range_bins);

  CompressionMetrics compression_metrics{};  // taken from the first pulse
  for (uint32_t p = 0; p < cpi.num_pulses; ++p) {
    // Extract one pulse row
    std::vector<std::complex<float>> pulse(
        raw.begin() + static_cast<std::ptrdiff_t>(p * cpi.range_bins),
        raw.begin() + static_cast<std::ptrdiff_t>((p + 1) * cpi.range_bins));

    auto c = compressor_->compress(pulse);
    if (p == 0) {
      // Compute PSL/ISL once per CPI using the first compressed pulse
      compression_metrics = PulseCompressor::compute_metrics(c);
    }
    std::copy(c.begin(), c.end(),
              compressed.begin() + static_cast<std::ptrdiff_t>(p * cpi.range_bins));
  }
  perf_->end("2_pulse_compress");

  // ── Stage 3: Range-Doppler Map ────────────────────────────────────────────
  perf_->begin("3_range_doppler");
  auto rd_complex = rd_proc_->process(compressed);
  auto rd_db_raw = RangeDopplerProcessor::to_db(rd_complex);
  auto rd_db = RangeDopplerProcessor::fftshift_doppler(rd_db_raw, cpi.num_pulses, cpi.range_bins);
  perf_->end("3_range_doppler");

  // ── Stage 4: CFAR ─────────────────────────────────────────────────────────
  perf_->begin("4_cfar");
  auto detections = cfar_->detect(rd_db, cpi.num_pulses, cpi.range_bins);
  perf_->end("4_cfar");

  // ── Stage 5: Tracking ─────────────────────────────────────────────────────
  perf_->begin("5_tracker");
  tracker_->update(detections);
  auto tracks = tracker_->get_tracks();
  perf_->end("5_tracker");

  // ── Publish output ────────────────────────────────────────────────────────
  PipelineOutput out{};
  out.rd_map_db = std::move(rd_db);
  out.detections = std::move(detections);
  out.tracks = std::move(tracks);
  out.num_doppler = cpi.num_pulses;
  out.num_range = cpi.range_bins;
  out.cpi_index = cpi_count_.load();
  out.psl_db = compression_metrics.psl_db;
  out.isl_db = compression_metrics.isl_db;

  // Non-blocking push — drop if consumer is behind
  output_queue_.try_push(std::move(out));
  cpi_count_.fetch_add(1, std::memory_order_relaxed);
}
