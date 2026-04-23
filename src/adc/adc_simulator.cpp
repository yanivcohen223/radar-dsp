#include "adc_simulator.hpp"

#include <cmath>

static constexpr float kC = 3.0e8f;  // Speed of light [m/s]
static constexpr float kPi = static_cast<float>(M_PI);

AdcSimulator::AdcSimulator(CpiConfig cfg, std::vector<TargetConfig> targets, uint32_t seed)
    : cfg_(cfg), targets_(std::move(targets)), rng_(seed) {}

float AdcSimulator::target_amplitude(const TargetConfig& t, float /*range_m*/) const noexcept {
  // RCS amplitude: linear scale from dB value
  // A = 10^(rcs_db / 20) — voltage amplitude proportional to sqrt(RCS)
  return std::pow(10.0f, t.rcs_db / 20.0f);
}

std::vector<std::complex<float>> AdcSimulator::generate_cpi() {
  const uint32_t Np = cfg_.num_pulses;
  const uint32_t Nr = cfg_.range_bins;
  const float fs = cfg_.sample_rate_hz;
  const float tau = cfg_.pulse_width_s;
  const float prt = cfg_.prt_s;
  const float bw = cfg_.chirp_bw_hz;

  // LFM chirp rate [Hz/s]
  // k = B / tau
  const float k = bw / tau;

  // Noise standard deviation from SNR
  // SNR_lin = A^2 / (2 * sigma^2)  →  sigma = A / sqrt(2 * SNR_lin)
  // We normalise target amplitude to 1 and scale noise accordingly.
  const float snr_lin = std::pow(10.0f, cfg_.snr_db / 10.0f);
  const float noise_sigma = 1.0f / std::sqrt(2.0f * snr_lin);

  std::vector<std::complex<float>> cpi(static_cast<std::size_t>(Np) * Nr,
                                       std::complex<float>{0.0f, 0.0f});

  const float ts = 1.0f / fs;  // sample period

  for (uint32_t p = 0; p < Np; ++p) {
    // Pulse time offset from start of CPI
    // t_pulse would be used for absolute range-migration tracking; unused in
    // this baseband model where migration is handled per-pulse below.
    [[maybe_unused]] const float t_pulse =
        (static_cast<float>(cpi_index_) * static_cast<float>(Np) + static_cast<float>(p)) * prt;

    for (const auto& tgt : targets_) {
      // Range migration: target moves v * t_pulse since start of CPI
      // Δr = v * p * prt  (range walk per pulse within the CPI)
      const float range_p = tgt.range_m - tgt.velocity_ms * static_cast<float>(p) * prt;
      if (range_p < 0.0f) continue;

      // Two-way time delay [s]
      const float delay = 2.0f * range_p / kC;

      // Range bin (fractional)
      const float bin_f = delay * fs;
      const int bin_start = static_cast<int>(bin_f);
      if (bin_start < 0 || bin_start >= static_cast<int>(Nr)) continue;

      // Amplitude
      const float A = target_amplitude(tgt, range_p);

      // Doppler phase: φ = 4π * f_c_eff * range / c
      // For baseband model we use the Doppler phase shift per pulse:
      // φ_d = 4π * v / lambda.  We approximate lambda = c / (fc + bw/2).
      // In a pure LFM baseband model the Doppler shows up as a slow-time
      // phase ramp: φ[p] = -4π * v * p * prt / (c / bw_center_equiv)
      // For a 10 MHz chirp starting at DC we use carrier = bw / 2 as eff freq.
      const float fc_eff = bw / 2.0f;
      const float lambda = kC / fc_eff;
      const float doppler_phase = -4.0f * kPi * tgt.velocity_ms * static_cast<float>(p) * prt /
                                  lambda;  // Doppler phase ramp across pulses

      // Inject LFM return: for each range sample the received signal is the
      // time-delayed, phase-shifted chirp.
      // s(t) = A * exp(j * 2π * (k/2 * (t - delay)^2))  for t ∈ [delay, delay+tau]
      for (uint32_t n = bin_start; n < Nr; ++n) {
        const float t = static_cast<float>(n) * ts;  // fast-time of sample n
        const float dt = t - delay;                   // time relative to pulse start
        if (dt < 0.0f || dt > tau) break;

        // LFM phase: φ = 2π * (k/2) * dt^2
        const float lfm_phase = kPi * k * dt * dt;  // 2π*(k/2)*dt² = π*k*dt²
        const float total_phase = lfm_phase + doppler_phase;

        // Complex exponential: exp(j*φ) = cos(φ) + j*sin(φ)
        const std::complex<float> signal = A * std::complex<float>{std::cos(total_phase),
                                                                    std::sin(total_phase)};

        cpi[static_cast<std::size_t>(p) * Nr + n] += signal;
      }
    }

    // Add AWGN (complex: real + imaginary each ~ N(0, σ))
    for (uint32_t n = 0; n < Nr; ++n) {
      const float ni = noise_sigma * noise_dist_(rng_);
      const float nq = noise_sigma * noise_dist_(rng_);
      cpi[static_cast<std::size_t>(p) * Nr + n] += std::complex<float>{ni, nq};
    }
  }

  ++cpi_index_;
  return cpi;
}
