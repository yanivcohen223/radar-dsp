#include "radar/pulse_compressor.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <numeric>
#include <stdexcept>

PulseCompressor::PulseCompressor(uint32_t fft_size,
                                 std::vector<std::complex<float>> chirp_ref,
                                 std::vector<float> window)
    : fft_size_(fft_size), window_(std::move(window)) {
  assert(fft_size_ > 0);
  assert(chirp_ref.size() <= fft_size_);

  // Allocate FFTW-aligned buffers
  ref_freq_ = fftwf_alloc_complex(fft_size_);
  in_buf_ = fftwf_alloc_complex(fft_size_);
  freq_buf_ = fftwf_alloc_complex(fft_size_);
  out_buf_ = fftwf_alloc_complex(fft_size_);

  if (!ref_freq_ || !in_buf_ || !freq_buf_ || !out_buf_) {
    throw std::bad_alloc();
  }

  // Create plans using ESTIMATE (no measurement — the data in buffers is
  // intentionally overwritten here, plans must be created before use)
  fwd_plan_ = fftwf_plan_dft_1d(static_cast<int>(fft_size_), in_buf_, freq_buf_,
                                 FFTW_FORWARD, FFTW_ESTIMATE);
  inv_plan_ = fftwf_plan_dft_1d(static_cast<int>(fft_size_), freq_buf_, out_buf_,
                                 FFTW_BACKWARD, FFTW_ESTIMATE);

  // Pre-compute conjugate FFT of reference chirp: H[k] = conj(FFT{s_ref})
  std::memset(in_buf_, 0, sizeof(fftwf_complex) * fft_size_);
  for (std::size_t n = 0; n < chirp_ref.size(); ++n) {
    in_buf_[n][0] = chirp_ref[n].real();
    in_buf_[n][1] = chirp_ref[n].imag();
  }
  // Temporary plan for the reference FFT
  fftwf_plan ref_plan = fftwf_plan_dft_1d(static_cast<int>(fft_size_), in_buf_, ref_freq_,
                                           FFTW_FORWARD, FFTW_ESTIMATE);
  fftwf_execute(ref_plan);
  fftwf_destroy_plan(ref_plan);

  // Conjugate in-place and normalise by 1/N (absorbed into IFFT normalisation)
  for (uint32_t k = 0; k < fft_size_; ++k) {
    ref_freq_[k][1] = -ref_freq_[k][1];  // conj: negate imaginary part
  }
}

PulseCompressor::~PulseCompressor() {
  if (fwd_plan_) fftwf_destroy_plan(fwd_plan_);
  if (inv_plan_) fftwf_destroy_plan(inv_plan_);
  fftwf_free(ref_freq_);
  fftwf_free(in_buf_);
  fftwf_free(freq_buf_);
  fftwf_free(out_buf_);
}

std::vector<std::complex<float>> PulseCompressor::compress(
    const std::vector<std::complex<float>>& rx) const {
  // Step 1: apply Hann window and copy into FFT input buffer
  std::memset(in_buf_, 0, sizeof(fftwf_complex) * fft_size_);
  const std::size_t copy_len = std::min(rx.size(), static_cast<std::size_t>(fft_size_));
  for (std::size_t n = 0; n < copy_len; ++n) {
    const float w = (n < window_.size()) ? window_[n] : 1.0f;
    in_buf_[n][0] = rx[n].real() * w;
    in_buf_[n][1] = rx[n].imag() * w;
  }

  // Step 2: FFT of windowed received pulse: R[k] = FFT{r_w}
  fftwf_execute(fwd_plan_);

  // Step 3: Pointwise multiply with conjugate reference: Y[k] = R[k] * H[k]
  for (uint32_t k = 0; k < fft_size_; ++k) {
    // (a + jb)(c + jd) = (ac - bd) + j(ad + bc)
    // H[k] already conjugated in constructor, so H = (c - jd)
    const float a = freq_buf_[k][0];
    const float b = freq_buf_[k][1];
    const float c = ref_freq_[k][0];
    const float d = ref_freq_[k][1];  // d is already negated (conjugate stored)
    freq_buf_[k][0] = a * c - b * d;
    freq_buf_[k][1] = a * d + b * c;
  }

  // Step 4: IFFT → compressed pulse y[n] = IFFT{Y}
  fftwf_execute(inv_plan_);

  // Copy output and normalise by 1/N (FFTW does not normalise)
  const float norm = 1.0f / static_cast<float>(fft_size_);
  std::vector<std::complex<float>> out(fft_size_);
  for (uint32_t n = 0; n < fft_size_; ++n) {
    out[n] = {out_buf_[n][0] * norm, out_buf_[n][1] * norm};
  }
  return out;
}

CompressionMetrics PulseCompressor::compute_metrics(
    const std::vector<std::complex<float>>& compressed) {
  // Compute power profile |y[n]|²
  std::vector<float> power(compressed.size());
  for (std::size_t n = 0; n < compressed.size(); ++n) {
    const float re = compressed[n].real();
    const float im = compressed[n].imag();
    power[n] = re * re + im * im;
  }

  // Find peak
  const auto peak_it = std::max_element(power.begin(), power.end());
  const float peak_power = *peak_it;
  const std::size_t peak_idx = static_cast<std::size_t>(peak_it - power.begin());

  // Guard region around main lobe: ±3 samples
  constexpr std::size_t kGuard = 3;

  // PSL: highest sidelobe power outside guard
  float max_sidelobe = 0.0f;
  float total_sidelobe = 0.0f;
  for (std::size_t n = 0; n < power.size(); ++n) {
    const std::size_t dist = (n > peak_idx) ? (n - peak_idx) : (peak_idx - n);
    // Wrap-around distance
    const std::size_t wrap_dist = std::min(dist, power.size() - dist);
    if (wrap_dist > kGuard) {
      max_sidelobe = std::max(max_sidelobe, power[n]);
      total_sidelobe += power[n];
    }
  }

  CompressionMetrics m{};
  if (peak_power > 0.0f) {
    // PSL = 10 * log10(max_sidelobe / peak)
    m.psl_db = (max_sidelobe > 0.0f)
                   ? 10.0f * std::log10(max_sidelobe / peak_power)
                   : -100.0f;
    // ISL = 10 * log10(sum_sidelobes / peak)
    m.isl_db = (total_sidelobe > 0.0f)
                   ? 10.0f * std::log10(total_sidelobe / peak_power)
                   : -100.0f;
  }
  return m;
}
