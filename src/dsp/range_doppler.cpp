#include "range_doppler.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include "waveform/waveform_generator.hpp"

RangeDopplerProcessor::RangeDopplerProcessor(uint32_t num_pulses, uint32_t range_bins,
                                             float prt_s)
    : num_pulses_(num_pulses), range_bins_(range_bins), prt_s_(prt_s) {
  // Hamming window over slow-time (pulse dimension)
  hamming_win_ = WaveformGenerator::hamming_window(num_pulses);

  // FFTW buffers for Doppler FFT (length = num_pulses)
  fft_in_ = fftwf_alloc_complex(num_pulses_);
  fft_out_ = fftwf_alloc_complex(num_pulses_);
  if (!fft_in_ || !fft_out_) throw std::bad_alloc();

  // Single 1D FFT plan — reused for each range bin
  doppler_plan_ = fftwf_plan_dft_1d(static_cast<int>(num_pulses_), fft_in_, fft_out_,
                                    FFTW_FORWARD, FFTW_ESTIMATE);
}

RangeDopplerProcessor::~RangeDopplerProcessor() {
  if (doppler_plan_) fftwf_destroy_plan(doppler_plan_);
  fftwf_free(fft_in_);
  fftwf_free(fft_out_);
}

std::vector<std::complex<float>> RangeDopplerProcessor::process(
    const std::vector<std::complex<float>>& compressed_cpi) const {
  // compressed_cpi: [Np × Nr] row-major (row = pulse)
  // Output:         [Np × Nr] row-major (row = Doppler bin)

  std::vector<std::complex<float>> rd_map(
      static_cast<std::size_t>(num_pulses_) * range_bins_);

  for (uint32_t r = 0; r < range_bins_; ++r) {
    // Load one column (range bin r) across all pulses into FFT buffer,
    // applying Hamming window along slow-time.
    for (uint32_t p = 0; p < num_pulses_; ++p) {
      const auto& s = compressed_cpi[static_cast<std::size_t>(p) * range_bins_ + r];
      const float w = hamming_win_[p];
      fft_in_[p][0] = s.real() * w;
      fft_in_[p][1] = s.imag() * w;
    }

    // Doppler FFT: slow-time → frequency
    fftwf_execute(doppler_plan_);

    // Normalise and store into output row-major [Doppler × Range]
    const float norm = 1.0f / static_cast<float>(num_pulses_);
    for (uint32_t d = 0; d < num_pulses_; ++d) {
      rd_map[static_cast<std::size_t>(d) * range_bins_ + r] = {
          fft_out_[d][0] * norm, fft_out_[d][1] * norm};
    }
  }

  return rd_map;
}

std::vector<float> RangeDopplerProcessor::to_db(
    const std::vector<std::complex<float>>& rd_map) {
  constexpr float kEps = 1e-12f;
  std::vector<float> db(rd_map.size());
  for (std::size_t i = 0; i < rd_map.size(); ++i) {
    const float re = rd_map[i].real();
    const float im = rd_map[i].imag();
    // 20*log10(|z|) = 10*log10(|z|²)
    db[i] = 10.0f * std::log10(re * re + im * im + kEps);
  }
  return db;
}

std::vector<float> RangeDopplerProcessor::fftshift_doppler(const std::vector<float>& data,
                                                            uint32_t num_rows,
                                                            uint32_t num_cols) {
  // Shift by num_rows/2 along the row (Doppler) axis
  std::vector<float> shifted(data.size());
  const uint32_t half = num_rows / 2;
  for (uint32_t d = 0; d < num_rows; ++d) {
    const uint32_t src_row = (d + half) % num_rows;
    for (uint32_t r = 0; r < num_cols; ++r) {
      shifted[static_cast<std::size_t>(d) * num_cols + r] =
          data[static_cast<std::size_t>(src_row) * num_cols + r];
    }
  }
  return shifted;
}

float RangeDopplerProcessor::doppler_bin_to_velocity(int bin) const noexcept {
  // PRF = 1 / PRT,  Doppler frequency = bin * PRF / Np
  // Velocity: v = f_d * lambda / 2  — but lambda requires carrier frequency.
  // In our baseband model we return normalised m/s assuming lambda is applied
  // externally.  Here we return: v = bin * (1/PRT) / Np  [cycles/s].
  const float prf = 1.0f / prt_s_;
  const float f_d = static_cast<float>(bin) * prf / static_cast<float>(num_pulses_);
  return f_d;  // caller multiplies by lambda/2
}
