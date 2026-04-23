#pragma once

/**
 * @file range_doppler.hpp
 * @brief 2D Range-Doppler map processor.
 *
 * Processing steps:
 *   1. Receive Np × Nr matrix of pulse-compressed samples.
 *   2. Apply Hamming window along the slow-time (pulse) axis.
 *   3. Compute FFT along slow-time for each range bin → Doppler bins.
 *   4. Output |RD[f_d, r]| in dB for display and CFAR.
 *
 * The Doppler FFT maps pulse index p → Doppler bin f_d:
 *   v = f_d * λ / 2  where  f_d = bin * PRF / Np
 */

#include <complex>
#include <cstdint>
#include <vector>

#include <fftw3.h>

/**
 * @brief Produces the 2D Range-Doppler map from a CPI of compressed pulses.
 *
 * Output is stored row-major as [Doppler_bins × range_bins].
 * Doppler bin 0 = zero Doppler (DC), bins are ordered DC → Nyquist → −Nyquist + 1.
 * Use fftshift to centre zero-Doppler before display.
 */
class RangeDopplerProcessor {
 public:
  /**
   * @param num_pulses   Number of slow-time samples (Doppler FFT length)
   * @param range_bins   Number of range samples
   * @param prt_s        Pulse repetition time [s]
   */
  RangeDopplerProcessor(uint32_t num_pulses, uint32_t range_bins, float prt_s);
  ~RangeDopplerProcessor();

  RangeDopplerProcessor(const RangeDopplerProcessor&) = delete;
  RangeDopplerProcessor& operator=(const RangeDopplerProcessor&) = delete;

  /**
   * @brief Compute the Range-Doppler map.
   *
   * @param compressed_cpi Flat [Np × Nr] compressed pulse matrix (row = pulse)
   * @return Flat [Np × Nr] complex RD map (row = Doppler bin)
   */
  [[nodiscard]] std::vector<std::complex<float>> process(
      const std::vector<std::complex<float>>& compressed_cpi) const;

  /**
   * @brief Convert complex RD map to log-magnitude in dB.
   *
   * mag_db[i] = 20 * log10(|z[i]| + eps)
   */
  [[nodiscard]] static std::vector<float> to_db(
      const std::vector<std::complex<float>>& rd_map);

  /**
   * @brief Apply FFT-shift along the Doppler axis so DC is centred.
   *
   * @param data     Flat [Np × Nr] matrix (row = Doppler bin)
   * @param num_rows Number of Doppler bins (Np)
   * @param num_cols Number of range bins (Nr)
   */
  [[nodiscard]] static std::vector<float> fftshift_doppler(
      const std::vector<float>& data, uint32_t num_rows, uint32_t num_cols);

  /// @brief Convert Doppler bin index to velocity [m/s].
  [[nodiscard]] float doppler_bin_to_velocity(int bin) const noexcept;

  [[nodiscard]] uint32_t num_pulses() const noexcept { return num_pulses_; }
  [[nodiscard]] uint32_t range_bins() const noexcept { return range_bins_; }

 private:
  uint32_t num_pulses_;
  uint32_t range_bins_;
  float prt_s_;
  std::vector<float> hamming_win_;  ///< Hamming window over slow-time

  fftwf_complex* fft_in_{nullptr};
  fftwf_complex* fft_out_{nullptr};
  fftwf_plan doppler_plan_{nullptr};
};
