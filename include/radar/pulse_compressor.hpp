#pragma once

/**
 * @file pulse_compressor.hpp
 * @brief Matched-filter pulse compression using FFTW3 (single precision).
 *
 * Algorithm (frequency-domain correlation):
 *   1. Window received pulse: r_w[n] = r[n] * w[n]
 *   2. FFT of windowed received pulse:     R[k] = FFT{r_w}
 *   3. Precomputed conjugate FFT of ref:   H[k] = conj(FFT{s_ref})
 *   4. Pointwise multiply:                 Y[k] = R[k] * H[k]
 *   5. IFFT:                               y[n] = IFFT{Y}  → compressed pulse
 *
 * Peak Sidelobe Level (PSL) and Integrated Sidelobe Level (ISL) are
 * computed after compression and should satisfy:
 *   PSL < -13 dB  (Hann window theoretical: -31.5 dB)
 */

#include <complex>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <fftw3.h>

/// @brief Quality metrics after pulse compression.
struct CompressionMetrics {
  float psl_db{0.0f};  ///< Peak sidelobe level [dB relative to main lobe]
  float isl_db{0.0f};  ///< Integrated sidelobe level [dB]
};

/**
 * @brief FFTW3-based matched filter for LFM pulse compression.
 *
 * FFTW plans are created once in the constructor; subsequent calls to
 * compress() reuse the plans for maximum throughput.
 */
class PulseCompressor {
 public:
  /**
   * @param fft_size     Length of the FFT (should be power of two ≥ pulse samples)
   * @param chirp_ref    Complex chirp reference vector (length ≤ fft_size)
   * @param window       Hann window (same length as chirp_ref)
   */
  PulseCompressor(uint32_t fft_size, std::vector<std::complex<float>> chirp_ref,
                  std::vector<float> window);
  ~PulseCompressor();

  // Non-copyable (FFTW plans are not copyable)
  PulseCompressor(const PulseCompressor&) = delete;
  PulseCompressor& operator=(const PulseCompressor&) = delete;

  /**
   * @brief Compress one received pulse.
   * @param rx  Received pulse samples (length = fft_size, zero-padded if shorter)
   * @return    Compressed pulse of length fft_size.
   */
  [[nodiscard]] std::vector<std::complex<float>> compress(
      const std::vector<std::complex<float>>& rx) const;

  /**
   * @brief Compute PSL and ISL from a compressed pulse.
   * @param compressed  Output of compress().
   */
  [[nodiscard]] static CompressionMetrics compute_metrics(
      const std::vector<std::complex<float>>& compressed);

  [[nodiscard]] uint32_t fft_size() const noexcept { return fft_size_; }

 private:
  uint32_t fft_size_;

  // FFTW buffers and plans
  fftwf_complex* ref_freq_{nullptr};   ///< Precomputed conjugate FFT of reference
  fftwf_complex* in_buf_{nullptr};     ///< FFT input buffer
  fftwf_complex* freq_buf_{nullptr};   ///< Frequency-domain intermediate
  fftwf_complex* out_buf_{nullptr};    ///< IFFT output buffer
  fftwf_plan fwd_plan_{nullptr};       ///< Forward FFT plan (in_buf → freq_buf)
  fftwf_plan inv_plan_{nullptr};       ///< Inverse FFT plan (freq_buf → out_buf)

  std::vector<float> window_;          ///< Hann window
};
