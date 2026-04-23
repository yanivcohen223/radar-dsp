#pragma once

/**
 * @file waveform_generator.hpp
 * @brief Generates the LFM chirp reference waveform and window functions.
 *
 * The reference chirp is used as the matched filter kernel in pulse
 * compression.  The Hann window suppresses range sidelobes at the cost of
 * ~1.5 dB SNR loss and ~1.3× main-lobe broadening.
 */

#include <complex>
#include <cstdint>
#include <vector>

/**
 * @brief Parameters describing the transmitted LFM chirp.
 */
struct ChirpParams {
  float bandwidth_hz{10e6f};   ///< LFM sweep bandwidth [Hz]
  float pulse_width_s{10e-6f}; ///< Pulse duration [s]
  float sample_rate_hz{10e6f}; ///< ADC sample rate [Hz]
};

/**
 * @brief Waveform generator producing LFM chirp and window vectors.
 */
class WaveformGenerator {
 public:
  explicit WaveformGenerator(ChirpParams params);

  /**
   * @brief Generate the complex LFM chirp reference.
   *
   * The up-chirp is: s(t) = exp(j * π * k * t²),  t ∈ [0, τ]
   * where k = B / τ is the chirp rate.
   *
   * @return Complex reference vector of length floor(pulse_width_s * sample_rate_hz).
   */
  [[nodiscard]] std::vector<std::complex<float>> chirp_reference() const;

  /**
   * @brief Generate a Hann window of length N.
   *
   * w[n] = 0.5 * (1 − cos(2π n / (N−1))),  n = 0 … N−1
   */
  [[nodiscard]] static std::vector<float> hann_window(uint32_t N);

  /**
   * @brief Generate a Hamming window of length N.
   *
   * w[n] = 0.54 − 0.46 * cos(2π n / (N−1)),  n = 0 … N−1
   */
  [[nodiscard]] static std::vector<float> hamming_window(uint32_t N);

  [[nodiscard]] const ChirpParams& params() const noexcept { return params_; }
  [[nodiscard]] uint32_t num_samples() const noexcept { return num_samples_; }

 private:
  ChirpParams params_;
  uint32_t num_samples_;  ///< Number of samples in one pulse = floor(τ * fs)
};
