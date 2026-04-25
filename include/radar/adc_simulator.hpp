#pragma once

/**
 * @file adc_simulator.hpp
 * @brief Simulates a radar ADC producing complex I/Q baseband samples.
 *
 * Models N targets at specified ranges and radial velocities.  AWGN noise
 * is added at a configurable SNR.  Range migration across the CPI is
 * simulated by advancing each target's delay by v*T_prt per pulse.
 */

#include <complex>
#include <cstdint>
#include <random>
#include <vector>

/// @brief Describes one simulated radar target.
struct TargetConfig {
  float range_m;       ///< Initial slant range [m]
  float velocity_ms;   ///< Radial velocity, positive = approaching [m/s]
  float rcs_db;        ///< Radar cross-section relative amplitude [dB]
};

/// @brief Parameters for one coherent processing interval.
struct CpiConfig {
  float sample_rate_hz{10e6f};  ///< ADC sample rate [Hz]
  float chirp_bw_hz{10e6f};     ///< LFM bandwidth [Hz]
  float pulse_width_s{10e-6f};  ///< Pulse duration [s]
  float prt_s{200e-6f};         ///< Pulse repetition time [s]
  uint32_t num_pulses{64};      ///< Pulses per CPI
  uint32_t range_bins{1024};    ///< Samples per pulse (range dimension)
  float snr_db{15.0f};          ///< Signal-to-noise ratio [dB]
};

/**
 * @brief Generates one full CPI of complex I/Q ADC samples.
 *
 * Returns a 2D matrix stored row-major as pulses × range_bins.
 * Each element is std::complex<float> representing baseband I+jQ.
 */
class AdcSimulator {
 public:
  explicit AdcSimulator(CpiConfig cfg, std::vector<TargetConfig> targets, uint32_t seed = 42);

  /**
   * @brief Generate the next CPI.
   * @return Flat vector of size num_pulses × range_bins (row = pulse).
   */
  [[nodiscard]] std::vector<std::complex<float>> generate_cpi();

  [[nodiscard]] const CpiConfig& config() const noexcept { return cfg_; }

 private:
  /// @brief Amplitude of one target given RCS and range.
  [[nodiscard]] float target_amplitude(const TargetConfig& t, float range_m) const noexcept;

  CpiConfig cfg_;
  std::vector<TargetConfig> targets_;
  std::mt19937 rng_;
  std::normal_distribution<float> noise_dist_{0.0f, 1.0f};

  uint64_t cpi_index_{0};  ///< Incremented each call to generate_cpi()
};
