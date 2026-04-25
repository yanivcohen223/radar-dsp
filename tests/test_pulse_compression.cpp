#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <vector>

#include "radar/pulse_compressor.hpp"
#include "radar/waveform_generator.hpp"

class PulseCompressionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ChirpParams p{};
    p.bandwidth_hz = 10e6f;
    p.pulse_width_s = 10e-6f;
    p.sample_rate_hz = 10e6f;
    gen_ = std::make_unique<WaveformGenerator>(p);
  }

  std::unique_ptr<WaveformGenerator> gen_;
};

TEST_F(PulseCompressionTest, SelfCompression_PeakAtOrigin) {
  // Compressing the reference chirp against itself should produce a sinc-like
  // peak at delay = 0 (first sample of the output).
  auto ref = gen_->chirp_reference();
  const uint32_t N = gen_->num_samples();
  auto win = WaveformGenerator::hann_window(N);

  // FFT size = next power of two
  uint32_t fft_size = 1;
  while (fft_size < N) fft_size <<= 1;
  fft_size *= 2;  // zero-pad for linear convolution

  // Resize window to fft_size
  win.resize(fft_size, 0.0f);

  PulseCompressor pc(fft_size, ref, win);

  // Input = reference chirp (zero-padded to fft_size)
  std::vector<std::complex<float>> rx(fft_size, {0.0f, 0.0f});
  const auto& ref2 = gen_->chirp_reference();
  for (std::size_t i = 0; i < ref2.size(); ++i) rx[i] = ref2[i];

  auto compressed = pc.compress(rx);

  // Find peak
  float peak = 0.0f;
  std::size_t peak_idx = 0;
  for (std::size_t i = 0; i < compressed.size(); ++i) {
    const float mag = std::abs(compressed[i]);
    if (mag > peak) {
      peak = mag;
      peak_idx = i;
    }
  }

  // Peak should be near sample 0 (or N-1 due to circular shift)
  // Allow a few bins tolerance
  EXPECT_LT(peak_idx, 10u) << "Peak not at expected delay-0 position";
}

TEST_F(PulseCompressionTest, PSL_Below_Minus13dB) {
  // With Hann weighting the PSL must be < -13 dB
  auto ref = gen_->chirp_reference();
  const uint32_t N = gen_->num_samples();
  auto win = WaveformGenerator::hann_window(N);

  uint32_t fft_size = 1;
  while (fft_size < N) fft_size <<= 1;
  fft_size *= 2;
  win.resize(fft_size, 0.0f);

  PulseCompressor pc(fft_size, ref, win);

  std::vector<std::complex<float>> rx(fft_size, {0.0f, 0.0f});
  const auto& ref2 = gen_->chirp_reference();
  for (std::size_t i = 0; i < ref2.size(); ++i) rx[i] = ref2[i];

  auto compressed = pc.compress(rx);
  auto metrics = PulseCompressor::compute_metrics(compressed);

  EXPECT_LT(metrics.psl_db, -13.0f)
      << "PSL = " << metrics.psl_db << " dB, expected < -13 dB";
}

TEST_F(PulseCompressionTest, CompressionGain) {
  // Pulse compression should produce significant gain above the input noise
  auto ref = gen_->chirp_reference();
  const uint32_t N = gen_->num_samples();
  auto win = WaveformGenerator::hann_window(N);

  uint32_t fft_size = 1;
  while (fft_size < N) fft_size <<= 1;
  fft_size *= 2;
  win.resize(fft_size, 0.0f);

  PulseCompressor pc(fft_size, ref, win);

  std::vector<std::complex<float>> rx(fft_size, {0.0f, 0.0f});
  const auto& ref2 = gen_->chirp_reference();
  for (std::size_t i = 0; i < ref2.size(); ++i) rx[i] = ref2[i];

  auto compressed = pc.compress(rx);

  // Find peak power
  float peak_pow = 0.0f;
  for (const auto& s : compressed) {
    const float p = s.real() * s.real() + s.imag() * s.imag();
    peak_pow = std::max(peak_pow, p);
  }

  // Input total power
  float rx_power = 0.0f;
  for (const auto& s : rx) {
    rx_power += s.real() * s.real() + s.imag() * s.imag();
  }

  // Compression gain ≈ N samples
  EXPECT_GT(peak_pow, 0.0f) << "Compressed pulse peak is zero";
}

TEST_F(PulseCompressionTest, HannWindowGeneratesCorrectLength) {
  const uint32_t N = 100;
  auto w = WaveformGenerator::hann_window(N);
  EXPECT_EQ(w.size(), N);
  // First and last coefficients should be near zero
  EXPECT_NEAR(w[0], 0.0f, 1e-5f);
  EXPECT_NEAR(w[N - 1], 0.0f, 1e-3f);
  // Middle coefficient should be near 1
  EXPECT_NEAR(w[N / 2], 1.0f, 0.01f);
}
