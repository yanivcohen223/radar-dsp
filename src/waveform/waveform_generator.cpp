#include "waveform_generator.hpp"

#include <cmath>

static constexpr float kPi = static_cast<float>(M_PI);

WaveformGenerator::WaveformGenerator(ChirpParams params)
    : params_(params),
      num_samples_(static_cast<uint32_t>(params.pulse_width_s * params.sample_rate_hz)) {}

std::vector<std::complex<float>> WaveformGenerator::chirp_reference() const {
  // LFM up-chirp:
  //   s(t) = exp(j * π * k * t²)
  //   where k = B / τ  [Hz/s]
  const float k = params_.bandwidth_hz / params_.pulse_width_s;
  const float ts = 1.0f / params_.sample_rate_hz;

  std::vector<std::complex<float>> ref(num_samples_);
  for (uint32_t n = 0; n < num_samples_; ++n) {
    const float t = static_cast<float>(n) * ts;
    const float phase = kPi * k * t * t;  // π * k * t²
    ref[n] = {std::cos(phase), std::sin(phase)};
  }
  return ref;
}

std::vector<float> WaveformGenerator::hann_window(uint32_t N) {
  // w[n] = 0.5 * (1 − cos(2π n / (N−1)))
  std::vector<float> w(N);
  const float scale = 2.0f * kPi / static_cast<float>(N - 1);
  for (uint32_t n = 0; n < N; ++n) {
    w[n] = 0.5f * (1.0f - std::cos(scale * static_cast<float>(n)));
  }
  return w;
}

std::vector<float> WaveformGenerator::hamming_window(uint32_t N) {
  // w[n] = 0.54 − 0.46 * cos(2π n / (N−1))
  std::vector<float> w(N);
  const float scale = 2.0f * kPi / static_cast<float>(N - 1);
  for (uint32_t n = 0; n < N; ++n) {
    w[n] = 0.54f - 0.46f * std::cos(scale * static_cast<float>(n));
  }
  return w;
}
