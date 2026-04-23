#include <gtest/gtest.h>

#include <complex>
#include <cmath>
#include <vector>

#include "dsp/range_doppler.hpp"
#include "waveform/waveform_generator.hpp"

class RangeDopplerTest : public ::testing::Test {
 protected:
  static constexpr uint32_t kNp = 64;   // pulses
  static constexpr uint32_t kNr = 256;  // range bins
  static constexpr float kPrt = 200e-6f;

  void SetUp() override {
    rdp_ = std::make_unique<RangeDopplerProcessor>(kNp, kNr, kPrt);
  }

  std::unique_ptr<RangeDopplerProcessor> rdp_;
};

TEST_F(RangeDopplerTest, StaticTarget_AppearsAtBin0) {
  // A static target (zero Doppler) with constant amplitude across all pulses
  // should concentrate in Doppler bin 0.
  const uint32_t target_range_bin = 50;
  const float amplitude = 10.0f;

  std::vector<std::complex<float>> cpi(static_cast<std::size_t>(kNp) * kNr,
                                       {0.0f, 0.0f});
  for (uint32_t p = 0; p < kNp; ++p) {
    cpi[static_cast<std::size_t>(p) * kNr + target_range_bin] = {amplitude, 0.0f};
  }

  auto rd = rdp_->process(cpi);
  auto db = RangeDopplerProcessor::to_db(rd);

  // DC bin (0) at the target range should be the maximum
  float dc_val = db[static_cast<std::size_t>(0) * kNr + target_range_bin];

  // Check all other Doppler bins at the same range bin are lower
  for (uint32_t d = 1; d < kNp; ++d) {
    float val = db[static_cast<std::size_t>(d) * kNr + target_range_bin];
    EXPECT_LT(val, dc_val + 1.0f)  // allow 1 dB tolerance for windowing
        << "Doppler bin " << d << " is unexpectedly high";
  }
}

TEST_F(RangeDopplerTest, DopplerTarget_AppearsAtCorrectBin) {
  // A target with a Doppler phase ramp of k cycles/CPI should appear at bin k.
  const uint32_t target_range_bin = 80;
  const uint32_t target_doppler_bin = 8;  // 8 cycles across 64 pulses
  const float amplitude = 5.0f;

  std::vector<std::complex<float>> cpi(static_cast<std::size_t>(kNp) * kNr,
                                       {0.0f, 0.0f});
  for (uint32_t p = 0; p < kNp; ++p) {
    // Phase ramp: exp(j * 2π * k * p / Np)
    const float phi =
        2.0f * static_cast<float>(M_PI) * static_cast<float>(target_doppler_bin) *
        static_cast<float>(p) / static_cast<float>(kNp);
    cpi[static_cast<std::size_t>(p) * kNr + target_range_bin] = {amplitude * std::cos(phi),
                                                                   amplitude * std::sin(phi)};
  }

  auto rd = rdp_->process(cpi);
  auto db = RangeDopplerProcessor::to_db(rd);

  // Find peak in range slice at target range bin
  float peak_val = -1e9f;
  uint32_t peak_bin = 0;
  for (uint32_t d = 0; d < kNp; ++d) {
    float val = db[static_cast<std::size_t>(d) * kNr + target_range_bin];
    if (val > peak_val) {
      peak_val = val;
      peak_bin = d;
    }
  }

  EXPECT_EQ(peak_bin, target_doppler_bin)
      << "Expected peak at Doppler bin " << target_doppler_bin
      << " but found at bin " << peak_bin;
}

TEST_F(RangeDopplerTest, FftShift_CentresDC) {
  // After fftshift, the previously-bin-0 peak should be at row Np/2.
  const uint32_t target_range_bin = 30;
  std::vector<std::complex<float>> cpi(static_cast<std::size_t>(kNp) * kNr, {0.0f, 0.0f});
  for (uint32_t p = 0; p < kNp; ++p) {
    cpi[static_cast<std::size_t>(p) * kNr + target_range_bin] = {1.0f, 0.0f};
  }

  auto rd = rdp_->process(cpi);
  auto db = RangeDopplerProcessor::to_db(rd);
  auto shifted = RangeDopplerProcessor::fftshift_doppler(db, kNp, kNr);

  // After shift, DC should be at row kNp/2
  const uint32_t dc_row = kNp / 2;
  float dc_val = shifted[static_cast<std::size_t>(dc_row) * kNr + target_range_bin];

  // Verify it's the maximum in the column
  for (uint32_t d = 0; d < kNp; ++d) {
    if (d == dc_row) continue;
    EXPECT_LE(shifted[static_cast<std::size_t>(d) * kNr + target_range_bin], dc_val + 1.0f);
  }
}

TEST_F(RangeDopplerTest, HammingWindow_CorrectLength) {
  auto w = WaveformGenerator::hamming_window(kNp);
  EXPECT_EQ(w.size(), kNp);
  // Hamming window coefficients should be in range [0.08, 1.0]
  for (float v : w) {
    EXPECT_GE(v, 0.07f);
    EXPECT_LE(v, 1.01f);
  }
}
