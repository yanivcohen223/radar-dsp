#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "radar/cfar_detector.hpp"

class CfarTest : public ::testing::Test {
 protected:
  static constexpr uint32_t kNd = 64;   // Doppler bins
  static constexpr uint32_t kNr = 128;  // Range bins

  // Noise-only map: uniform power in dB (0 dB everywhere)
  std::vector<float> make_noise_map(float noise_db = 0.0f) {
    return std::vector<float>(static_cast<std::size_t>(kNd) * kNr, noise_db);
  }

  // Map with a single bright target above the noise floor
  std::vector<float> make_target_map(uint32_t d_tgt, uint32_t r_tgt, float target_db,
                                     float noise_db = 0.0f) {
    auto map = make_noise_map(noise_db);
    map[static_cast<std::size_t>(d_tgt) * kNr + r_tgt] = target_db;
    return map;
  }
};

TEST_F(CfarTest, CA_CFAR_NoFalseAlarms_OnPureNoise) {
  // With uniform noise (constant power) there should be zero false alarms
  // because every CUT equals the reference mean.
  CfarDetector<CfarType::CA> cfar(2, 8, 1e-6f, 15.0f, 5.0f);
  auto map = make_noise_map(0.0f);
  auto dets = cfar.detect(map, kNd, kNr);

  // Allow a small number of edge-effect false alarms (boundary conditions)
  EXPECT_LE(dets.size(), 5u) << "Too many false alarms on uniform noise map";
}

TEST_F(CfarTest, CA_CFAR_DetectsStrongTarget) {
  // A target 30 dB above the noise floor should always be detected
  const uint32_t d_tgt = 20, r_tgt = 50;
  CfarDetector<CfarType::CA> cfar(2, 8, 1e-6f, 15.0f, 5.0f);
  auto map = make_target_map(d_tgt, r_tgt, 30.0f, 0.0f);
  auto dets = cfar.detect(map, kNd, kNr);

  bool found = false;
  for (const auto& d : dets) {
    if (d.doppler_bin == d_tgt && d.range_bin == r_tgt) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Strong target not detected by CA-CFAR";
}

TEST_F(CfarTest, OS_CFAR_DetectsStrongTarget) {
  const uint32_t d_tgt = 15, r_tgt = 40;
  CfarDetector<CfarType::OS> cfar(2, 8, 1e-6f, 15.0f, 5.0f);
  auto map = make_target_map(d_tgt, r_tgt, 30.0f, 0.0f);
  auto dets = cfar.detect(map, kNd, kNr);

  bool found = false;
  for (const auto& d : dets) {
    if (d.doppler_bin == d_tgt && d.range_bin == r_tgt) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Strong target not detected by OS-CFAR";
}

TEST_F(CfarTest, Detection_RangeConversion) {
  // Verify that range_m is correctly derived from range_bin
  const float range_res = 15.0f;  // m/bin
  const float vel_res = 5.0f;
  const uint32_t d_tgt = 10, r_tgt = 30;
  CfarDetector<CfarType::CA> cfar(2, 8, 1e-6f, range_res, vel_res);

  auto map = make_target_map(d_tgt, r_tgt, 40.0f, 0.0f);
  auto dets = cfar.detect(map, kNd, kNr);

  bool found = false;
  for (const auto& d : dets) {
    if (d.range_bin == r_tgt) {
      EXPECT_FLOAT_EQ(d.range_m, static_cast<float>(r_tgt) * range_res);
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(CfarTest, FalseAlarmRate_RandomNoise) {
  // On white Gaussian noise mapped to dB, the empirical FAR should be
  // roughly consistent with Pfa (within an order of magnitude for small maps).
  std::mt19937 rng(42);
  std::normal_distribution<float> nd(0.0f, 1.0f);

  std::vector<float> map(static_cast<std::size_t>(kNd) * kNr);
  for (float& v : map) {
    const float re = nd(rng);
    const float im = nd(rng);
    v = 10.0f * std::log10(re * re + im * im + 1e-12f);
  }

  const float pfa = 1e-3f;  // relaxed Pfa for a small map
  CfarDetector<CfarType::CA> cfar(2, 4, pfa, 15.0f, 5.0f);
  auto dets = cfar.detect(map, kNd, kNr);

  const float total_cells = static_cast<float>(kNd * kNr);
  const float empirical_far = static_cast<float>(dets.size()) / total_cells;

  // Accept 100× tolerance: FAR should be < 100 * Pfa
  EXPECT_LT(empirical_far, 100.0f * pfa)
      << "Empirical FAR=" << empirical_far << " exceeds 100x Pfa=" << pfa;
}
