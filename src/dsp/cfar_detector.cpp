#include "cfar_detector.hpp"

#include <cmath>
#include <numeric>

// ─── Constructor ─────────────────────────────────────────────────────────────

template <CfarType Type>
CfarDetector<Type>::CfarDetector(uint32_t guard_cells, uint32_t ref_cells, float pfa,
                                 float range_res_m, float velocity_res_ms, float os_k)
    : guard_(guard_cells),
      ref_(ref_cells),
      range_res_m_(range_res_m),
      velocity_res_ms_(velocity_res_ms) {
  // CA-CFAR threshold multiplier:
  //   N = total reference cells = (2*window+1)^2 - (2*guard+1)^2
  //   alpha = N * (Pfa^{-1/N} - 1)   [Finn & Johnson, 1968]
  const uint32_t window = guard_ + ref_;
  const uint32_t total_ref = (2 * window + 1) * (2 * window + 1) -
                              (2 * guard_ + 1) * (2 * guard_ + 1);
  const float N = static_cast<float>(total_ref);
  alpha_ = N * (std::pow(pfa, -1.0f / N) - 1.0f);

  // OS-CFAR: use k-th order statistic (default k = 75th percentile)
  os_k_ = static_cast<uint32_t>(os_k * static_cast<float>(total_ref));
  if (os_k_ >= total_ref) os_k_ = total_ref - 1;
}

// ─── Reference cell collection ───────────────────────────────────────────────

template <CfarType Type>
std::vector<float> CfarDetector<Type>::collect_reference(const std::vector<float>& rd_db,
                                                          uint32_t nd, uint32_t nr,
                                                          uint32_t d, uint32_t r) const {
  const uint32_t window = guard_ + ref_;
  std::vector<float> cells;
  cells.reserve(64);

  for (int32_t dd = -static_cast<int32_t>(window); dd <= static_cast<int32_t>(window); ++dd) {
    for (int32_t dr = -static_cast<int32_t>(window); dr <= static_cast<int32_t>(window); ++dr) {
      // Skip guard cells and CUT
      if (std::abs(dd) <= static_cast<int32_t>(guard_) &&
          std::abs(dr) <= static_cast<int32_t>(guard_)) {
        continue;
      }
      const int32_t nd_d = static_cast<int32_t>(d) + dd;
      const int32_t nr_r = static_cast<int32_t>(r) + dr;
      if (nd_d < 0 || nd_d >= static_cast<int32_t>(nd)) continue;
      if (nr_r < 0 || nr_r >= static_cast<int32_t>(nr)) continue;

      // Convert dB to linear power for noise averaging
      const float val_db =
          rd_db[static_cast<std::size_t>(nd_d) * nr + static_cast<std::size_t>(nr_r)];
      cells.push_back(std::pow(10.0f, val_db / 10.0f));
    }
  }
  return cells;
}

// ─── Main detection loop ─────────────────────────────────────────────────────

template <CfarType Type>
std::vector<Detection> CfarDetector<Type>::detect(const std::vector<float>& rd_db, uint32_t nd,
                                                   uint32_t nr) const {
  std::vector<Detection> detections;

  for (uint32_t d = 0; d < nd; ++d) {
    for (uint32_t r = 0; r < nr; ++r) {
      auto ref_cells = collect_reference(rd_db, nd, nr, d, r);
      if (ref_cells.empty()) continue;

      // noise_estimate dispatches at compile time via if constexpr in header
      float noise = const_cast<CfarDetector<Type>*>(this)->noise_estimate(ref_cells);

      // Threshold in linear power
      const float threshold_linear = alpha_ * noise;

      // CUT power (convert from dB)
      const float cut_db = rd_db[static_cast<std::size_t>(d) * nr + r];
      const float cut_linear = std::pow(10.0f, cut_db / 10.0f);

      if (cut_linear > threshold_linear) {
        Detection det{};
        det.range_bin = r;
        det.doppler_bin = d;
        det.range_m = static_cast<float>(r) * range_res_m_;
        det.velocity_ms = static_cast<float>(d) * velocity_res_ms_;
        det.snr_db = (noise > 0.0f) ? 10.0f * std::log10(cut_linear / noise) : 0.0f;
        detections.push_back(det);
      }
    }
  }
  return detections;
}

// ─── Explicit instantiations ─────────────────────────────────────────────────
template class CfarDetector<CfarType::CA>;
template class CfarDetector<CfarType::OS>;
