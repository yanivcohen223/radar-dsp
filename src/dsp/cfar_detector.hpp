#pragma once

/**
 * @file cfar_detector.hpp
 * @brief 2D CFAR detector supporting CA-CFAR and OS-CFAR variants.
 *
 * CA-CFAR threshold:
 *   T = alpha * mean(reference_cells)
 *   alpha = N * (Pfa^(-1/N) - 1)   [Finn & Johnson, 1968]
 *   where N = total reference cells
 *
 * OS-CFAR: orders the reference cells and uses the k-th order statistic
 *   (typically k = 3N/4) as the noise estimate.  More robust to clutter.
 *
 * Guard cells prevent the main lobe from contaminating the noise estimate.
 * Applied in 2D: guard_cells × guard_cells window around the CUT.
 */

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>

/// @brief Selector for CFAR algorithm variant.
enum class CfarType { CA, OS };

/// @brief One CFAR detection.
struct Detection {
  float range_m{0.0f};
  float velocity_ms{0.0f};
  float snr_db{0.0f};
  uint32_t range_bin{0};
  uint32_t doppler_bin{0};
};

/**
 * @brief Template-based 2D CFAR detector.
 *
 * @tparam Type  CfarType::CA or CfarType::OS
 */
template <CfarType Type = CfarType::CA>
class CfarDetector {
 public:
  /**
   * @param guard_cells    One-sided guard cells (same for range and Doppler)
   * @param ref_cells      One-sided reference cells
   * @param pfa            Target probability of false alarm
   * @param range_res_m    Range resolution [m/bin]
   * @param velocity_res   Velocity resolution [m/s per Doppler bin]
   * @param os_k           OS-CFAR order statistic index (fraction of ref window)
   */
  CfarDetector(uint32_t guard_cells, uint32_t ref_cells, float pfa, float range_res_m,
               float velocity_res_ms, float os_k = 0.75f);

  /**
   * @brief Run 2D CFAR on a Range-Doppler map.
   *
   * @param rd_db   Flat [num_doppler × num_range] dB-magnitude map
   * @param nd      Number of Doppler bins (rows)
   * @param nr      Number of range bins (columns)
   * @return        List of detections
   */
  [[nodiscard]] std::vector<Detection> detect(const std::vector<float>& rd_db, uint32_t nd,
                                              uint32_t nr) const;

 private:
  uint32_t guard_;
  uint32_t ref_;
  float alpha_;         ///< CA-CFAR threshold multiplier
  float range_res_m_;
  float velocity_res_ms_;
  uint32_t os_k_;       ///< OS-CFAR order statistic index

  /// @brief Collect reference cells in 2D window (excludes guard + CUT).
  [[nodiscard]] std::vector<float> collect_reference(const std::vector<float>& rd_db,
                                                     uint32_t nd, uint32_t nr,
                                                     uint32_t d, uint32_t r) const;

  /// @brief Estimate noise from reference cells based on CFAR type.
  /// Uses if constexpr to select CA mean vs OS k-th order statistic.
  [[nodiscard]] float noise_estimate(std::vector<float>& ref_linear) const {
    if (ref_linear.empty()) return 1.0f;
    if constexpr (Type == CfarType::CA) {
      // CA-CFAR: mean of all reference cells
      float sum = 0.0f;
      for (float v : ref_linear) sum += v;
      return sum / static_cast<float>(ref_linear.size());
    } else {
      // OS-CFAR: k-th order statistic — robust to clutter edges
      std::size_t k = std::min(static_cast<std::size_t>(os_k_), ref_linear.size() - 1);
      std::nth_element(ref_linear.begin(),
                       ref_linear.begin() + static_cast<std::ptrdiff_t>(k),
                       ref_linear.end());
      return ref_linear[k];
    }
  }
};

// ─── Explicit instantiations declared here, defined in .cpp ──────────────────
extern template class CfarDetector<CfarType::CA>;
extern template class CfarDetector<CfarType::OS>;
