#pragma once

/**
 * @file track_manager.hpp
 * @brief Nearest-neighbour track-before-detect tracker.
 *
 * Association logic:
 *   - For each detection, find the nearest existing track within the gate.
 *   - If associated: update track position (α-β filter), increment hits.
 *   - If unassociated: create a new tentative track.
 *
 * Track lifecycle:
 *   - Tentative → Confirmed: after kInitHits consecutive hits.
 *   - Confirmed  → Coasted:  after first miss.
 *   - Any       → Dropped:   after kMaxMisses consecutive misses.
 *   - Dropped tracks are removed from the active list.
 *
 * Thread-safety: guarded by a single std::mutex.
 * Pipeline thread calls update(); dashboard thread calls get_tracks().
 */

#include <mutex>
#include <vector>

#include "radar/target.hpp"
#include "radar/cfar_detector.hpp"

/**
 * @brief Track manager configuration.
 */
struct TrackManagerConfig {
  float gate_range_m{50.0f};      ///< Maximum range difference for association [m]
  float gate_velocity_ms{5.0f};   ///< Maximum velocity difference [m/s]
  int init_hits{2};               ///< Hits required to confirm a track
  int max_misses{3};              ///< Misses before dropping a track
};

/**
 * @brief Thread-safe nearest-neighbour track manager.
 */
class TrackManager {
 public:
  explicit TrackManager(TrackManagerConfig cfg = {});

  /**
   * @brief Update tracker with the latest CFAR detections.
   * @param detections  Output of CfarDetector::detect()
   */
  void update(const std::vector<Detection>& detections);

  /// @brief Snapshot of the current track list (thread-safe).
  [[nodiscard]] std::vector<Track> get_tracks() const;

  /// @brief Number of confirmed active tracks.
  [[nodiscard]] int active_track_count() const;

 private:
  TrackManagerConfig cfg_;
  mutable std::mutex mtx_;
  std::vector<Track> tracks_;
  int next_id_{1};

  /// @brief α-β filter state update for a single track.
  static void update_track_state(Track& t, const Detection& d) noexcept;

  /// @brief Euclidean-ish gate check.
  [[nodiscard]] bool in_gate(const Track& t, const Detection& d) const noexcept;
};
