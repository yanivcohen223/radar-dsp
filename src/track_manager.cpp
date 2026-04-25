#include "radar/track_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

TrackManager::TrackManager(TrackManagerConfig cfg) : cfg_(cfg) {}

bool TrackManager::in_gate(const Track& t, const Detection& d) const noexcept {
  const float dr = std::fabs(t.range_m - d.range_m);
  const float dv = std::fabs(t.velocity_ms - d.velocity_ms);
  return dr <= cfg_.gate_range_m && dv <= cfg_.gate_velocity_ms;
}

void TrackManager::update_track_state(Track& t, const Detection& d) noexcept {
  // α-β filter: smooth position and velocity estimates.
  // α controls range smoothing, β controls velocity smoothing.
  // Simple fixed values — production would tune based on track age/SNR.
  constexpr float kAlpha = 0.6f;
  constexpr float kBeta = 0.1f;

  const float range_err = d.range_m - t.range_m;
  const float vel_err = d.velocity_ms - t.velocity_ms;

  // Filtered state update:
  //   x̂(n) = x̂(n−) + α * (z(n) − x̂(n−))
  t.range_m += kAlpha * range_err;
  t.velocity_ms += kBeta * vel_err;
}

void TrackManager::update(const std::vector<Detection>& detections) {
  std::lock_guard<std::mutex> lock(mtx_);

  // Mark all tracks as not associated this update
  std::vector<bool> track_associated(tracks_.size(), false);
  std::vector<bool> det_associated(detections.size(), false);

  // Nearest-neighbour association
  for (std::size_t di = 0; di < detections.size(); ++di) {
    const auto& d = detections[di];
    float best_dist = std::numeric_limits<float>::max();
    std::size_t best_ti = tracks_.size();  // sentinel = no match

    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (!in_gate(tracks_[ti], d)) continue;

      // Gating distance metric: normalised Euclidean in range-velocity space
      const float dr = (d.range_m - tracks_[ti].range_m) / cfg_.gate_range_m;
      const float dv = (d.velocity_ms - tracks_[ti].velocity_ms) / cfg_.gate_velocity_ms;
      const float dist = std::sqrt(dr * dr + dv * dv);

      if (dist < best_dist) {
        best_dist = dist;
        best_ti = ti;
      }
    }

    if (best_ti < tracks_.size()) {
      // Associate detection with track
      track_associated[best_ti] = true;
      det_associated[di] = true;
      auto& t = tracks_[best_ti];
      update_track_state(t, d);
      t.miss_count = 0;
      ++t.hit_count;
      ++t.age;

      if (t.status == TrackStatus::Tentative && t.hit_count >= cfg_.init_hits) {
        t.status = TrackStatus::Confirmed;
      } else if (t.status == TrackStatus::Coasted) {
        t.status = TrackStatus::Confirmed;
      }
    }
  }

  // Handle unassociated tracks (misses)
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (!track_associated[ti]) {
      auto& t = tracks_[ti];
      ++t.miss_count;
      ++t.age;
      t.hit_count = 0;
      if (t.status == TrackStatus::Confirmed) {
        t.status = TrackStatus::Coasted;
      }
      if (t.miss_count >= cfg_.max_misses) {
        t.status = TrackStatus::Dropped;
      }
    }
  }

  // Spawn new tentative tracks for unassociated detections
  for (std::size_t di = 0; di < detections.size(); ++di) {
    if (!det_associated[di]) {
      Track t{};
      t.id = next_id_++;
      t.range_m = detections[di].range_m;
      t.velocity_ms = detections[di].velocity_ms;
      t.age = 1;
      t.hit_count = 1;
      t.miss_count = 0;
      t.status = TrackStatus::Tentative;
      tracks_.push_back(t);
    }
  }

  // Remove dropped tracks
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [](const Track& t) {
                                 return t.status == TrackStatus::Dropped;
                               }),
                tracks_.end());
}

std::vector<Track> TrackManager::get_tracks() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return tracks_;
}

int TrackManager::active_track_count() const {
  std::lock_guard<std::mutex> lock(mtx_);
  int count = 0;
  for (const auto& t : tracks_) {
    if (t.status == TrackStatus::Confirmed) ++count;
  }
  return count;
}
