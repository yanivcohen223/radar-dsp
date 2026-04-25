#pragma once

/**
 * @file target.hpp
 * @brief Data types for radar tracking.
 */

#include <cstdint>
#include <string>

/// @brief Status of a track in the track manager.
enum class TrackStatus { Tentative, Confirmed, Coasted, Dropped };

inline const char* track_status_str(TrackStatus s) {
  switch (s) {
    case TrackStatus::Tentative: return "TENT";
    case TrackStatus::Confirmed: return "CONF";
    case TrackStatus::Coasted:   return "COAS";
    case TrackStatus::Dropped:   return "DROP";
    default:                     return "????";
  }
}

/// @brief One maintained radar track.
struct Track {
  int id{0};
  float range_m{0.0f};
  float velocity_ms{0.0f};
  int age{0};           ///< Number of CPI updates (including misses)
  int hit_count{0};     ///< Consecutive detections
  int miss_count{0};    ///< Consecutive misses
  TrackStatus status{TrackStatus::Tentative};
};
