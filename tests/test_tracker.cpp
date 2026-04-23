#include <gtest/gtest.h>

#include <vector>

#include "dsp/cfar_detector.hpp"
#include "tracker/track_manager.hpp"

// Helper: make a detection at given range/velocity
static Detection make_det(float range_m, float velocity_ms, float snr_db = 20.0f) {
  Detection d{};
  d.range_m = range_m;
  d.velocity_ms = velocity_ms;
  d.snr_db = snr_db;
  d.range_bin = static_cast<uint32_t>(range_m / 15.0f);
  d.doppler_bin = 0;
  return d;
}

TEST(TrackerTest, TrackInitiation_After2Hits) {
  TrackManagerConfig cfg{};
  cfg.init_hits = 2;
  cfg.max_misses = 3;
  TrackManager tm(cfg);

  Detection d = make_det(1000.0f, 10.0f);

  // First hit → tentative
  tm.update({d});
  {
    auto tracks = tm.get_tracks();
    ASSERT_EQ(tracks.size(), 1u);
    EXPECT_EQ(tracks[0].status, TrackStatus::Tentative);
  }

  // Second hit → confirmed
  tm.update({d});
  {
    auto tracks = tm.get_tracks();
    ASSERT_EQ(tracks.size(), 1u);
    EXPECT_EQ(tracks[0].status, TrackStatus::Confirmed);
  }
}

TEST(TrackerTest, TrackDrop_After3Misses) {
  TrackManagerConfig cfg{};
  cfg.init_hits = 2;
  cfg.max_misses = 3;
  TrackManager tm(cfg);

  Detection d = make_det(2000.0f, 5.0f);

  // Initiate and confirm
  tm.update({d});
  tm.update({d});

  // Three consecutive misses → dropped
  tm.update({});
  tm.update({});
  tm.update({});

  auto tracks = tm.get_tracks();
  EXPECT_EQ(tracks.size(), 0u) << "Track should be dropped after 3 misses";
}

TEST(TrackerTest, NearestNeighbour_Association) {
  TrackManager tm;
  Detection d1 = make_det(500.0f, 0.0f);
  Detection d2 = make_det(5000.0f, 0.0f);

  // Two separate targets
  tm.update({d1, d2});
  tm.update({d1, d2});

  auto tracks = tm.get_tracks();
  EXPECT_EQ(tracks.size(), 2u);

  // Both should be confirmed
  for (const auto& t : tracks) {
    EXPECT_EQ(t.status, TrackStatus::Confirmed);
  }
}

TEST(TrackerTest, GateReject_OutOfGate) {
  TrackManagerConfig cfg{};
  cfg.gate_range_m = 50.0f;
  cfg.init_hits = 2;
  cfg.max_misses = 3;
  TrackManager tm(cfg);

  // First detection
  tm.update({make_det(1000.0f, 0.0f)});
  // Second detection far outside gate (Δrange = 500 m >> 50 m gate)
  tm.update({make_det(1500.0f, 0.0f)});

  // Should have two separate tracks (unassociated)
  auto tracks = tm.get_tracks();
  EXPECT_EQ(tracks.size(), 2u);
}

TEST(TrackerTest, ThreadSafety_ConcurrentReadWrite) {
  // Basic test: update and get_tracks from different contexts do not crash
  TrackManager tm;
  Detection d = make_det(3000.0f, 20.0f);

  // Run 50 updates and 50 reads interleaved
  for (int i = 0; i < 50; ++i) {
    tm.update({d});
    [[maybe_unused]] auto tracks = tm.get_tracks();
  }
  SUCCEED();
}

TEST(TrackerTest, ActiveTrackCount) {
  TrackManager tm;
  Detection d = make_det(800.0f, 15.0f);

  tm.update({d});
  EXPECT_EQ(tm.active_track_count(), 0);  // tentative only

  tm.update({d});
  EXPECT_EQ(tm.active_track_count(), 1);  // confirmed
}
