#pragma once

/**
 * @file terminal_dashboard.hpp
 * @brief ncurses-based terminal dashboard refreshed at 5 Hz.
 *
 * Layout:
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │ Range-Doppler ASCII heatmap (top-left, largest pane)         │
 *  ├──────────────────────┬──────────────────────────────────────┤
 *  │ Detection table      │ Track list                            │
 *  ├──────────────────────┴──────────────────────────────────────┤
 *  │ Stage latency bar graph                                      │
 *  ├─────────────────────────────────────────────────────────────┤
 *  │ Status bar: CPI rate | active tracks | uptime | jitter p99   │
 *  └─────────────────────────────────────────────────────────────┘
 */

#include <chrono>
#include <memory>
#include <vector>

#include "radar/cfar_detector.hpp"
#include "radar/perf_counter.hpp"
#include "radar/radar_pipeline.hpp"
#include "radar/target.hpp"

/**
 * @brief Manages the ncurses terminal UI for the radar pipeline.
 */
class TerminalDashboard {
 public:
  explicit TerminalDashboard(const PerfCounter* perf);
  ~TerminalDashboard();

  TerminalDashboard(const TerminalDashboard&) = delete;
  TerminalDashboard& operator=(const TerminalDashboard&) = delete;

  /**
   * @brief Redraw the dashboard with the latest pipeline output.
   * @param output  Latest PipelineOutput from the pipeline.
   * @param cpi_rate  CPI completions per second.
   */
  void update(const PipelineOutput& output, float cpi_rate);

  /// @brief Return true if the user pressed 'q'.
  [[nodiscard]] bool should_quit() const noexcept { return quit_; }

 private:
  void draw_rd_heatmap(const PipelineOutput& output);
  void draw_detection_table(const std::vector<Detection>& dets);
  void draw_track_table(const std::vector<Track>& tracks);
  void draw_latency_bars(const std::vector<StageStats>& stats);
  void draw_status_bar(float cpi_rate, int active_tracks, double uptime_s);

  const PerfCounter* perf_;
  std::chrono::steady_clock::time_point start_time_;
  bool quit_{false};
  bool initialized_{false};

  // Gradient characters for ASCII heatmap
  static constexpr const char* kGradient = " .:;+*#@";
  static constexpr int kGradientLen = 8;
};
