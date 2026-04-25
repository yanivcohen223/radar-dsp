#include "radar/terminal_dashboard.hpp"

#include <ncurses.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

TerminalDashboard::TerminalDashboard(const PerfCounter* perf)
    : perf_(perf), start_time_(std::chrono::steady_clock::now()) {
  // Initialise ncurses
  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);  // non-blocking getch
  keypad(stdscr, TRUE);
  curs_set(0);  // hide cursor

  if (has_colors()) {
    start_color();
    // Color pairs: 1=header, 2=confirmed track, 3=tentative, 4=warning, 5=dim
    init_pair(1, COLOR_CYAN, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);
    init_pair(5, COLOR_WHITE, COLOR_BLACK);
  }

  initialized_ = true;
}

TerminalDashboard::~TerminalDashboard() {
  if (initialized_) {
    endwin();
  }
}

void TerminalDashboard::update(const PipelineOutput& output, float cpi_rate) {
  int ch = getch();
  if (ch == 'q' || ch == 'Q') {
    quit_ = true;
    return;
  }

  erase();

  const int rows = LINES;
  const int cols = COLS;
  (void)rows;

  // Split layout:
  //   Top 40%: RD heatmap
  //   Middle left 25%: detections
  //   Middle right 25%: tracks
  //   Bottom 20%: latency bars
  //   Last line: status bar

  const int heatmap_rows = std::max(10, LINES * 40 / 100);
  const int mid_rows = std::max(8, LINES * 25 / 100);
  [[maybe_unused]] const int bar_rows = std::max(6, LINES * 15 / 100);

  // ── Range-Doppler heatmap ─────────────────────────────────────────────────
  move(0, 0);
  attron(COLOR_PAIR(1) | A_BOLD);
  mvprintw(0, 2, "[ Range-Doppler Map (CPI #%lu) ]",
           static_cast<unsigned long>(output.cpi_index));
  attroff(COLOR_PAIR(1) | A_BOLD);

  draw_rd_heatmap(output);

  // ── Detection table ───────────────────────────────────────────────────────
  const int det_y = heatmap_rows + 1;
  attron(COLOR_PAIR(1) | A_BOLD);
  mvprintw(det_y, 0, "DETECTIONS (%zu)",
           output.detections.size());
  attroff(COLOR_PAIR(1) | A_BOLD);
  draw_detection_table(output.detections);

  // ── Track table ───────────────────────────────────────────────────────────
  const int track_x = cols / 2;
  attron(COLOR_PAIR(1) | A_BOLD);
  mvprintw(det_y, track_x, "TRACKS (%zu)", output.tracks.size());
  attroff(COLOR_PAIR(1) | A_BOLD);
  draw_track_table(output.tracks);

  // ── Latency bars ──────────────────────────────────────────────────────────
  const int bar_y = det_y + mid_rows + 1;
  attron(COLOR_PAIR(1) | A_BOLD);
  mvprintw(bar_y, 0, "PIPELINE LATENCY");
  attroff(COLOR_PAIR(1) | A_BOLD);
  auto stats = perf_->compute_stats();
  draw_latency_bars(stats);

  // ── Status bar ────────────────────────────────────────────────────────────
  int active = 0;
  for (const auto& t : output.tracks) {
    if (t.status == TrackStatus::Confirmed) ++active;
  }
  const double uptime_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  draw_status_bar(cpi_rate, active, uptime_s);

  refresh();
}

void TerminalDashboard::draw_rd_heatmap(const PipelineOutput& output) {
  if (output.rd_map_db.empty()) return;

  const int max_rows = std::max(8, LINES * 40 / 100) - 2;
  const int max_cols = COLS - 2;

  const uint32_t nd = output.num_doppler;
  const uint32_t nr = output.num_range;

  // Find dynamic range for colour mapping
  float vmin = output.rd_map_db[0];
  float vmax = vmin;
  for (float v : output.rd_map_db) {
    vmin = std::min(vmin, v);
    vmax = std::max(vmax, v);
  }
  const float range = vmax - vmin + 1e-6f;

  // Down-sample to terminal size
  const float dr_step = static_cast<float>(nd) / static_cast<float>(max_rows);
  const float dc_step = static_cast<float>(nr) / static_cast<float>(max_cols);

  for (int row = 0; row < max_rows; ++row) {
    const uint32_t d = static_cast<uint32_t>(static_cast<float>(row) * dr_step);
    for (int col = 0; col < max_cols; ++col) {
      const uint32_t r = static_cast<uint32_t>(static_cast<float>(col) * dc_step);
      const std::size_t idx = static_cast<std::size_t>(d) * nr + r;
      if (idx >= output.rd_map_db.size()) continue;

      const float norm = (output.rd_map_db[idx] - vmin) / range;
      const int gi = static_cast<int>(norm * static_cast<float>(kGradientLen - 1));
      const char c = kGradient[std::clamp(gi, 0, kGradientLen - 1)];

      // Colour intensity based on brightness
      const int pair = (gi >= 6) ? 4 : (gi >= 4) ? 3 : (gi >= 2) ? 2 : 5;
      attron(COLOR_PAIR(pair));
      mvaddch(row + 2, col + 1, static_cast<chtype>(c));
      attroff(COLOR_PAIR(pair));
    }
  }
}

void TerminalDashboard::draw_detection_table(const std::vector<Detection>& dets) {
  const int base_y = std::max(10, LINES * 40 / 100) + 2;
  const int max_rows = std::max(8, LINES * 25 / 100) - 1;
  const int max_cols = COLS / 2 - 1;

  attron(A_UNDERLINE);
  mvprintw(base_y, 0, "%-8s %-8s %-6s", "Range(m)", "Vel(m/s)", "SNR(dB)");
  attroff(A_UNDERLINE);

  const int n = std::min(static_cast<int>(dets.size()), max_rows);
  for (int i = 0; i < n; ++i) {
    const auto& d = dets[static_cast<std::size_t>(i)];
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%-8.1f %-8.1f %-6.1f", d.range_m, d.velocity_ms, d.snr_db);
    mvprintw(base_y + 1 + i, 0, "%.*s", max_cols, buf);
  }
}

void TerminalDashboard::draw_track_table(const std::vector<Track>& tracks) {
  const int base_y = std::max(10, LINES * 40 / 100) + 2;
  const int max_rows = std::max(8, LINES * 25 / 100) - 1;
  const int x = COLS / 2;

  attron(A_UNDERLINE);
  mvprintw(base_y, x, "%-4s %-8s %-8s %-4s %-5s", "ID", "Range", "Vel", "Age", "Stat");
  attroff(A_UNDERLINE);

  const int n = std::min(static_cast<int>(tracks.size()), max_rows);
  for (int i = 0; i < n; ++i) {
    const auto& t = tracks[static_cast<std::size_t>(i)];
    const int pair = (t.status == TrackStatus::Confirmed) ? 2
                   : (t.status == TrackStatus::Tentative)  ? 3
                   : 4;
    attron(COLOR_PAIR(pair));
    mvprintw(base_y + 1 + i, x, "%-4d %-8.1f %-8.1f %-4d %-5s", t.id, t.range_m,
             t.velocity_ms, t.age, track_status_str(t.status));
    attroff(COLOR_PAIR(pair));
  }
}

void TerminalDashboard::draw_latency_bars(const std::vector<StageStats>& stats) {
  const int base_y = std::max(10, LINES * 40 / 100) + std::max(8, LINES * 25 / 100) + 3;
  const int bar_width = COLS - 30;

  float max_val = 1.0f;
  for (const auto& s : stats) max_val = std::max(max_val, static_cast<float>(s.p99_us));

  int row = 0;
  for (const auto& s : stats) {
    // p99 bar
    const int bar_len = static_cast<int>(static_cast<float>(bar_width) *
                                         static_cast<float>(s.p99_us) / max_val);
    mvprintw(base_y + row, 0, "%-20s", s.name.substr(0, 20).c_str());
    attron(COLOR_PAIR(2));
    for (int b = 0; b < std::min(bar_len, bar_width); ++b) {
      mvaddch(base_y + row, 20 + b, ACS_BLOCK);
    }
    attroff(COLOR_PAIR(2));
    mvprintw(base_y + row, 20 + bar_width + 1, "p99=%.0fµs", s.p99_us);
    ++row;
  }
}

void TerminalDashboard::draw_status_bar(float cpi_rate, int active_tracks, double uptime_s) {
  const int y = LINES - 1;
  attron(COLOR_PAIR(1) | A_REVERSE);

  char buf[256];
  std::snprintf(buf, sizeof(buf),
                " CPI: %.1f Hz | Tracks: %d | Uptime: %.0fs | Press q to quit ",
                cpi_rate, active_tracks, uptime_s);
  mvprintw(y, 0, "%-*s", COLS - 1, buf);
  attroff(COLOR_PAIR(1) | A_REVERSE);
}
