#pragma once

/**
 * @file lock_free_queue.hpp
 * @brief Single-producer / single-consumer lock-free ring buffer.
 *
 * Uses a fixed-capacity power-of-two ring buffer with two atomic indices
 * (head_ = write cursor, tail_ = read cursor).
 *
 * Memory ordering rationale:
 *   - head_.store uses release  so the element write is visible before the
 *     index advance (pairs with the acquire load in try_pop).
 *   - tail_.store uses release  so the slot is visible as free before the
 *     index advance (pairs with the acquire load in try_push).
 *   - Loads that gate whether the queue is full/empty use acquire to see the
 *     corresponding stores from the other thread.
 *
 * This is the classic Dmitry Vyukov SPSC ring buffer pattern.
 */

#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <optional>
#include <type_traits>

/// @brief SPSC lock-free ring buffer with compile-time capacity (must be power of two).
template <typename T, std::size_t Capacity>
class LockFreeQueue {
  static_assert((Capacity & (Capacity - 1)) == 0,
                "Capacity must be a power of two for bitmask indexing");
  static_assert(std::is_default_constructible_v<T>);

 public:
  LockFreeQueue() : head_(0), tail_(0) {}

  // Non-copyable, non-movable (atomics cannot be moved)
  LockFreeQueue(const LockFreeQueue&) = delete;
  LockFreeQueue& operator=(const LockFreeQueue&) = delete;

  /**
   * @brief Push one element (producer side).
   * @return true on success, false when the queue is full.
   */
  bool try_push(T item) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & kMask;
    // Full check: next write position must not equal the read cursor
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;  // full
    }
    buffer_[head] = std::move(item);
    // Release: the element write must be visible before the index advance
    head_.store(next, std::memory_order_release);
    return true;
  }

  /**
   * @brief Pop one element (consumer side).
   * @return std::optional<T> with the element, or std::nullopt when empty.
   */
  std::optional<T> try_pop() noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    // Empty check
    if (tail == head_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }
    T item = std::move(buffer_[tail]);
    // Release: the slot is free for the producer after the index advance
    tail_.store((tail + 1) & kMask, std::memory_order_release);
    return item;
  }

  /// @return Approximate number of elements (safe to call from any thread).
  [[nodiscard]] std::size_t size_approx() const noexcept {
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    return (head - tail) & kMask;
  }

  [[nodiscard]] bool empty() const noexcept { return size_approx() == 0; }

  static constexpr std::size_t capacity() noexcept { return Capacity - 1; }

 private:
  static constexpr std::size_t kMask = Capacity - 1;

  // Pad to separate cache lines — head_ is written by producer,
  // tail_ is written by consumer; false sharing would add latency on every op.
  alignas(64) std::atomic<std::size_t> head_;
  alignas(64) std::atomic<std::size_t> tail_;

  // Element storage — accessed by both threads but at different indices
  alignas(64) std::array<T, Capacity> buffer_;
};
