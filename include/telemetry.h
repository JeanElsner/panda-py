#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

/// Single-producer, single-consumer ring buffer for 1 kHz telemetry.
///
/// The control loop is the producer and never blocks or allocates: it claims
/// a slot, fills it and publishes it. When the ring is full the sample is
/// dropped and counted rather than overwriting unread ones, so a reader that
/// falls behind sees a gap in the tick counter, never silently reordered data.
/// Readers drain under their own lock, so several Python threads may call it.
template <typename T>
class TelemetryRing {
 public:
  explicit TelemetryRing(size_t capacity) : buffer_(capacity ? capacity + 1 : 0) {}

  bool enabled() const { return !buffer_.empty(); }
  size_t capacity() const { return enabled() ? buffer_.size() - 1 : 0; }

  /// Producer: a slot to fill, or nullptr when the ring is full or disabled.
  T* claim() {
    if (!enabled()) {
      return nullptr;
    }
    const size_t head = head_.load(std::memory_order_relaxed);
    if (next(head) == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return nullptr;
    }
    return &buffer_[head];
  }

  /// Producer: makes the slot returned by the last claim() readable.
  void publish() {
    const size_t head = head_.load(std::memory_order_relaxed);
    head_.store(next(head), std::memory_order_release);
  }

  /// Consumer: appends every published sample to out and frees its slot.
  size_t drain(std::vector<T>& out) {
    std::lock_guard<std::mutex> lock(consumer_mux_);
    size_t tail = tail_.load(std::memory_order_relaxed);
    const size_t head = head_.load(std::memory_order_acquire);
    size_t count = 0;
    while (tail != head) {
      out.push_back(buffer_[tail]);
      tail = next(tail);
      count++;
    }
    tail_.store(tail, std::memory_order_release);
    return count;
  }

  /// Samples the producer could not store because the ring was full.
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  size_t next(size_t i) const { return i + 1 == buffer_.size() ? 0 : i + 1; }

  std::vector<T> buffer_;
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
  std::atomic<uint64_t> dropped_{0};
  std::mutex consumer_mux_;
};
