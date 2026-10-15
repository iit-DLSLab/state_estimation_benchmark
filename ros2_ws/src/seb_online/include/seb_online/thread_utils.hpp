#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace seb {

inline std::int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Optional scheduling settings of a worker thread. Nothing is applied unless
// configured (no machine-specific defaults).
struct ThreadSettings {
  int cpu = -1;           // CPU to pin the thread to; -1 = no affinity
  int rt_priority = 0;    // SCHED_FIFO priority (1..99); 0 = keep the default scheduler
};

// Applies the settings to the calling thread and names it (Linux: 15 chars).
// Returns an empty string on success or a description of what failed (the
// thread keeps running with the default settings).
std::string applyThreadSettings(const std::string& name, const ThreadSettings& settings);

}  // namespace seb
