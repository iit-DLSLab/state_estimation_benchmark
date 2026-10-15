#pragma once

// Per-run CSV logging for reproducible performance analysis.
// Files written in <directory>:
//   samples.csv     one row per estimator update (identity, all timestamps, derived
//                   durations, queue occupancy, state, diagnostics)
//   events.csv      drops, gaps, resets, late results (with reason)
//   run_config.yaml configuration of the run (written by the caller)
// Writing happens in a dedicated thread; producers never block: if the log queue
// is full the record is counted in dropped_records() (reported in the summary).

#include "seb_online/bounded_queue.hpp"
#include "seb_online/estimator_worker.hpp"
#include "seb_online/types.hpp"

#include <atomic>
#include <fstream>
#include <string>
#include <thread>
#include <variant>

namespace seb {

class RunLogger {
 public:
  RunLogger(const std::string& directory, std::size_t queue_capacity);
  ~RunLogger();

  RunLogger(const RunLogger&) = delete;
  RunLogger& operator=(const RunLogger&) = delete;

  const std::string& directory() const { return directory_; }

  void start();
  void stop();  // flushes queued records

  void logSample(const StateEstimate& s);
  void logEvent(const PipelineEvent& e);
  void writeTextFile(const std::string& name, const std::string& content);

  std::uint64_t droppedRecords() const { return dropped_records_.load(); }

 private:
  using Record = std::variant<StateEstimate, PipelineEvent>;
  void run();
  void writeSample(const StateEstimate& s);
  void writeEvent(const PipelineEvent& e);

  std::string directory_;
  BoundedQueue<Record> queue_;
  std::ofstream samples_;
  std::ofstream events_;
  std::atomic<std::uint64_t> dropped_records_{0};
  std::thread thread_;
};

}  // namespace seb
