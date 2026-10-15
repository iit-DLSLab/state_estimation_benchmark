#pragma once

// Conversion of a StateEstimate into the published ROS messages, and the
// diagnostics accumulator shared by the node and the replay tool.

#include "seb_online/estimator_worker.hpp"
#include "seb_online/pipeline.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <seb_interfaces/msg/estimator_state.hpp>

#include <mutex>
#include <string>

namespace seb {

void toMsg(const StateEstimate& s, const std::string& world_frame, const std::string& base_frame,
           seb_interfaces::msg::EstimatorState& msg);

// Odometry: pose of the base in world, twist in the base frame. Covariances are
// not provided by the benchmark interface and are left at zero; the orientation
// and position semantics are those of EstimatorState.
void toOdometry(const StateEstimate& s, const std::string& world_frame, const std::string& base_frame,
                nav_msgs::msg::Odometry& msg);

// Accumulates per-period statistics of published estimates (thread-safe).
class DiagnosticsAccumulator {
 public:
  void add(const StateEstimate& s);
  // Builds the status for the period since the last call and resets the period.
  diagnostic_msgs::msg::DiagnosticStatus makeStatus(const std::string& name, const std::string& estimator,
                                                    const PipelineCounters& counters, const SynchronizerStats& sync,
                                                    double period_s, std::string* warning);

 private:
  std::mutex mutex_;
  std::uint64_t n_ = 0;
  double exec_sum_ = 0.0, exec_max_ = 0.0;
  double latency_sum_ = 0.0, latency_max_ = 0.0;
  std::size_t queue_max_ = 0;
  std::uint64_t not_valid_ = 0;
  PipelineCounters last_counters_;
};

}  // namespace seb
