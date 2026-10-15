#include "seb_online/ros_output.hpp"

#include <algorithm>
#include <sstream>

namespace seb {

namespace {

void setVec(geometry_msgs::msg::Vector3& v, const Eigen::Vector3d& e) {
  v.x = e.x();
  v.y = e.y();
  v.z = e.z();
}

std::string str(double v) {
  std::ostringstream ss;
  ss << v;
  return ss.str();
}

diagnostic_msgs::msg::KeyValue kv(const std::string& k, const std::string& v) {
  diagnostic_msgs::msg::KeyValue x;
  x.key = k;
  x.value = v;
  return x;
}

}  // namespace

void toMsg(const StateEstimate& s, const std::string& world_frame, const std::string& base_frame,
           seb_interfaces::msg::EstimatorState& msg) {
  // The stamp is the time of the state (source sample), never the publication time.
  msg.header.stamp = rclcpp::Time(s.stamp_ns, RCL_ROS_TIME);
  msg.header.frame_id = world_frame;
  msg.child_frame_id = base_frame;
  msg.estimator = s.estimator;
  msg.sample_id = s.sample_id;
  msg.lowstate_tick = s.tick;
  msg.stamp_source = s.stamp_source == StampSource::kReceiveTime ? msg.STAMP_RECEIVE_TIME : msg.STAMP_LOWSTATE_TICK;
  msg.status = static_cast<std::uint8_t>(s.status);
  msg.orientation.w = s.q_wb.w();
  msg.orientation.x = s.q_wb.x();
  msg.orientation.y = s.q_wb.y();
  msg.orientation.z = s.q_wb.z();
  msg.position.x = s.p_wb.x();
  msg.position.y = s.p_wb.y();
  msg.position.z = s.p_wb.z();
  msg.position_valid = s.position_valid;
  setVec(msg.linear_velocity_world, s.v_wb);
  setVec(msg.linear_velocity_base, s.v_b);
  setVec(msg.angular_velocity_base, s.omega_b);
  msg.angular_velocity_source =
      s.omega_bias_corrected ? msg.ANGULAR_VELOCITY_IMU_BIAS_CORRECTED : msg.ANGULAR_VELOCITY_IMU_RAW;
  setVec(msg.gyro_bias, s.gyro_bias);
  setVec(msg.accel_bias, s.acc_bias);
  msg.gyro_bias_valid = s.gyro_bias_valid;
  msg.accel_bias_valid = s.acc_bias_valid;
  for (int i = 0; i < kNumLegs; ++i) msg.contact[i] = s.contact[i];
  msg.latency_to_estimate_s = static_cast<double>(s.est_end_steady_ns - s.receive_steady_ns) * 1e-9;
  msg.estimator_compute_time_s = static_cast<double>(s.est_end_steady_ns - s.est_start_steady_ns) * 1e-9;
  msg.state_age_at_publish_s =
      s.publish_ros_ns > 0 ? static_cast<double>(s.publish_ros_ns - s.stamp_ns) * 1e-9 : 0.0;
  msg.estimator_queue_depth = static_cast<std::uint32_t>(s.estimator_queue_depth);
  msg.dropped_samples_total = s.dropped_total;
}

void toOdometry(const StateEstimate& s, const std::string& world_frame, const std::string& base_frame,
                nav_msgs::msg::Odometry& msg) {
  msg.header.stamp = rclcpp::Time(s.stamp_ns, RCL_ROS_TIME);
  msg.header.frame_id = world_frame;
  msg.child_frame_id = base_frame;
  msg.pose.pose.position.x = s.p_wb.x();
  msg.pose.pose.position.y = s.p_wb.y();
  msg.pose.pose.position.z = s.p_wb.z();
  msg.pose.pose.orientation.w = s.q_wb.w();
  msg.pose.pose.orientation.x = s.q_wb.x();
  msg.pose.pose.orientation.y = s.q_wb.y();
  msg.pose.pose.orientation.z = s.q_wb.z();
  setVec(msg.twist.twist.linear, s.v_b);
  setVec(msg.twist.twist.angular, s.omega_b);
}

void DiagnosticsAccumulator::add(const StateEstimate& s) {
  const double exec = static_cast<double>(s.est_end_steady_ns - s.est_start_steady_ns) * 1e-9;
  const double lat = static_cast<double>(s.est_end_steady_ns - s.receive_steady_ns) * 1e-9;
  std::lock_guard<std::mutex> lock(mutex_);
  ++n_;
  exec_sum_ += exec;
  exec_max_ = std::max(exec_max_, exec);
  latency_sum_ += lat;
  latency_max_ = std::max(latency_max_, lat);
  queue_max_ = std::max(queue_max_, s.estimator_queue_depth);
  if (s.status != EstimateStatus::kValid) ++not_valid_;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsAccumulator::makeStatus(const std::string& name,
                                                                          const std::string& estimator,
                                                                          const PipelineCounters& c,
                                                                          const SynchronizerStats& sync,
                                                                          double period_s, std::string* warning) {
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = name;
  st.hardware_id = "unitree_go2";

  const std::uint64_t dropped_total = c.droppedBeforeEstimator();
  const std::uint64_t dropped_period = dropped_total - last_counters_.droppedBeforeEstimator();
  const std::uint64_t received_period = c.received - last_counters_.received;
  const double rate = period_s > 0 ? static_cast<double>(n_) / period_s : 0.0;

  st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::ostringstream msg;
  msg << estimator << ": " << n_ << " updates (" << rate << " Hz)";
  if (received_period == 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg << ", no LowState received";
  }
  if (dropped_period > 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg << ", " << dropped_period << " samples lost (falling behind or missing results)";
  }
  if (sync.identity_mismatches > 0 || sync.unknown_results > 0 || sync.duplicate_results > 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg << ", synchronisation integrity error";
  }
  st.message = msg.str();
  if (warning && st.level != diagnostic_msgs::msg::DiagnosticStatus::OK) *warning = st.message;

  st.values.push_back(kv("estimator", estimator));
  st.values.push_back(kv("updates_per_s", str(rate)));
  st.values.push_back(kv("estimator_exec_mean_s", str(n_ ? exec_sum_ / n_ : 0.0)));
  st.values.push_back(kv("estimator_exec_max_s", str(exec_max_)));
  st.values.push_back(kv("latency_to_estimate_mean_s", str(n_ ? latency_sum_ / n_ : 0.0)));
  st.values.push_back(kv("latency_to_estimate_max_s", str(latency_max_)));
  st.values.push_back(kv("estimator_queue_max", std::to_string(queue_max_)));
  st.values.push_back(kv("estimates_not_valid", std::to_string(not_valid_)));
  st.values.push_back(kv("received_total", std::to_string(c.received)));
  st.values.push_back(kv("dispatched_total", std::to_string(c.dispatched)));
  st.values.push_back(kv("processed_total", std::to_string(c.processed)));
  st.values.push_back(kv("lost_total", std::to_string(dropped_total)));
  for (std::size_t i = 0; i < c.events.size(); ++i) {
    if (c.events[i] > 0) {
      st.values.push_back(kv(std::string("events.") + toString(static_cast<EventType>(i)), std::to_string(c.events[i])));
    }
  }
  st.values.push_back(kv("sync.pending", std::to_string(sync.pending)));
  st.values.push_back(kv("sync.max_pending", std::to_string(sync.max_pending)));
  st.values.push_back(kv("sync.max_output_depth", std::to_string(sync.max_output_depth)));
  st.values.push_back(kv("sync.late_results", std::to_string(sync.late_results)));
  st.values.push_back(kv("sync.identity_mismatches", std::to_string(sync.identity_mismatches)));

  n_ = 0;
  exec_sum_ = exec_max_ = latency_sum_ = latency_max_ = 0.0;
  queue_max_ = 0;
  not_valid_ = 0;
  last_counters_ = c;
  return st;
}

}  // namespace seb
