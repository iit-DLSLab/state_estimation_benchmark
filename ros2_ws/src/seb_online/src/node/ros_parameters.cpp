#include "seb_online/ros_parameters.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter_value.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace seb {

namespace {

using Params = std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>;

template <typename T>
T param(const Params& p, const std::string& name, const T& def, const std::string& description) {
  if (!p->has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.name = name;
    d.description = description;
    // Parameters are applied when the pipeline is configured; runtime changes would be ignored.
    d.read_only = true;
    p->declare_parameter(name, rclcpp::ParameterValue(def), d);
  }
  return p->get_parameter(name).get_value<T>();
}

template <std::size_t N>
std::array<int, N> intArray(const Params& p, const std::string& name, const std::array<int, N>& def,
                            const std::string& description) {
  std::vector<std::int64_t> d(def.begin(), def.end());
  const auto v = param<std::vector<std::int64_t>>(p, name, d, description);
  if (v.size() != N) throw std::invalid_argument(name + " must have " + std::to_string(N) + " entries");
  std::array<int, N> out{};
  for (std::size_t i = 0; i < N; ++i) out[i] = static_cast<int>(v[i]);
  return out;
}

template <std::size_t N>
std::array<std::string, N> stringArray(const Params& p, const std::string& name,
                                       const std::array<std::string, N>& def, const std::string& description) {
  std::vector<std::string> d(def.begin(), def.end());
  const auto v = param<std::vector<std::string>>(p, name, d, description);
  if (v.size() != N) throw std::invalid_argument(name + " must have " + std::to_string(N) + " entries");
  std::array<std::string, N> out;
  for (std::size_t i = 0; i < N; ++i) out[i] = v[i];
  return out;
}

Eigen::Vector3d vec3(const Params& p, const std::string& name, const std::string& description) {
  const auto v = param<std::vector<double>>(p, name, {0.0, 0.0, 0.0}, description);
  if (v.size() != 3) throw std::invalid_argument(name + " must have 3 entries");
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

ThreadSettings threadSettings(const Params& p, const std::string& prefix) {
  ThreadSettings t;
  t.cpu = static_cast<int>(param<std::int64_t>(p, prefix + ".cpu", -1, "CPU to pin the thread to (-1: none)"));
  t.rt_priority = static_cast<int>(
      param<std::int64_t>(p, prefix + ".rt_priority", 0, "SCHED_FIFO priority 1..99 (0: default scheduler)"));
  return t;
}

InvariantCovarianceSettings covariances(const Params& p, const std::string& prefix) {
  InvariantCovarianceSettings c;
  const std::string d = "log10 of the covariance diagonal";
  c.gyro_exp = param(p, prefix + ".gyro", c.gyro_exp, d);
  c.acc_exp = param(p, prefix + ".acc", c.acc_exp, d);
  c.slip_exp = param(p, prefix + ".slip", c.slip_exp, d);
  c.contact_exp = param(p, prefix + ".contact", c.contact_exp, d);
  c.encoder_exp = param(p, prefix + ".encoder", c.encoder_exp, d);
  c.bias_gyro_exp = param(p, prefix + ".bias_gyro", c.bias_gyro_exp, d);
  c.bias_acc_exp = param(p, prefix + ".bias_acc", c.bias_acc_exp, d);
  c.prior_orientation_exp = param(p, prefix + ".prior_orientation", c.prior_orientation_exp, d);
  c.prior_velocity_exp = param(p, prefix + ".prior_velocity", c.prior_velocity_exp, d);
  c.prior_position_exp = param(p, prefix + ".prior_position", c.prior_position_exp, d);
  c.prior_bias_gyro_exp = param(p, prefix + ".prior_bias_gyro", c.prior_bias_gyro_exp, d);
  c.prior_bias_acc_exp = param(p, prefix + ".prior_bias_acc", c.prior_bias_acc_exp, d);
  return c;
}

std::string expandHome(const std::string& path) {
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "") + path.substr(1);
  }
  return path;
}

}  // namespace

NodeSettings declareAndLoadSettings(const Params& p) {
  NodeSettings s;
  PipelineConfig& pc = s.pipeline;

  // ---------------- robot / mapping ----------------
  const std::string type = param<std::string>(p, "robot.type", "go2", "Robot type (only go2 is supported)");
  if (type != "go2") throw std::invalid_argument("robot.type: only 'go2' is supported, got '" + type + "'");
  RobotMapping m = RobotMapping::go2Defaults();
  m.robot_type = type;
  m.urdf_path = param<std::string>(p, "robot.urdf_path", "", "URDF file; empty: <seb_online share>/models/go2.urdf");
  if (m.urdf_path.empty()) {
    m.urdf_path = ament_index_cpp::get_package_share_directory("seb_online") + "/models/go2.urdf";
  }
  m.urdf_path = expandHome(m.urdf_path);
  m.base_frame = param<std::string>(p, "robot.base_frame", m.base_frame, "URDF base frame");
  m.imu_frame = param<std::string>(p, "robot.imu_frame", m.imu_frame,
                                   "URDF IMU frame; empty: use robot.imu_xyz / robot.imu_rpy");
  const Eigen::Vector3d imu_xyz = vec3(p, "robot.imu_xyz", "IMU position in base [m] (only if imu_frame is empty)");
  const Eigen::Vector3d imu_rpy =
      vec3(p, "robot.imu_rpy", "IMU orientation in base, roll pitch yaw [rad] (only if imu_frame is empty)");
  m.lowstate_motor_index = intArray<12>(p, "robot.lowstate_motor_index", m.lowstate_motor_index,
                                        "LowState.motor_state index of each canonical joint (LF,RF,LH,RH x HAA,HFE,KFE)");
  m.lowstate_foot_force_index = intArray<4>(p, "robot.lowstate_foot_force_index", m.lowstate_foot_force_index,
                                            "LowState.foot_force index of each canonical leg (LF,RF,LH,RH)");
  m.urdf_joint_names = stringArray<12>(p, "robot.urdf_joint_names", m.urdf_joint_names,
                                       "URDF joint names in canonical order");
  m.urdf_foot_frames = stringArray<4>(p, "robot.urdf_foot_frames", m.urdf_foot_frames,
                                      "URDF foot frames in canonical order (LF,RF,LH,RH)");
  if (m.imu_frame.empty()) {
    m.p_base_imu = imu_xyz;
    m.R_base_imu = (Eigen::AngleAxisd(imu_rpy.z(), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(imu_rpy.y(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(imu_rpy.x(), Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
  } else {
    resolveImuFromUrdf(m);
  }
  m.validate();
  pc.mapping = m;

  // ---------------- input ----------------
  s.lowstate_topic = param<std::string>(p, "input.lowstate_topic", s.lowstate_topic, "unitree_go/LowState topic");
  s.lowstate_qos_depth = static_cast<int>(param<std::int64_t>(p, "input.qos_depth", 10, "Subscription queue depth"));
  s.lowstate_qos_reliable =
      param<bool>(p, "input.qos_reliable", false, "true: RELIABLE subscription; false: BEST_EFFORT (sensor data)");

  // ---------------- time base / acquisition ----------------
  const std::string stamp_source = param<std::string>(
      p, "time.stamp_source", "receive_time",
      "Estimator time base: receive_time (ROS clock at reception) or lowstate_tick (LowState.tick)");
  if (stamp_source == "receive_time") {
    pc.acquisition.stamp_source = StampSource::kReceiveTime;
  } else if (stamp_source == "lowstate_tick") {
    pc.acquisition.stamp_source = StampSource::kLowStateTick;
  } else {
    throw std::invalid_argument("time.stamp_source must be receive_time or lowstate_tick");
  }
  pc.acquisition.tick_period_ns = static_cast<std::int64_t>(
      std::llround(param<double>(p, "time.tick_period_s", 0.001, "Duration of one LowState.tick count [s]") * 1e9));
  pc.acquisition.time_jump_reset_ns = static_cast<std::int64_t>(std::llround(
      param<double>(p, "time.time_jump_reset_s", 1.0, "Backward time jump that re-initialises the pipeline [s]") *
      1e9));
  pc.acquisition.drop_duplicates =
      param<bool>(p, "acquisition.drop_duplicates", true, "Drop messages identical to the previous one (same tick)");
  pc.acquisition.decimation = static_cast<int>(
      param<std::int64_t>(p, "acquisition.decimation", 1, "Process one sample every N (skips are logged)"));
  if (pc.acquisition.decimation < 1) throw std::invalid_argument("acquisition.decimation must be >= 1");

  // ---------------- contact detection ----------------
  pc.contact.threshold = param(p, "contact.threshold", pc.contact.threshold, "Foot-force threshold (no hysteresis)");
  pc.contact.use_hysteresis = param(p, "contact.use_hysteresis", pc.contact.use_hysteresis, "Enable hysteresis");
  pc.contact.threshold_low = param(p, "contact.threshold_low", pc.contact.threshold_low, "Stance -> swing threshold");
  pc.contact.threshold_high =
      param(p, "contact.threshold_high", pc.contact.threshold_high, "Swing -> stance threshold");
  pc.contact.valid_min = param(p, "contact.valid_min", pc.contact.valid_min, "Minimum plausible foot-force reading");
  pc.contact.valid_max = param(p, "contact.valid_max", pc.contact.valid_max, "Maximum plausible foot-force reading");

  // ---------------- synchronisation / queues ----------------
  pc.sync.capacity = static_cast<std::size_t>(
      param<std::int64_t>(p, "sync.capacity", 64, "Maximum samples waiting for worker results"));
  pc.sync.worker_timeout_ns = static_cast<std::int64_t>(std::llround(
      param<double>(p, "sync.worker_timeout_s", 0.02, "Maximum dispatch-to-result time before a sample is expired") *
      1e9));
  pc.worker_queue_capacity = static_cast<std::size_t>(
      param<std::int64_t>(p, "queues.worker_capacity", 64, "Input queue capacity of each worker"));
  pc.sync.output_capacity = static_cast<std::size_t>(
      param<std::int64_t>(p, "queues.estimator_capacity", 50, "Estimator input queue capacity"));
  const std::string policy = param<std::string>(p, "queues.estimator_overflow_policy", "drop_oldest",
                                                "drop_oldest | reject_newest (dropped samples are logged)");
  if (policy == "drop_oldest") {
    pc.sync.output_overflow = OverflowPolicy::kDropOldest;
  } else if (policy == "reject_newest") {
    pc.sync.output_overflow = OverflowPolicy::kRejectNewest;
  } else {
    throw std::invalid_argument("queues.estimator_overflow_policy must be drop_oldest or reject_newest");
  }
  if (pc.sync.capacity == 0 || pc.worker_queue_capacity == 0 || pc.sync.output_capacity == 0) {
    throw std::invalid_argument("queue capacities must be > 0");
  }

  // ---------------- estimator ----------------
  EstimatorSettings& es = pc.estimator;
  es.type = param<std::string>(p, "estimator.type", "muse", "muse | iekf | invariant_smoother (exactly one runs)");
  const std::string init = param<std::string>(p, "estimator.init_attitude", "gravity_aligned",
                                              "gravity_aligned | identity");
  if (init == "gravity_aligned") {
    es.common.init_attitude = InitAttitude::kGravityAligned;
  } else if (init == "identity") {
    es.common.init_attitude = InitAttitude::kIdentity;
  } else {
    throw std::invalid_argument("estimator.init_attitude must be gravity_aligned or identity");
  }
  const std::string dt_mode =
      param<std::string>(p, "estimator.dt_mode", "measured", "measured | offline_compat (regression only)");
  if (dt_mode == "measured") {
    es.common.dt_mode = DtMode::kMeasured;
  } else if (dt_mode == "offline_compat") {
    es.common.dt_mode = DtMode::kOfflineCompat;
  } else {
    throw std::invalid_argument("estimator.dt_mode must be measured or offline_compat");
  }
  es.common.nominal_dt_s =
      param(p, "estimator.nominal_dt_s", es.common.nominal_dt_s, "dt passed to Initialize() (first step) [s]");
  EstimatorWorkerConfig& ew = pc.estimator_worker;
  ew.max_gap_s = param(p, "estimator.max_gap_s", ew.max_gap_s, "Larger dt re-initialises the estimator [s]");
  ew.max_input_age_s = param(p, "estimator.max_input_age_s", ew.max_input_age_s,
                             "Older inputs are dropped as stale [s]; <= 0 disables");
  ew.min_dt_s = param(p, "estimator.min_dt_s", ew.min_dt_s, "Smaller dt is not processed [s]");
  ew.max_speed_mps = param(p, "estimator.max_speed_mps", ew.max_speed_mps, "Divergence check on base speed [m/s]");
  ew.hold_without_stance = param<bool>(p, "estimator.hold_without_stance", ew.hold_without_stance,
                                       "No leg in stance: force zero velocity and hold position (IEKF, smoother)");
  ew.hold_restart_s = param(p, "estimator.hold_restart_s", ew.hold_restart_s,
                            "Stance after a hold this long: restart at the held pose [s] (< 0: never)");
  ew.warmup_samples = static_cast<int>(param<std::int64_t>(p, "estimator.warmup_samples", ew.warmup_samples,
                                                           "Samples flagged INITIALIZING after (re)initialisation"));

  MuseSettings& mu = es.muse;
  mu.ki = param(p, "muse.ki", mu.ki, "Attitude observer integral gain");
  mu.kp = param(p, "muse.kp", mu.kp, "Attitude observer proportional gain");
  mu.attitude_P0 = param(p, "muse.attitude_P0", mu.attitude_P0, "Attitude XKF initial covariance (diagonal)");
  mu.attitude_Q = param(p, "muse.attitude_Q", mu.attitude_Q, "Attitude XKF process noise (diagonal)");
  mu.attitude_R = param(p, "muse.attitude_R", mu.attitude_R, "Attitude XKF measurement noise (diagonal)");
  mu.fusion_P0 = param(p, "muse.fusion_P0", mu.fusion_P0, "Sensor-fusion KF initial covariance (diagonal)");
  mu.fusion_Q = param(p, "muse.fusion_Q", mu.fusion_Q, "Sensor-fusion KF process noise (diagonal)");
  mu.fusion_R = param(p, "muse.fusion_R", mu.fusion_R, "Sensor-fusion KF measurement noise (diagonal)");
  mu.gravity = param(p, "muse.gravity", mu.gravity, "Gravity magnitude [m/s^2]");
  mu.use_updated_attitude = param(p, "muse.use_updated_attitude", mu.use_updated_attitude,
                                  "true: attitude of the current sample; false: offline one-step-lagged attitude");

  IekfSettings& ie = es.iekf;
  ie.cov = covariances(p, "iekf.cov_exp");
  ie.starting_point = static_cast<int>(param<std::int64_t>(p, "iekf.starting_point", ie.starting_point, "NUM_OF_TRASH_DATA"));
  ie.slip_rejection = param(p, "iekf.slip_rejection", ie.slip_rejection, "Slip rejection mode");
  ie.slip_threshold = param(p, "iekf.slip_threshold", ie.slip_threshold, "Slip threshold [m/s]");
  ie.variable_contact_cov = param(p, "iekf.variable_contact_cov", ie.variable_contact_cov, "Variable contact covariance");
  ie.cov_amplifier = param(p, "iekf.cov_amplifier", ie.cov_amplifier, "Covariance amplifier");
  ie.max_iterations = static_cast<int>(param<std::int64_t>(p, "iekf.max_iterations", ie.max_iterations, "Max_Iteration"));
  ie.convergence = param(p, "iekf.convergence", ie.convergence, "Optimization_Epsilon");

  SmootherSettings& sm = es.smoother;
  sm.cov = covariances(p, "smoother.cov_exp");
  sm.window_size = static_cast<int>(
      param<std::int64_t>(p, "smoother.window_size", sm.window_size, "Fixed-lag window size (propagation factors)"));
  if (sm.window_size < 1) throw std::invalid_argument("smoother.window_size must be >= 1");
  sm.starting_point = static_cast<int>(param<std::int64_t>(p, "smoother.starting_point", sm.starting_point, "NUM_OF_TRASH_DATA"));
  sm.slip_rejection = param(p, "smoother.slip_rejection", sm.slip_rejection, "Slip rejection mode");
  sm.slip_threshold = param(p, "smoother.slip_threshold", sm.slip_threshold, "Slip threshold [m/s]");
  sm.variable_contact_cov = param(p, "smoother.variable_contact_cov", sm.variable_contact_cov, "Variable contact covariance");
  sm.cov_amplifier = param(p, "smoother.cov_amplifier", sm.cov_amplifier, "Covariance amplifier");
  sm.max_iterations = static_cast<int>(param<std::int64_t>(p, "smoother.max_iterations", sm.max_iterations, "Max_Iteration"));
  sm.max_backpropagate = static_cast<int>(
      param<std::int64_t>(p, "smoother.max_backpropagate", sm.max_backpropagate, "Max_backpropagate_num"));
  sm.backpropagate_rate = param(p, "smoother.backpropagate_rate", sm.backpropagate_rate, "backppgn_rate");
  sm.convergence = param(p, "smoother.convergence", sm.convergence, "Optimization_Epsilon");
  sm.retract_all = param(p, "smoother.retract_all", sm.retract_all, "Retract_All_flag");
  sm.per_interval_dt = param(p, "smoother.per_interval_dt", sm.per_interval_dt,
                             "Each propagation factor uses its own measured interval");

  // ---------------- threads ----------------
  pc.kinematics_thread = threadSettings(p, "threads.kinematics");
  pc.contact_thread = threadSettings(p, "threads.contact");
  pc.estimator_worker.thread = threadSettings(p, "threads.estimator");

  // ---------------- output / diagnostics / logging ----------------
  s.output_topic = param<std::string>(p, "output.topic", s.output_topic, "seb_interfaces/EstimatorState topic");
  s.world_frame = param<std::string>(p, "output.world_frame", s.world_frame, "World frame id");
  s.base_frame = param<std::string>(p, "output.base_frame", s.base_frame, "Base frame id");
  s.publish_odometry = param<bool>(p, "output.publish_odometry", s.publish_odometry,
                                   "Also publish nav_msgs/Odometry (covariances not provided)");
  s.odometry_topic = param<std::string>(p, "output.odometry_topic", s.odometry_topic, "nav_msgs/Odometry topic");
  s.publish_base_state = param<bool>(p, "output.publish_base_state", s.publish_base_state,
                                     "Also publish dls2_interface/BaseState (the state read by the MPC)");
  s.base_state_topic =
      param<std::string>(p, "output.base_state_topic", s.base_state_topic, "dls2_interface/BaseState topic");
  s.robot_name = param<std::string>(p, "output.robot_name", s.robot_name, "BaseState.robot_name");
  s.publish_timing = param<bool>(p, "profiling.publish_timing", s.publish_timing,
                                 "Publish seb_interfaces/EstimatorTiming for every estimator update");
  s.timing_topic = param<std::string>(p, "profiling.timing_topic", s.timing_topic, "EstimatorTiming topic");
  s.diagnostics_period_s =
      param<double>(p, "diagnostics.period_s", s.diagnostics_period_s, "Period of /diagnostics [s]");
  s.log_enabled = param<bool>(p, "log.enabled", s.log_enabled, "Write per-sample CSV logs");
  s.log_directory = param<std::string>(p, "log.directory", s.log_directory, "Root directory of the run logs");
  s.run_name = param<std::string>(p, "log.run_name", s.run_name, "Run sub-directory; empty: <date>_<estimator>");
  s.log_queue_capacity =
      static_cast<int>(param<std::int64_t>(p, "log.queue_capacity", s.log_queue_capacity, "Log queue capacity"));
  return s;
}

std::string dumpParameters(const Params& p) {
  std::ostringstream ss;
  const auto names = p->list_parameters({}, 0).names;
  std::vector<std::string> sorted(names.begin(), names.end());
  std::sort(sorted.begin(), sorted.end());
  for (const auto& n : sorted) {
    ss << n << ": " << rclcpp::to_string(p->get_parameter(n).get_parameter_value()) << "\n";
  }
  return ss.str();
}

std::string makeRunDirectory(const std::string& directory, const std::string& run_name,
                             const std::string& estimator) {
  std::string name = run_name;
  if (name.empty()) {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << estimator;
    name = ss.str();
  }
  const std::string path = expandHome(directory) + "/" + name;
  std::filesystem::create_directories(path);
  return path;
}

}  // namespace seb
