// Online state-estimation node for the Unitree Go2 (monitoring only).
//
// Lifecycle:
//   configure  : load parameters, build the URDF model, create the selected
//                estimator (exactly one), the pipeline and the run logger
//   activate   : start the worker threads, subscribe to LowState, publish
//   deactivate : unsubscribe, drain and stop the threads, flush the logs
//   cleanup    : destroy the pipeline
// Services: ~/reset_estimator (std_srvs/Trigger)
//
// SAFETY: this node never publishes commands (no low-level command / actuation interface;
// enforced by test/check_no_actuation.cmake).

#include "seb_online/pipeline.hpp"
#include "seb_online/ros_output.hpp"
#include "seb_online/ros_parameters.hpp"
#include "seb_online/run_logger.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <seb_interfaces/msg/estimator_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <memory>
#include <sstream>

namespace seb {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OnlineEstimatorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit OnlineEstimatorNode(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("seb_online_estimator", options) {
    declare_parameter<bool>("autostart", true);
  }

  ~OnlineEstimatorNode() override { teardown(); }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      settings_ = declareAndLoadSettings(get_node_parameters_interface());
      auto adapter = createEstimator(settings_.pipeline.estimator);
      const std::string estimator_name = adapter->name();
      const std::string estimator_description = adapter->describe();

      if (settings_.log_enabled) {
        const std::string dir = makeRunDirectory(settings_.log_directory, settings_.run_name, estimator_name);
        logger_ = std::make_unique<RunLogger>(dir, static_cast<std::size_t>(settings_.log_queue_capacity));
        std::ostringstream cfg;
        cfg << "# seb_online run configuration\n"
            << "mode: online\n"
            << "estimator_description: \"" << estimator_description << "\"\n"
            << "urdf_path: " << settings_.pipeline.mapping.urdf_path << "\n"
            << "p_base_imu: [" << settings_.pipeline.mapping.p_base_imu.transpose() << "]\n"
            << "R_base_imu_rowmajor: [" << settings_.pipeline.mapping.R_base_imu.row(0) << ", "
            << settings_.pipeline.mapping.R_base_imu.row(1) << ", " << settings_.pipeline.mapping.R_base_imu.row(2)
            << "]\n"
            << "parameters:\n";
        std::istringstream lines(dumpParameters(get_node_parameters_interface()));
        for (std::string l; std::getline(lines, l);) cfg << "  " << l << "\n";
        logger_->writeTextFile("run_config.yaml", cfg.str());
        RCLCPP_INFO(get_logger(), "Logging run to %s", dir.c_str());
      }

      state_pub_ = create_publisher<seb_interfaces::msg::EstimatorState>(settings_.output_topic, rclcpp::QoS(10));
      if (settings_.publish_odometry) {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(settings_.odometry_topic, rclcpp::QoS(10));
      }
      if (settings_.publish_base_state) {
        // Depth 1, as the MPC subscription: only the latest state matters.
        base_state_pub_ =
            create_publisher<dls2_interface::msg::BaseState>(settings_.base_state_topic, rclcpp::QoS(1));
        RCLCPP_INFO(get_logger(), "Publishing dls2_interface/BaseState on %s", settings_.base_state_topic.c_str());
      }
      diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
      if (settings_.publish_timing) {
        // Depth for rosbag recording at the sample rate (500 Hz).
        timing_pub_ = create_publisher<seb_interfaces::msg::EstimatorTiming>(settings_.timing_topic, rclcpp::QoS(500));
        RCLCPP_INFO(get_logger(), "Publishing estimator timing on %s", settings_.timing_topic.c_str());
      }

      Pipeline::Sinks sinks;
      sinks.output = [this](StateEstimate& s) { publish(s); };
      sinks.record = [this](const StateEstimate& s) {
        diagnostics_.add(s);
        if (logger_) logger_->logSample(s);
      };
      sinks.event = [this](const PipelineEvent& e) {
        if (logger_) logger_->logEvent(e);
      };
      sinks.status = [this](const std::string& m) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", m.c_str());
      };
      pipeline_ = std::make_unique<Pipeline>(settings_.pipeline, std::move(adapter), sinks);

      const auto& m = settings_.pipeline.mapping;
      RCLCPP_INFO(get_logger(),
                  "Configured: estimator=%s, topic=%s, stamp_source=%s, URDF=%s, p_base_imu=[%.4f %.4f %.4f]. "
                  "MONITORING ONLY: no actuation interface.",
                  estimator_name.c_str(), settings_.lowstate_topic.c_str(),
                  settings_.pipeline.acquisition.stamp_source == StampSource::kReceiveTime ? "receive_time"
                                                                                           : "lowstate_tick",
                  m.urdf_path.c_str(), m.p_base_imu.x(), m.p_base_imu.y(), m.p_base_imu.z());
      RCLCPP_INFO(get_logger(), "%s", estimator_description.c_str());
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
      teardown();
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    if (logger_) logger_->start();
    pipeline_->start();
    state_pub_->on_activate();
    if (odom_pub_) odom_pub_->on_activate();
    if (base_state_pub_) base_state_pub_->on_activate();
    diag_pub_->on_activate();
    if (timing_pub_) timing_pub_->on_activate();

    acquisition_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(settings_.lowstate_qos_depth)));
    if (settings_.lowstate_qos_reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = acquisition_group_;
    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
        settings_.lowstate_topic, qos,
        [this](const unitree_go::msg::LowState::ConstSharedPtr& msg, const rclcpp::MessageInfo& info) {
          onLowState(*msg, info);
        },
        sub_options);

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_estimator",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          pipeline_->requestEstimatorReset();
          res->success = true;
          res->message = "estimator will be re-initialised on the next sample";
        },
        rclcpp::ServicesQoS().get_rmw_qos_profile(), service_group_);

    last_diag_steady_ns_ = steadyNowNs();
    diag_timer_ = create_wall_timer(
        std::chrono::duration<double>(settings_.diagnostics_period_s), [this] { publishDiagnostics(); },
        service_group_);
    RCLCPP_INFO(get_logger(), "Active: waiting for %s", settings_.lowstate_topic.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    stopProcessing();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    teardown();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    teardown();
    return CallbackReturn::SUCCESS;
  }

  // Stops subscriptions and threads and flushes the logs (idempotent).
  void stopProcessing() {
    lowstate_sub_.reset();
    diag_timer_.reset();
    reset_srv_.reset();
    if (pipeline_) {
      pipeline_->stop();
      const auto c = pipeline_->counters();
      RCLCPP_INFO(get_logger(), "Stopped: received=%lu dispatched=%lu processed=%lu lost=%lu",
                  static_cast<unsigned long>(c.received), static_cast<unsigned long>(c.dispatched),
                  static_cast<unsigned long>(c.processed), static_cast<unsigned long>(c.droppedBeforeEstimator()));
    }
    if (logger_) {
      logger_->stop();
      if (logger_->droppedRecords() > 0) {
        RCLCPP_WARN(get_logger(), "%lu log records were dropped (log queue full)",
                    static_cast<unsigned long>(logger_->droppedRecords()));
      }
    }
    if (state_pub_ && state_pub_->is_activated()) state_pub_->on_deactivate();
    if (odom_pub_ && odom_pub_->is_activated()) odom_pub_->on_deactivate();
    if (base_state_pub_ && base_state_pub_->is_activated()) base_state_pub_->on_deactivate();
    if (diag_pub_ && diag_pub_->is_activated()) diag_pub_->on_deactivate();
    if (timing_pub_ && timing_pub_->is_activated()) timing_pub_->on_deactivate();
  }

  void teardown() {
    stopProcessing();
    pipeline_.reset();
    logger_.reset();
    state_pub_.reset();
    odom_pub_.reset();
    base_state_pub_.reset();
    diag_pub_.reset();
    timing_pub_.reset();
  }

 private:
  void onLowState(const unitree_go::msg::LowState& msg, const rclcpp::MessageInfo& info) {
    // Reception times first, before any processing.
    RawLowState raw;
    raw.receive_steady_ns = steadyNowNs();
    raw.receive_ros_ns = get_clock()->now().nanoseconds();
    raw.dds_source_ns = info.get_rmw_message_info().source_timestamp;
    fillRawLowState(msg, raw);
    pipeline_->onLowState(raw);
  }

  // Called from the estimator thread, right after the update.
  void publish(StateEstimate& s) {
    s.publish_steady_ns = steadyNowNs();
    s.publish_ros_ns = get_clock()->now().nanoseconds();
    toMsg(s, settings_.world_frame, settings_.base_frame, state_msg_);
    state_pub_->publish(state_msg_);
    if (odom_pub_) {
      toOdometry(s, settings_.world_frame, settings_.base_frame, odom_msg_);
      odom_pub_->publish(odom_msg_);
    }
    // The controller only receives estimates it can use (no warm-up, no diverged state).
    if (base_state_pub_ && usableForControl(s)) {
      toBaseState(s, settings_.world_frame, settings_.robot_name, base_state_msg_);
      base_state_pub_->publish(base_state_msg_);
    }
    // After the state: the timing message never delays the controller input.
    if (timing_pub_) {
      toTiming(s, timing_msg_);
      timing_pub_->publish(timing_msg_);
    }
  }

  void publishDiagnostics() {
    if (!pipeline_) return;
    const std::int64_t now = steadyNowNs();
    const double period = static_cast<double>(now - last_diag_steady_ns_) * 1e-9;
    last_diag_steady_ns_ = now;
    std::string warning;
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = get_clock()->now();
    arr.status.push_back(diagnostics_.makeStatus(std::string(get_name()) + ": pipeline",
                                                 pipeline_->estimator().name(), pipeline_->counters(),
                                                 pipeline_->syncStats(), period, &warning));
    diag_pub_->publish(arr);
    if (!warning.empty()) RCLCPP_WARN(get_logger(), "%s", warning.c_str());
  }

  NodeSettings settings_;
  std::unique_ptr<RunLogger> logger_;
  std::unique_ptr<Pipeline> pipeline_;
  DiagnosticsAccumulator diagnostics_;

  rclcpp::CallbackGroup::SharedPtr acquisition_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp_lifecycle::LifecyclePublisher<seb_interfaces::msg::EstimatorState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<dls2_interface::msg::BaseState>::SharedPtr base_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp_lifecycle::LifecyclePublisher<seb_interfaces::msg::EstimatorTiming>::SharedPtr timing_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::int64_t last_diag_steady_ns_ = 0;

  // Only used from the estimator thread.
  seb_interfaces::msg::EstimatorState state_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  dls2_interface::msg::BaseState base_state_msg_;
  seb_interfaces::msg::EstimatorTiming timing_msg_;
};

}  // namespace seb

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<seb::OnlineEstimatorNode>(rclcpp::NodeOptions());

  // Subscription and service/timer callbacks are in separate callback groups.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());

  if (node->get_parameter("autostart").as_bool()) {
    const auto& configured = node->configure();
    if (configured.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      node->activate();
    } else {
      RCLCPP_ERROR(node->get_logger(), "Configuration failed; node stays unconfigured.");
    }
  }

  executor.spin();
  // Flush logs and join threads even if the context was shut down by a signal.
  node->teardown();
  rclcpp::shutdown();
  return 0;
}
