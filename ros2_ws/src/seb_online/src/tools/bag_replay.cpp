// Deterministic offline replay of a recorded Go2 LowState bag through the SAME
// online pipeline (acquisition, kinematics/contact worker threads, exact
// synchronizer, estimator worker) and the SAME parameters as the live node.
//
// Differences with the live node, by design:
//   * input is read from the bag file, not from DDS: nothing is published on
//     /lowstate, so a replay can never feed data to a robot controller;
//   * receive_ros_ns of every sample is the RECORDED reception time stored in
//     the bag (so the estimator time base matches the recording);
//   * receive_steady_ns is the injection time in this process (latencies measure
//     this machine's processing, not the original transport).
// Modes:
//   replay.mode = lossless : the reader waits for the pipeline (bounded in-flight
//                            samples), no sample is lost because of compute time;
//                            use it to profile execution time, e.g. per window size.
//   replay.mode = realtime : messages are injected at the recorded pace
//                            (scaled by replay.rate); overload behaves as on the
//                            robot (backlog, drop policy).

#include "seb_online/pipeline.hpp"
#include "seb_online/ros_output.hpp"
#include "seb_online/ros_parameters.hpp"
#include "seb_online/run_logger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <seb_interfaces/msg/estimator_state.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("seb_replay");
  int rc = 0;
  try {
    auto params = node->get_node_parameters_interface();
    const std::string bag = node->declare_parameter<std::string>("replay.bag_path", "");
    const std::string topic = node->declare_parameter<std::string>("replay.topic", "/lowstate");
    const std::string mode = node->declare_parameter<std::string>("replay.mode", "lossless");
    const double rate = node->declare_parameter<double>("replay.rate", 1.0);
    const std::int64_t max_messages = node->declare_parameter<std::int64_t>("replay.max_messages", 0);
    const bool publish = node->declare_parameter<bool>("replay.publish", false);
    const std::int64_t max_in_flight = node->declare_parameter<std::int64_t>("replay.max_in_flight", 1);
    if (bag.empty()) throw std::invalid_argument("replay.bag_path is required");
    if (mode != "lossless" && mode != "realtime") throw std::invalid_argument("replay.mode: lossless | realtime");
    if (rate <= 0.0) throw std::invalid_argument("replay.rate must be > 0");

    seb::NodeSettings settings = seb::declareAndLoadSettings(params);
    if (mode == "lossless") {
      // Inputs are injected as fast as the estimator consumes them; their age
      // since injection is irrelevant, stale-dropping would only discard data.
      settings.pipeline.estimator_worker.max_input_age_s = 0.0;
    }
    auto adapter = seb::createEstimator(settings.pipeline.estimator);
    const std::string est_name = adapter->name();
    const std::string est_desc = adapter->describe();

    std::unique_ptr<seb::RunLogger> logger;
    if (settings.log_enabled) {
      const std::string dir = seb::makeRunDirectory(settings.log_directory, settings.run_name, est_name);
      logger = std::make_unique<seb::RunLogger>(dir, static_cast<std::size_t>(settings.log_queue_capacity));
      std::ostringstream cfg;
      cfg << "# seb_online run configuration\n"
          << "mode: replay_" << mode << "\n"
          << "bag: " << bag << "\n"
          << "estimator_description: \"" << est_desc << "\"\n"
          << "urdf_path: " << settings.pipeline.mapping.urdf_path << "\n"
          << "p_base_imu: [" << settings.pipeline.mapping.p_base_imu.transpose() << "]\n"
          << "parameters:\n";
      std::istringstream lines(seb::dumpParameters(params));
      for (std::string l; std::getline(lines, l);) cfg << "  " << l << "\n";
      logger->writeTextFile("run_config.yaml", cfg.str());
      logger->start();
      std::cout << "Logging run to " << dir << std::endl;
    }

    rclcpp::Publisher<seb_interfaces::msg::EstimatorState>::SharedPtr pub;
    if (publish) pub = node->create_publisher<seb_interfaces::msg::EstimatorState>(settings.output_topic, 10);
    seb_interfaces::msg::EstimatorState msg;

    seb::Pipeline::Sinks sinks;
    sinks.output = [&](seb::StateEstimate& s) {
      s.publish_steady_ns = seb::steadyNowNs();
      if (pub) {
        // In replay the ROS clock is not the recording clock: publication age is
        // reported in the recording time base by mapping steady time since injection.
        s.publish_ros_ns = s.receive_ros_ns + (s.publish_steady_ns - s.receive_steady_ns);
        seb::toMsg(s, settings.world_frame, settings.base_frame, msg);
        pub->publish(msg);
      } else {
        s.publish_ros_ns = s.receive_ros_ns + (s.publish_steady_ns - s.receive_steady_ns);
      }
    };
    sinks.record = [&](const seb::StateEstimate& s) {
      if (logger) logger->logSample(s);
    };
    sinks.event = [&](const seb::PipelineEvent& e) {
      if (logger) logger->logEvent(e);
    };
    sinks.status = [&](const std::string& m) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "%s", m.c_str());
    };

    seb::Pipeline pipeline(settings.pipeline, std::move(adapter), sinks);
    pipeline.start();

    rosbag2_cpp::Reader reader;
    reader.open(bag);
    rclcpp::Serialization<unitree_go::msg::LowState> serialization;
    unitree_go::msg::LowState lowstate;
    seb::RawLowState raw;

    std::int64_t n = 0;
    std::int64_t first_bag_ns = -1;
    const auto start_steady = std::chrono::steady_clock::now();
    const auto wall_start = std::chrono::steady_clock::now();
    while (reader.has_next() && rclcpp::ok()) {
      auto bag_msg = reader.read_next();
      if (bag_msg->topic_name != topic) continue;
      if (max_messages > 0 && n >= max_messages) break;
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      serialization.deserialize_message(&serialized, &lowstate);

      const std::int64_t bag_ns = bag_msg->time_stamp;  // recorded reception time
      if (first_bag_ns < 0) first_bag_ns = bag_ns;
      if (mode == "realtime") {
        const auto target = start_steady + std::chrono::nanoseconds(static_cast<std::int64_t>(
                                               static_cast<double>(bag_ns - first_bag_ns) / rate));
        std::this_thread::sleep_until(target);
      } else {
        if (!pipeline.waitForInFlightBelow(static_cast<std::uint64_t>(std::max<std::int64_t>(1, max_in_flight)),
                                           std::chrono::seconds(10))) {
          throw std::runtime_error("pipeline did not make progress for 10 s");
        }
      }
      raw = seb::RawLowState();
      raw.receive_ros_ns = bag_ns;
      raw.receive_steady_ns = seb::steadyNowNs();
      raw.dds_source_ns = 0;
      seb::fillRawLowState(lowstate, raw);
      pipeline.onLowState(raw);
      ++n;
    }
    pipeline.drain(std::chrono::seconds(30));
    pipeline.stop();
    if (logger) logger->stop();
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();

    const auto c = pipeline.counters();
    const auto st = pipeline.syncStats();
    std::cout << "Replay finished (" << mode << "): estimator=" << est_name << " messages=" << n
              << " received=" << c.received << " dispatched=" << c.dispatched << " processed=" << c.processed
              << " lost=" << c.droppedBeforeEstimator() << " wall_time_s=" << wall << "\n";
    for (std::size_t i = 0; i < c.events.size(); ++i) {
      if (c.events[i] > 0) std::cout << "  event " << seb::toString(static_cast<seb::EventType>(i)) << ": " << c.events[i] << "\n";
    }
    std::cout << "  sync: max_pending=" << st.max_pending << " max_output_depth=" << st.max_output_depth
              << " late_results=" << st.late_results << " identity_mismatches=" << st.identity_mismatches
              << " unknown_results=" << st.unknown_results << "\n";
    if (logger && logger->droppedRecords() > 0) {
      std::cout << "  WARNING: " << logger->droppedRecords() << " log records dropped (log.queue_capacity)\n";
    }
    if (st.identity_mismatches > 0 || st.unknown_results > 0 || st.duplicate_results > 0) rc = 2;
  } catch (const std::exception& e) {
    std::cerr << "seb_replay: " << e.what() << std::endl;
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
