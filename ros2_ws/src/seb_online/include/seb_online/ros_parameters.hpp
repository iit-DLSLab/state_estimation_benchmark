#pragma once

// ROS 2 parameter declaration / loading, shared by the online node and the
// deterministic bag replay tool. All parameters, their defaults and meaning are
// listed in config/go2_online.yaml.

#include "seb_online/pipeline.hpp"

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include <memory>
#include <string>

namespace seb {

struct NodeSettings {
  PipelineConfig pipeline;

  // input
  std::string lowstate_topic = "/lowstate";
  int lowstate_qos_depth = 10;
  bool lowstate_qos_reliable = false;

  // output
  std::string output_topic = "~/state";
  std::string world_frame = "odom";
  std::string base_frame = "base";
  bool publish_odometry = false;
  std::string odometry_topic = "~/odometry";
  double diagnostics_period_s = 1.0;

  // logging
  bool log_enabled = true;
  std::string log_directory = "~/.ros/seb_online";
  std::string run_name;
  int log_queue_capacity = 20000;
};

// Declares every parameter (if not yet declared) and returns the settings.
// Throws std::invalid_argument for invalid values. Resolves the URDF path and the
// IMU placement from the URDF.
NodeSettings declareAndLoadSettings(
    const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& params);

// "name: value" lines of every parameter of the node (run_config.yaml).
std::string dumpParameters(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& params);

// Expands a leading "~" and creates <directory>/<run_name>; returns the path.
std::string makeRunDirectory(const std::string& directory, const std::string& run_name,
                             const std::string& estimator);

}  // namespace seb
