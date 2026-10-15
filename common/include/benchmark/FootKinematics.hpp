#pragma once

// Foot kinematics shared by the offline precomputation (data_process) and the
// online ROS 2 kinematics worker. The computation is the one originally written
// in data_process/apps/precompute_feet_kinematics.cpp:
//   - fixed-base Pinocchio model built from the URDF,
//   - foot positions from the frame placements,
//   - foot velocities from getFrameVelocity(LOCAL_WORLD_ALIGNED),
//   - 3x3 per-leg linear Jacobians from computeFrameJacobian(LOCAL_WORLD_ALIGNED).
//
// Leg order of every array is the benchmark order LF, RF, LH, RH, and the joint
// order within a leg is HAA (hip ab/adduction), HFE (hip flexion), KFE (knee).

#include <Eigen/Core>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <string>

namespace benchmark {

struct FootKinematicsConfig {
    std::string urdf_path;
    // Foot frame names, order LF, RF, LH, RH.
    std::array<std::string, 4> foot_frames;
    // Joint names, order LF(HAA,HFE,KFE), RF(...), LH(...), RH(...).
    std::array<std::string, 12> joint_names;
    // Frame in which results are expressed. Empty: the fixed-base model's world
    // frame, i.e. the URDF root link (original offline behaviour). Otherwise the
    // results are re-expressed in the named frame.
    std::string base_frame;
};

struct FootKinematicsSample {
    std::array<Eigen::Vector3d, 4> position;  // foot position [m]
    std::array<Eigen::Vector3d, 4> velocity;  // foot velocity relative to the base [m/s]
    std::array<Eigen::Matrix3d, 4> jacobian;  // d(position)/d(q_leg), columns HAA,HFE,KFE
};

class FootKinematicsModel {
public:
    // Throws std::runtime_error if the URDF, a foot frame or a joint is missing.
    explicit FootKinematicsModel(const FootKinematicsConfig& config);

    // q and dq: 12 joint positions/velocities in the order of config.joint_names.
    // Not thread-safe: one instance per thread (it owns its pinocchio::Data).
    void compute(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& q,
                 const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& dq,
                 FootKinematicsSample& out);

    const pinocchio::Model& model() const { return model_; }
    const FootKinematicsConfig& config() const { return config_; }

    // Placement of an arbitrary model frame expressed in the result frame
    // (base_frame or model world), at the given joint configuration.
    Eigen::Vector3d framePosition(const std::string& frame,
                                  const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& q);

private:
    FootKinematicsConfig config_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::array<pinocchio::FrameIndex, 4> foot_ids_{};
    pinocchio::FrameIndex base_id_{0};
    bool use_base_frame_{false};
    std::array<int, 12> idx_q_{};
    std::array<int, 12> idx_v_{};
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> J6_;
};

}  // namespace benchmark
