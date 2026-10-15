#include "benchmark/FootKinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <sstream>
#include <stdexcept>

namespace benchmark {

namespace {

std::string listFrames(const pinocchio::Model& model)
{
    std::ostringstream ss;
    for (const auto& f : model.frames) ss << " " << f.name;
    return ss.str();
}

std::string listJoints(const pinocchio::Model& model)
{
    std::ostringstream ss;
    for (const auto& n : model.names) ss << " " << n;
    return ss.str();
}

}  // namespace

FootKinematicsModel::FootKinematicsModel(const FootKinematicsConfig& config)
    : config_(config)
{
    // Fixed-base model, as in the original offline precomputation.
    pinocchio::urdf::buildModel(config_.urdf_path, model_);
    data_ = pinocchio::Data(model_);

    for (int leg = 0; leg < 4; ++leg) {
        const auto& name = config_.foot_frames[leg];
        if (!model_.existFrame(name)) {
            throw std::runtime_error("Foot frame not found in URDF: '" + name +
                                     "'. Available frames:" + listFrames(model_));
        }
        foot_ids_[leg] = model_.getFrameId(name);
    }

    for (int i = 0; i < 12; ++i) {
        const auto& name = config_.joint_names[i];
        if (!model_.existJointName(name)) {
            throw std::runtime_error("Joint not found in URDF: '" + name +
                                     "'. Available joints:" + listJoints(model_));
        }
        const auto jid = model_.getJointId(name);
        if (model_.joints[jid].nq() != 1 || model_.joints[jid].nv() != 1) {
            throw std::runtime_error("Joint '" + name + "' is not a 1-DoF joint.");
        }
        idx_q_[i] = model_.joints[jid].idx_q();
        idx_v_[i] = model_.joints[jid].idx_v();
    }

    if (!config_.base_frame.empty()) {
        if (!model_.existFrame(config_.base_frame)) {
            throw std::runtime_error("Base frame not found in URDF: '" + config_.base_frame +
                                     "'. Available frames:" + listFrames(model_));
        }
        base_id_ = model_.getFrameId(config_.base_frame);
        use_base_frame_ = true;
    }

    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    J6_.setZero(6, model_.nv);
}

void FootKinematicsModel::compute(const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& q,
                                  const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& dq,
                                  FootKinematicsSample& out)
{
    for (int i = 0; i < 12; ++i) {
        q_[idx_q_[i]] = q[i];
        v_[idx_v_[i]] = dq[i];
    }

    pinocchio::forwardKinematics(model_, data_, q_, v_);
    pinocchio::updateFramePlacements(model_, data_);

    for (int leg = 0; leg < 4; ++leg) {
        const auto fid = foot_ids_[leg];

        // Position of the foot frame in the model world frame.
        out.position[leg] = data_.oMf[fid].translation();

        // Frame velocity in LOCAL_WORLD_ALIGNED (origin at the frame, world axes).
        const pinocchio::Motion v6 =
            pinocchio::getFrameVelocity(model_, data_, fid, pinocchio::LOCAL_WORLD_ALIGNED);
        out.velocity[leg] = v6.linear();

        // Jacobian in the same reference frame, sliced on the 3 joints of the leg.
        J6_.setZero();
        pinocchio::computeFrameJacobian(model_, data_, q_, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6_);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                out.jacobian[leg](r, c) = J6_(r, idx_v_[leg * 3 + c]);
            }
        }
    }

    if (use_base_frame_) {
        // Fixed-base model: the base frame does not move, so relative velocities
        // and Jacobians only need to be rotated into base axes.
        const pinocchio::SE3& oMb = data_.oMf[base_id_];
        const Eigen::Matrix3d R_bw = oMb.rotation().transpose();
        for (int leg = 0; leg < 4; ++leg) {
            out.position[leg] = R_bw * (out.position[leg] - oMb.translation());
            out.velocity[leg] = R_bw * out.velocity[leg];
            out.jacobian[leg] = R_bw * out.jacobian[leg];
        }
    }
}

Eigen::Vector3d FootKinematicsModel::framePosition(
    const std::string& frame, const Eigen::Ref<const Eigen::Matrix<double, 12, 1>>& q)
{
    if (!model_.existFrame(frame)) {
        throw std::runtime_error("Frame not found in URDF: '" + frame + "'");
    }
    for (int i = 0; i < 12; ++i) q_[idx_q_[i]] = q[i];
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d p_w = data_.oMf[model_.getFrameId(frame)].translation();
    if (!use_base_frame_) return p_w;
    const pinocchio::SE3& oMb = data_.oMf[base_id_];
    return oMb.rotation().transpose() * (p_w - oMb.translation());
}

}  // namespace benchmark
