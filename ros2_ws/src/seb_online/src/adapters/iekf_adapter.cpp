// IEKF adapter: feeds InvariantExtendedKalmanFilter::Onestep exactly as
// iekf/apps/main_iekf.cpp does, from canonical synchronized measurements.

#include "seb_online/estimator_adapter.hpp"
#include "seb_online/robot_mapping.hpp"
#include "seb_online/zeroed_allocation.hpp"

#include "iekf/estimator/InvariantExtendedKalmanFilter.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <optional>
#include <sstream>

namespace seb {

std::unique_ptr<EstimatorAdapter> createIekfAdapter(const EstimatorSettings& settings);

namespace {

EstimatorCovariances makeCovariances(const InvariantCovarianceSettings& c) {
  auto d = [](double e) { return Eigen::Vector3d::Constant(std::pow(10, e)); };
  // EstimatorCovariances does not initialise its 3x3 matrices and
  // Covariance_Reset() only writes their diagonals: start from the values set by
  // the EstimatorCommonStruct constructor (identity) so off-diagonals are zero.
  EstimatorCovariances cov = EstimatorCommonStruct().estimator_covariances_;
  cov.cov_gyro_diagonal = d(c.gyro_exp);
  cov.cov_acc_diagonal = d(c.acc_exp);
  cov.cov_slip_diagonal = d(c.slip_exp);
  cov.cov_contact_diagonal = d(c.contact_exp);
  cov.cov_enc_diagonal = d(c.encoder_exp);
  cov.cov_bias_gyro_diagonal = d(c.bias_gyro_exp);
  cov.cov_bias_acc_diagonal = d(c.bias_acc_exp);
  cov.cov_prior_orientation_diagonal = d(c.prior_orientation_exp);
  cov.cov_prior_velocity_diagonal = d(c.prior_velocity_exp);
  cov.cov_prior_position_diagonal = d(c.prior_position_exp);
  cov.cov_prior_bias_gyro_diagonal = d(c.prior_bias_gyro_exp);
  cov.cov_prior_bias_acc_diagonal = d(c.prior_bias_acc_exp);
  return cov;
}

class IekfAdapter final : public EstimatorAdapter {
 public:
  explicit IekfAdapter(const EstimatorSettings& settings) : settings_(settings), cfg_(settings.iekf) {}

  const char* name() const override { return "iekf"; }

  void reset() override {
    filter_.reset();
    restart_pose_.reset();
  }

  std::string describe() const override {
    const auto& c = cfg_.cov;
    std::ostringstream ss;
    ss << "iekf: cov_exp[gyro=" << c.gyro_exp << " acc=" << c.acc_exp << " slip=" << c.slip_exp
       << " contact=" << c.contact_exp << " encoder=" << c.encoder_exp << " bg=" << c.bias_gyro_exp
       << " ba=" << c.bias_acc_exp << " prior_ori=" << c.prior_orientation_exp
       << " prior_vel=" << c.prior_velocity_exp << " prior_pos=" << c.prior_position_exp
       << " prior_bg=" << c.prior_bias_gyro_exp << " prior_ba=" << c.prior_bias_acc_exp << "]"
       << " starting_point=" << cfg_.starting_point << " slip_rejection=" << cfg_.slip_rejection
       << " slip_threshold=" << cfg_.slip_threshold << " variable_contact_cov=" << cfg_.variable_contact_cov
       << " cov_amplifier=" << cfg_.cov_amplifier << " max_iterations=" << cfg_.max_iterations
       << " convergence=" << cfg_.convergence;
    return ss.str();
  }

  bool holdStationary(const Eigen::Vector3d& p_hold, EstimatorOutput& out) override {
    // IMU2BD is zero: the filter state is the estimation frame E.
    inekf::RobotState x = filter_->filter.getState();
    x.setVelocity(Eigen::Vector3d::Zero());
    x.setPosition(p_hold);
    filter_->filter.setState(x);
    out.p_wE = p_hold;
    out.v_wE.setZero();
    return true;
  }

  void restart(const Eigen::Matrix3d& R_wE, const Eigen::Vector3d& p_wE) override { restart_pose_ = {R_wE, p_wE}; }

  EstimatorOutput step(const SynchronizedMeasurement& m, const StepTiming& timing) override {
    if (timing.first || !filter_) initialize(*m.sensor);

    const SensorSample& s = *m.sensor;

    // Sensor_ layout of the KAIST estimators: gyro(0:3), acc(3:6), joint position
    // (6:18), joint velocity (18:30), legs in kInvariantLegOrder (RH, LH, LF, RF).
    // Joints, contacts and kinematics all use this same leg order. (The offline
    // main fills the joint blocks as RH, LH, RF, LF; those blocks only enter the
    // slip / variable-covariance logic, disabled in the benchmark configuration.)
    sensor_.segment<3>(0) = s.gyro;
    sensor_.segment<3>(3) = s.acc;
    for (int k = 0; k < kNumLegs; ++k) {
      const int leg = kInvariantLegOrder[k];
      sensor_.segment<3>(6 + 3 * k) = s.q.segment<3>(3 * leg);
      sensor_.segment<3>(18 + 3 * k) = s.dq.segment<3>(3 * leg);
      contact_(k) = m.contacts.stance[leg];
      forkin_.forkin_position[k] = m.kinematics.p_foot[leg];
      forkin_.forkin_jacobian[k] = m.kinematics.J_foot[leg];
    }

    if (settings_.common.dt_mode == DtMode::kMeasured) {
      // Strap-down propagation over the measured interval (first step: no propagation).
      const double dt = timing.first ? settings_.common.nominal_dt_s : timing.dt_s;
      filter_->dt = dt;
      filter_->estimator_common_struct_.dt = dt;
    } else {
      // Offline main: only estimator_common_struct_.dt is updated, propagation uses dt_init.
      filter_->estimator_common_struct_.dt = timing.dt_s;
    }

    filter_->Onestep(sensor_, contact_, forkin_, state_);

    EstimatorOutput out;
    out.R_wE = state_.Rotation;
    out.p_wE = state_.Position;
    out.v_wE = state_.Velocity;
    out.position_valid = true;
    out.gyro_bias = state_.Bias_Gyro;
    out.acc_bias = state_.Bias_Acc;
    out.gyro_bias_valid = true;
    out.acc_bias_valid = true;
    out.iterations = cfg_.max_iterations;
    return out;
  }

 private:
  void initialize(const SensorSample& s) {
    filter_ = makeZeroed<InvariantExtendedKalmanFilter>();

    Eigen::Matrix<double, 16, 1> x0;
    x0 << 0.0, 0.0, 0.0,      // px py pz
        1.0, 0.0, 0.0, 0.0,   // q(w,x,y,z)
        0.0, 0.0, 0.0,        // vx, vy, vz
        0.0, 0.0, 0.0,        // bgx, bgy, bgz
        0.0, 0.0, 0.0;        // bax, bay, baz
    if (restart_pose_) {
      const Eigen::Quaterniond q0(restart_pose_->first);
      x0.segment<3>(0) = restart_pose_->second;
      x0.segment<4>(3) << q0.w(), q0.x(), q0.y(), q0.z();
      restart_pose_.reset();
    } else if (settings_.common.init_attitude == InitAttitude::kGravityAligned) {
      const Eigen::Quaterniond q0(gravityAlignedAttitude(s.acc));
      x0.segment<4>(3) << q0.w(), q0.x(), q0.y(), q0.z();
    }

    filter_->estimator_common_struct_.leg_no = 4;
    filter_->Optimization_Epsilon = cfg_.convergence;
    filter_->Max_Iteration = cfg_.max_iterations;
    filter_->NUM_OF_TRASH_DATA = cfg_.starting_point;
    filter_->slip_rejection_mode = cfg_.slip_rejection;
    filter_->slip_threshold = cfg_.slip_threshold;
    filter_->variable_contact_cov_mode = cfg_.variable_contact_cov;
    filter_->cov_amplifier = cfg_.cov_amplifier;
    filter_->Initialize(settings_.common.nominal_dt_s, makeCovariances(cfg_.cov), x0);

    sensor_.setZero();
    contact_.setZero();
    forkin_.forkin_position.assign(4, Eigen::Vector3d::Zero());
    forkin_.forkin_jacobian.assign(4, Eigen::Matrix3d::Zero());
  }

  EstimatorSettings settings_;
  IekfSettings cfg_;
  ZeroedPtr<InvariantExtendedKalmanFilter> filter_;
  Eigen::Matrix<double, 30, 1> sensor_ = Eigen::Matrix<double, 30, 1>::Zero();
  Eigen::Matrix<bool, 4, 1> contact_ = Eigen::Matrix<bool, 4, 1>::Zero();
  MEAS_FORWARD_KINEMATICS forkin_;
  ROBOT_STATES state_{};
  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> restart_pose_;  // pose of the next initialisation
};

}  // namespace

std::unique_ptr<EstimatorAdapter> createIekfAdapter(const EstimatorSettings& settings) {
  return std::make_unique<IekfAdapter>(settings);
}

}  // namespace seb
