// Invariant smoother adapter: feeds InvariantSmoother::Onestep exactly as
// invariant_smoother/apps/main_invariant_smoother.cpp does.
//
// Output semantics (from InvariantSmoother::send_states): the returned state is
// RS[frame_count], i.e. the NEWEST frame of the window after the optimisation,
// so its timestamp is the stamp of the current sample. The older (smoothed)
// frames of the window are not output. The output delay is therefore the
// optimisation time, not the window duration.

#include "seb_online/estimator_adapter.hpp"
#include "seb_online/robot_mapping.hpp"
#include "seb_online/zeroed_allocation.hpp"

#include "invariant_smoother/estimator/InvariantSmoother.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <deque>
#include <memory>
#include <sstream>

namespace seb {

std::unique_ptr<EstimatorAdapter> createSmootherAdapter(const EstimatorSettings& settings);

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

class SmootherAdapter final : public EstimatorAdapter {
 public:
  explicit SmootherAdapter(const EstimatorSettings& settings) : settings_(settings), cfg_(settings.smoother) {
    if (cfg_.window_size < 1) throw std::invalid_argument("smoother.window_size must be >= 1");
  }

  const char* name() const override { return "invariant_smoother"; }

  void reset() override {
    smoother_.reset();
    window_stamps_.clear();
  }

  std::string describe() const override {
    const auto& c = cfg_.cov;
    std::ostringstream ss;
    ss << "invariant_smoother: window_size=" << cfg_.window_size << " per_interval_dt="
       << (cfg_.per_interval_dt && settings_.common.dt_mode == DtMode::kMeasured) << " cov_exp[gyro=" << c.gyro_exp
       << " acc=" << c.acc_exp << " slip=" << c.slip_exp << " contact=" << c.contact_exp
       << " encoder=" << c.encoder_exp << " bg=" << c.bias_gyro_exp << " ba=" << c.bias_acc_exp
       << " prior_ori=" << c.prior_orientation_exp << " prior_vel=" << c.prior_velocity_exp
       << " prior_pos=" << c.prior_position_exp << " prior_bg=" << c.prior_bias_gyro_exp
       << " prior_ba=" << c.prior_bias_acc_exp << "] starting_point=" << cfg_.starting_point
       << " slip_rejection=" << cfg_.slip_rejection << " slip_threshold=" << cfg_.slip_threshold
       << " variable_contact_cov=" << cfg_.variable_contact_cov << " cov_amplifier=" << cfg_.cov_amplifier
       << " max_iterations=" << cfg_.max_iterations << " max_backpropagate=" << cfg_.max_backpropagate
       << " backpropagate_rate=" << cfg_.backpropagate_rate << " convergence=" << cfg_.convergence
       << " retract_all=" << cfg_.retract_all;
    return ss.str();
  }

  EstimatorOutput step(const SynchronizedMeasurement& m, const StepTiming& timing) override {
    if (timing.first || !smoother_) initialize(*m.sensor);

    const SensorSample& s = *m.sensor;
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
      // Member dt: initial guess and propagation information of the new interval;
      // with per_interval_dt it is also stored as that interval's factor dt.
      const double dt = timing.first ? settings_.common.nominal_dt_s : timing.dt_s;
      smoother_->dt = dt;
      smoother_->estimator_common_struct_.dt = dt;
    } else {
      smoother_->estimator_common_struct_.dt = timing.dt_s;
    }

    smoother_->Onestep(sensor_, contact_, forkin_, state_);

    window_stamps_.push_back(s.stamp_ns);
    while (static_cast<int>(window_stamps_.size()) > cfg_.window_size + 1) window_stamps_.pop_front();

    EstimatorOutput out;
    out.R_wE = state_.Rotation;
    out.p_wE = state_.Position;
    out.v_wE = state_.Velocity;
    out.position_valid = true;
    out.gyro_bias = state_.Bias_Gyro;
    out.acc_bias = state_.Bias_Acc;
    out.gyro_bias_valid = true;
    out.acc_bias_valid = true;
    out.iterations = smoother_->iteration_number;
    out.window_frames = static_cast<int>(window_stamps_.size());
    out.window_duration_s = static_cast<double>(window_stamps_.back() - window_stamps_.front()) * 1e-9;
    return out;
  }

 private:
  void initialize(const SensorSample& s) {
    smoother_ = makeZeroed<InvariantSmoother>();
    window_stamps_.clear();

    Eigen::Matrix<double, 16, 1> x0;
    x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    if (settings_.common.init_attitude == InitAttitude::kGravityAligned) {
      const Eigen::Quaterniond q0(gravityAlignedAttitude(s.acc));
      x0.segment<4>(3) << q0.w(), q0.x(), q0.y(), q0.z();
    }

    smoother_->estimator_common_struct_.leg_no = 4;
    smoother_->Optimization_Epsilon = cfg_.convergence;
    smoother_->Max_Iteration = cfg_.max_iterations;
    smoother_->Max_backpropagate_num = cfg_.max_backpropagate;
    smoother_->backppgn_rate = cfg_.backpropagate_rate;
    smoother_->NUM_OF_TRASH_DATA = cfg_.starting_point;
    smoother_->slip_rejection_mode = cfg_.slip_rejection;
    smoother_->slip_threshold = cfg_.slip_threshold;
    smoother_->variable_contact_cov_mode = cfg_.variable_contact_cov;
    smoother_->cov_amplifier = cfg_.cov_amplifier;
    smoother_->Retract_All_flag = cfg_.retract_all;
    smoother_->window_size = cfg_.window_size;
    smoother_->use_per_interval_dt = cfg_.per_interval_dt && settings_.common.dt_mode == DtMode::kMeasured;
    smoother_->Initialize(settings_.common.nominal_dt_s, makeCovariances(cfg_.cov), x0);

    sensor_.setZero();
    contact_.setZero();
    forkin_.forkin_position.assign(4, Eigen::Vector3d::Zero());
    forkin_.forkin_jacobian.assign(4, Eigen::Matrix3d::Zero());
  }

  EstimatorSettings settings_;
  SmootherSettings cfg_;
  ZeroedPtr<InvariantSmoother> smoother_;
  Eigen::Matrix<double, 30, 1> sensor_ = Eigen::Matrix<double, 30, 1>::Zero();
  Eigen::Matrix<bool, 4, 1> contact_ = Eigen::Matrix<bool, 4, 1>::Zero();
  MEAS_FORWARD_KINEMATICS forkin_;
  ROBOT_STATES state_{};
  std::deque<std::int64_t> window_stamps_;
};

}  // namespace

std::unique_ptr<EstimatorAdapter> createSmootherAdapter(const EstimatorSettings& settings) {
  return std::make_unique<SmootherAdapter>(settings);
}

}  // namespace seb
