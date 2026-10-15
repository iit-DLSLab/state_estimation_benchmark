// MUSE adapter: runs the per-sample body of muse/apps/main_muse.cpp
// (attitude XKF -> leg odometry -> KF sensor fusion) on synchronized measurements.

#include "seb_online/estimator_adapter.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Models/SensorFusion.hpp"
#include "Models/attitude_bias_XKF.hpp"
#include "lib.hpp"
#include "rotations.h"

#include <memory>
#include <sstream>

namespace seb {

std::unique_ptr<EstimatorAdapter> createMuseAdapter(const EstimatorSettings& settings);

namespace {

class MuseAdapter final : public EstimatorAdapter {
 public:
  explicit MuseAdapter(const EstimatorSettings& settings) : settings_(settings), cfg_(settings.muse) {}

  const char* name() const override { return "muse"; }

  void reset() override {
    attitude_.reset();
    kf_.reset();
  }

  std::string describe() const override {
    std::ostringstream ss;
    ss << "muse: ki=" << cfg_.ki << " kp=" << cfg_.kp << " attitude_P0=" << cfg_.attitude_P0
       << " attitude_Q=" << cfg_.attitude_Q << " attitude_R=" << cfg_.attitude_R << " fusion_P0=" << cfg_.fusion_P0
       << " fusion_Q=" << cfg_.fusion_Q << " fusion_R=" << cfg_.fusion_R << " gravity=" << cfg_.gravity
       << " use_updated_attitude=" << cfg_.use_updated_attitude;
    return ss.str();
  }

  EstimatorOutput step(const SynchronizedMeasurement& m, const StepTiming& timing) override {
    if (timing.first || !attitude_) initialize(*m.sensor);

    const SensorSample& s = *m.sensor;
    const double t_rel = timing.t_rel_s;
    const Eigen::Vector3d& omega = s.gyro;  // base axes
    const Eigen::Vector3d& acc = s.acc;     // base axes

    // ---- 1) ATTITUDE (main_muse.cpp) ----
    Eigen::Quaterniond quat_est;
    quat_est.w() = xhat_estimated_(0);
    quat_est.vec() << xhat_estimated_(1), xhat_estimated_(2), xhat_estimated_(3);

    Eigen::Vector3d f_b = b_R_imu_ * acc;
    Eigen::Vector3d m_b = iit::commons::quatToRotMat(quat_est) * m_n_;

    Eigen::Matrix<double, 6, 1> z;
    z << f_b, m_b;

    attitude_->update(t_rel, b_R_imu_ * omega, z);
    xhat_estimated_ = attitude_->getX();

    if (cfg_.use_updated_attitude) {
      quat_est.w() = xhat_estimated_(0);
      quat_est.vec() << xhat_estimated_(1), xhat_estimated_(2), xhat_estimated_(3);
    }

    // ---- 2) LEG ODOMETRY ----
    //   v_leg_b = -(vfoot + omega_b x pfoot), weighted average over stance legs
    const Eigen::Vector3d omega_b = b_R_imu_ * omega;
    const auto& kin = m.kinematics;
    const auto& stance = m.contacts.stance;

    const Eigen::Vector3d v_lf_b = -(kin.v_foot[LF] + omega_b.cross(kin.p_foot[LF]));
    const Eigen::Vector3d v_rf_b = -(kin.v_foot[RF] + omega_b.cross(kin.p_foot[RF]));
    const Eigen::Vector3d v_lh_b = -(kin.v_foot[LH] + omega_b.cross(kin.p_foot[LH]));
    const Eigen::Vector3d v_rh_b = -(kin.v_foot[RH] + omega_b.cross(kin.p_foot[RH]));

    const double w_lf = stance[LF] ? 1.0 : 0.0;
    const double w_rf = stance[RF] ? 1.0 : 0.0;
    const double w_lh = stance[LH] ? 1.0 : 0.0;
    const double w_rh = stance[RH] ? 1.0 : 0.0;
    double sum = w_lf + w_rf + w_lh + w_rh;
    if (sum < 1.0) sum = 1.0;

    const Eigen::Vector3d v_base_b = (w_lf * v_lf_b + w_rf * v_rf_b + w_lh * v_lh_b + w_rh * v_rh_b) / (sum + 1e-5);

    const Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
    const Eigen::Vector3d v_base_w = w_R_b * v_base_b;

    // ---- 3) SENSOR FUSION (KF) ----
    const Eigen::Vector3d gravity(0.0, 0.0, -cfg_.gravity);
    const Eigen::Vector3d u_w = w_R_b * f_b + gravity;

    kf_->predict(t_rel, u_w);
    kf_->update(t_rel, v_base_w);

    const auto xhat = kf_->getX();  // [px py pz vx vy vz]

    EstimatorOutput out;
    out.R_wE = w_R_b;
    out.p_wE = xhat.head<3>();
    out.v_wE = xhat.segment<3>(3);
    out.position_valid = true;
    out.gyro_bias = xhat_estimated_.tail<3>();
    out.gyro_bias_valid = true;
    out.acc_bias_valid = false;  // MUSE does not estimate an accelerometer bias
    return out;
  }

 private:
  void initialize(const SensorSample& s) {
    b_R_imu_.setIdentity();  // IMU data are already rotated into base axes by the acquisition stage
    m_n_ = Eigen::Vector3d(1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0));
    const Eigen::Vector3d f_n(0.0, 0.0, cfg_.gravity);

    const Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Identity() * cfg_.attitude_P0;
    const Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity() * cfg_.attitude_Q;
    const Eigen::Matrix<double, 6, 6> Ratt = Eigen::Matrix<double, 6, 6>::Identity() * cfg_.attitude_R;

    xhat_estimated_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    if (settings_.common.init_attitude == InitAttitude::kGravityAligned) {
      const Eigen::Quaterniond q0(gravityAlignedAttitude(s.acc));
      xhat_estimated_ << q0.w(), q0.x(), q0.y(), q0.z(), 0.0, 0.0, 0.0;
    }
    xhat_estimated_.head<4>() /= xhat_estimated_.head<4>().norm();

    attitude_ = std::make_unique<state_estimator::AttitudeBiasXKF>(0.0, xhat_estimated_, P0, Q, Ratt, f_n, m_n_,
                                                                   cfg_.ki, cfg_.kp);

    Eigen::Matrix<double, 6, 1> x0;
    x0.setZero();
    const Eigen::Matrix<double, 6, 6> Psf = Eigen::Matrix<double, 6, 6>::Identity() * cfg_.fusion_P0;
    const Eigen::Matrix<double, 6, 6> Qsf = Eigen::Matrix<double, 6, 6>::Identity() * cfg_.fusion_Q;
    const Eigen::Matrix<double, 3, 3> Rsf = Eigen::Matrix<double, 3, 3>::Identity() * cfg_.fusion_R;
    kf_ = std::make_unique<state_estimator::KFSensorFusion>(0.0, x0, Psf, Qsf, Rsf, false, false);
  }

  EstimatorSettings settings_;
  MuseSettings cfg_;
  Eigen::Matrix3d b_R_imu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d m_n_ = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 7, 1> xhat_estimated_ = Eigen::Matrix<double, 7, 1>::Zero();
  std::unique_ptr<state_estimator::AttitudeBiasXKF> attitude_;
  std::unique_ptr<state_estimator::KFSensorFusion> kf_;
};

}  // namespace

std::unique_ptr<EstimatorAdapter> createMuseAdapter(const EstimatorSettings& settings) {
  return std::make_unique<MuseAdapter>(settings);
}

}  // namespace seb
