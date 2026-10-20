#pragma once

// Common interface of the three estimators (MUSE, IEKF, invariant smoother).
//
// Adapters convert the canonical SynchronizedMeasurement into each estimator's
// own input representation (leg order, vector layout) and call the UNMODIFIED
// estimator update of the offline benchmark. The default settings below are the
// values of the offline mains (muse/apps/main_muse.cpp, iekf/apps/main_iekf.cpp,
// invariant_smoother/apps/main_invariant_smoother.cpp).
//
// Threading: an adapter is owned and called by exactly one thread (the
// estimator worker). Mutable update functions are never called concurrently.

#include "seb_online/types.hpp"

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <string>

namespace seb {

enum class InitAttitude {
  kIdentity,        // offline benchmark behaviour
  kGravityAligned,  // roll/pitch from the first accelerometer sample, yaw = 0
};

enum class DtMode {
  // dt of every step = measured interval between the current and the previous
  // processed sample (in the configured time base). Default.
  kMeasured,
  // Reproduce the offline mains exactly: IEKF and smoother keep the fixed member
  // dt = nominal_dt_s for the strap-down propagation, while the smoother factors
  // use the measured dt of the latest step. Used by the regression test only.
  kOfflineCompat,
};

struct CommonEstimatorSettings {
  InitAttitude init_attitude = InitAttitude::kGravityAligned;
  DtMode dt_mode = DtMode::kMeasured;
  double nominal_dt_s = 0.002;  // initial dt passed to Initialize(); dt of kOfflineCompat
};

struct MuseSettings {
  // Attitude observer (AttitudeBiasXKF)
  double ki = 0.02;
  double kp = 10.0;
  double attitude_P0 = 1e-12;
  double attitude_Q = 1e-15;
  double attitude_R = 1e-6;
  // Sensor fusion KF (KFSensorFusion)
  double fusion_P0 = 1e-14;
  double fusion_Q = 1e-14;
  double fusion_R = 5e-16;
  double gravity = 9.81;
  // The offline main uses the attitude of the PREVIOUS step to rotate leg
  // odometry and accelerations and to output the orientation (one-sample lag).
  // true (default online): use the attitude updated with the current sample, so
  // that the published orientation corresponds to the state timestamp.
  // false: offline behaviour (used by the regression test).
  bool use_updated_attitude = true;
};

// Covariances as powers of ten, exactly as in the offline mains.
struct InvariantCovarianceSettings {
  double gyro_exp = -4, acc_exp = -1, slip_exp = -1.3, contact_exp = -4, encoder_exp = -5;
  double bias_gyro_exp = -10, bias_acc_exp = -10;
  double prior_orientation_exp = -8, prior_velocity_exp = -8, prior_position_exp = -8;
  double prior_bias_gyro_exp = -10, prior_bias_acc_exp = -10;
};

struct IekfSettings {
  InvariantCovarianceSettings cov;
  int starting_point = 1;
  bool slip_rejection = false;
  double slip_threshold = 0.48;
  bool variable_contact_cov = false;
  double cov_amplifier = 1.0;
  int max_iterations = 1;
  double convergence = 1e-3;
};

struct SmootherSettings {
  InvariantCovarianceSettings cov;
  int window_size = 3;  // offline compile-time value
  int starting_point = 1;
  bool slip_rejection = false;
  double slip_threshold = 0.48;
  bool variable_contact_cov = false;
  double cov_amplifier = 1.0;
  int max_iterations = 10;
  int max_backpropagate = 10;
  double backpropagate_rate = 0.5;
  double convergence = 1e-3;
  bool retract_all = false;
  // Use the measured duration of each interval in its propagation factor
  // (InvariantSmoother::use_per_interval_dt). Only effective with DtMode::kMeasured.
  bool per_interval_dt = true;
};

struct EstimatorSettings {
  std::string type = "muse";  // "muse" | "iekf" | "invariant_smoother"
  CommonEstimatorSettings common;
  MuseSettings muse;
  IekfSettings iekf;
  SmootherSettings smoother;
};

struct StepTiming {
  bool first = false;     // first sample after (re)initialisation
  double dt_s = 0.0;      // interval since the previous processed sample (0 if first)
  double t_rel_s = 0.0;   // time since the first sample of the current initialisation
};

// Estimate of the estimation frame E (base axes, IMU origin) in the world frame.
struct EstimatorOutput {
  Eigen::Matrix3d R_wE = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_wE = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_wE = Eigen::Vector3d::Zero();
  bool position_valid = true;
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  bool gyro_bias_valid = false;
  bool acc_bias_valid = false;
  // Estimator-specific diagnostics (NaN / -1 when not applicable).
  int iterations = -1;
  int window_frames = -1;                                            // smoother: frames in the window
  double window_duration_s = std::numeric_limits<double>::quiet_NaN();  // smoother: newest - oldest frame stamp
};

class EstimatorAdapter {
 public:
  virtual ~EstimatorAdapter() = default;
  virtual const char* name() const = 0;
  // Discards the internal state; the next step() must have timing.first = true.
  virtual void reset() = 0;
  virtual EstimatorOutput step(const SynchronizedMeasurement& m, const StepTiming& timing) = 0;
  // Called right after step() on a sample with no leg in stance: writes into the
  // internal state a zero velocity and the position p_hold (frame E, world), sets
  // `out` accordingly and returns true. Attitude and biases are left to the
  // estimator. The default returns false and changes nothing: the estimator handles
  // such a sample itself (MUSE fuses a zero leg-odometry velocity, as the official one).
  virtual bool holdStationary(const Eigen::Vector3d& /*p_hold*/, EstimatorOutput& /*out*/) { return false; }
  // Called when stance returns after a long hold: the next step() (timing.first)
  // re-initialises the estimator at this pose with zero velocity, discarding the
  // covariance / marginalisation built from the IMU-only interval. reset() is NOT
  // called before it, so the estimator object may be reused. Only called on
  // estimators whose holdStationary() returned true.
  virtual void restart(const Eigen::Matrix3d& /*R_wE*/, const Eigen::Vector3d& /*p_wE*/) {}
  // Human-readable configuration (written to the run log).
  virtual std::string describe() const = 0;
};

// Creates exactly one estimator. Throws std::invalid_argument for unknown types.
std::unique_ptr<EstimatorAdapter> createEstimator(const EstimatorSettings& settings);

// Rotation body->world with the measured specific force mapped onto +z (yaw
// arbitrary, minimal rotation). Identity if the acceleration is not usable.
Eigen::Matrix3d gravityAlignedAttitude(const Eigen::Vector3d& acc_base);

// Roll/pitch from the accelerometer (robot at rest), heading of R_prev: attitude
// for a restart after a hold, where roll/pitch were integrated from the gyro alone.
Eigen::Matrix3d restartAttitude(const Eigen::Matrix3d& R_prev, const Eigen::Vector3d& acc_base);

}  // namespace seb
