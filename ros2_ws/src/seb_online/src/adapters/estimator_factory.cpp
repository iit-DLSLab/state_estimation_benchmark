#include "seb_online/estimator_adapter.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace seb {

std::unique_ptr<EstimatorAdapter> createMuseAdapter(const EstimatorSettings& settings);
std::unique_ptr<EstimatorAdapter> createIekfAdapter(const EstimatorSettings& settings);
std::unique_ptr<EstimatorAdapter> createSmootherAdapter(const EstimatorSettings& settings);

std::unique_ptr<EstimatorAdapter> createEstimator(const EstimatorSettings& settings) {
  if (settings.type == "muse") return createMuseAdapter(settings);
  if (settings.type == "iekf") return createIekfAdapter(settings);
  if (settings.type == "invariant_smoother") return createSmootherAdapter(settings);
  throw std::invalid_argument("Unknown estimator type '" + settings.type +
                              "' (expected muse, iekf or invariant_smoother)");
}

Eigen::Matrix3d gravityAlignedAttitude(const Eigen::Vector3d& acc_base) {
  const double n = acc_base.norm();
  if (!acc_base.allFinite() || n < 1e-3) return Eigen::Matrix3d::Identity();
  // At rest the accelerometer measures the specific force, +g along world z,
  // expressed in base axes: f_b = R_wb^T * (0, 0, g). Find R_wb with R_wb f_b ~ z.
  return Eigen::Quaterniond::FromTwoVectors(acc_base / n, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d restartAttitude(const Eigen::Matrix3d& R_prev, const Eigen::Vector3d& acc_base) {
  const Eigen::Matrix3d R_tilt = gravityAlignedAttitude(acc_base);
  auto heading = [](const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); };
  return Eigen::AngleAxisd(heading(R_prev) - heading(R_tilt), Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_tilt;
}

}  // namespace seb
