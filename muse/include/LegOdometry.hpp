#pragma once
#include "Streams.hpp"
#include <Eigen/Dense>
#include "rotations.h"

namespace muse_offline {

struct LegOdomResult {
  Eigen::Vector3d v_base_b{0,0,0};
  Eigen::Vector3d v_base_w{0,0,0};
  Eigen::Vector3d v_lf_b{0,0,0}, v_rf_b{0,0,0}, v_lh_b{0,0,0}, v_rh_b{0,0,0};
};

class LegOdometry {
public:
  explicit LegOdometry(const Eigen::Matrix3d& b_R_imu) : b_R_imu_(b_R_imu) {}

  LegOdomResult step(const SensorSample& s,
                      const FeetKin& f,
                      const Eigen::Quaterniond& q_wb_est) const
  {
    LegOdomResult out;


      Eigen::Matrix3d b_R_imu_ ;
      // b_R_imu_ << -1, 0, 0,
      //             0, 1, 0,
      //             0, 0, -1;
      Eigen::Quaterniond b_quat_imu;
      b_quat_imu.w() = 3.749399456654644e-33;
      b_quat_imu.x() = 6.123233995736766e-17;
      b_quat_imu.y() = 1.0;
      b_quat_imu.z() = 6.123233995736766e-17;
      b_R_imu_ = iit::commons::quatToRotMat(b_quat_imu).transpose();

      const Eigen::Vector3d omega_b = b_R_imu_ * s.omega_imu;

      // std::cout << "omega_b: " << omega_b.transpose() << std::endl;

      out.v_lf_b = -(f.v_lf + omega_b.cross(f.p_lf));
      out.v_rf_b = -(f.v_rf + omega_b.cross(f.p_rf));
      out.v_lh_b = -(f.v_lh + omega_b.cross(f.p_lh));
      out.v_rh_b = -(f.v_rh + omega_b.cross(f.p_rh));

      const double w_lf = s.stance_lf ? 1.0 : 0.0;
      const double w_rf = s.stance_rf ? 1.0 : 0.0;
      const double w_lh = s.stance_lh ? 1.0 : 0.0;
      const double w_rh = s.stance_rh ? 1.0 : 0.0;
      const double sum  = w_lf + w_rf + w_lh + w_rh;
      // std::cout << "sum:" << sum << std::endl;

      out.v_base_b = (w_lf*out.v_lf_b + w_rf*out.v_rf_b + w_lh*out.v_lh_b + w_rh*out.v_rh_b) / (sum + 1e-5);

      // world velocity: same convention you used before (w_R_b)
      // If q_wb_est is quaternion of body in world, then w_R_b = R(q).transpose() or R(q)? depends on your convention.
      const Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(q_wb_est).transpose();
      out.v_base_w = w_R_b * out.v_base_b;
      // std::cout << "out.v_base_w: " << out.v_base_w.transpose() << std::endl;

      return out;
  }

// private:
  Eigen::Matrix3d b_R_imu_;
};

inline Eigen::Matrix3d b_R_imu_anymalD() {
    Eigen::Matrix3d R;
    // R << -1, 0, 0,
    //      0, 1, 0,
    //      0, 0, -1;
    return R;                                       //(Eigen::AngleAxisd(-M_PI/2,  Eigen::Vector3d::UnitX()).toRotationMatrix() *
                                                    // Eigen::AngleAxisd( 0.0,     Eigen::Vector3d::UnitY()).toRotationMatrix() *
                                                    // Eigen::AngleAxisd( M_PI/2,  Eigen::Vector3d::UnitZ()).toRotationMatrix());

    /*
      frame_id: "base"
    child_frame_id: "imu_link"
    transform: 
      translation: 
        x: -0.25565
        y: 0.00255
        z: 0.07672
      rotation: 
        x: 6.123233995736766e-17
        y: 1.0
        z: 6.123233995736766e-17
        w: 3.749399456654644e-33
    */

    Eigen::Quaterniond b_quat_imu;
    b_quat_imu.w() = 3.749399456654644e-33;
    b_quat_imu.x() = 6.123233995736766e-17;
    b_quat_imu.y() = 1.0;
    b_quat_imu.z() = 6.123233995736766e-17;
    R = iit::commons::quatToRotMat(b_quat_imu).transpose();
    return R;

}

} // namespace muse_offline
