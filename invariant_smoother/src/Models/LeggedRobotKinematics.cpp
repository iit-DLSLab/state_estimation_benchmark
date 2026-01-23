// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.

#include <Models/LeggedRobotKinematics.h>



void LeggedRobotKinematics::GetJacobian(int lnum,
										const Eigen::Vector3d &joint_val,
										Eigen::Matrix3d &j) {
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  double l_1 = Robot::RollToPitch * (2 * (lnum % 2) - 1); // -1 for the right and 1 for the left
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + Robot::FootRadius;

  j(0, 0) = 0;
  j(0, 1) = -l_2 * cos(p) - l_3 * cos(p + k);
  j(0, 2) = -l_3 * cos(p + k);

  j(1, 0) = -l_1 * sin(r) + cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(1, 1) = -sin(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(1, 2) = -l_3 * sin(r) * sin(p + k);

  j(2, 0) = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(2, 1) = cos(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(2, 2) = l_3 * cos(r) * sin(p + k);
}

Eigen::Matrix3d LeggedRobotKinematics::GetJacobian(int lnum,
										const Eigen::Vector3d &joint_val) {

  Eigen::Matrix3d j;
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  double l_1 = Robot::RollToPitch * (2 * (lnum % 2) - 1); // -1 for the right and 1 for the left
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + Robot::FootRadius;

  j(0, 0) = 0;
  j(0, 1) = -l_2 * cos(p) - l_3 * cos(p + k);
  j(0, 2) = -l_3 * cos(p + k);

  j(1, 0) = -l_1 * sin(r) + cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(1, 1) = -sin(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(1, 2) = -l_3 * sin(r) * sin(p + k);

  j(2, 0) = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(2, 1) = cos(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(2, 2) = l_3 * cos(r) * sin(p + k);

  return j;
}


void LeggedRobotKinematics::GetJacobianDerivative(int lnum,
												  const Eigen::Vector3d &joint_pos,
												  const Eigen::Vector3d &joint_vel,
												  Eigen::Matrix3d &d_j) {
    int rightleft = Robot::LegSign[2*lnum+1];
    int fronthind = Robot::LegSign[2*lnum];

  double q_1 = joint_pos[0];  // Roll angle
  double q_2 = joint_pos[1];  // Pitch angle
  double q_3 = joint_pos[2];  // Knee angle

  double dq_1 = joint_vel[0];  // Roll angle
  double dq_2 = joint_vel[1];  // Pitch angle
  double dq_3 = joint_vel[2];  // Knee angle

  double l_1 = Robot::RollToPitch * rightleft; // -1 for the right and 1 for the left
  double l_2 = -Robot::ThighLength;
  double l_3 = -(Robot::CalfLength + Robot::FootRadius);

  d_j(0, 0) = 0;
  d_j(0, 1) = -dq_2 * (l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2)) + l_2 * sin(q_2)) -
             dq_3 * l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2));
  d_j(0, 2) = -dq_2 * l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2)) - dq_3 * l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2));
  d_j(1, 0) = dq_2 * (l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) + l_2 * cos(q_1) * sin(q_2)) -
             dq_1 * (l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1)) + l_1 * cos(q_1) -
                    l_2 * cos(q_2) * sin(q_1)) + dq_3 * l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2));
  d_j(1, 1) = dq_1 * (l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) + l_2 * cos(q_1) * sin(q_2)) -
             dq_2 * (l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1)) - l_2 * cos(q_2) * sin(q_1)) -
             dq_3 * l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1));
  d_j(1, 2) = dq_1 * l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) -
             dq_2 * l_3 * (sin(q_1) * sin(q_2) * sin(q_3) -
                         cos(q_2) * cos(q_3) * sin(q_1)) -
             dq_3 * l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1));
  d_j(2, 0) = dq_2 * (l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2)) + l_2 * sin(q_1) * sin(q_2)) -
             dq_1 * (l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3)) + l_1 * sin(q_1) +
                    l_2 * cos(q_1) * cos(q_2)) + dq_3 * l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2));
  d_j(2, 1) = dq_1 * (l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2)) + l_2 * sin(q_1) * sin(q_2)) -
             dq_2 * (l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3)) + l_2 * cos(q_1) * cos(q_2)) -
             dq_3 * l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3));
  d_j(2, 2) = dq_1 * l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2)) -
             dq_3 * l_3 * (cos(q_1) * cos(q_2) * cos(q_3) -
                         cos(q_1) * sin(q_2) * sin(q_3)) -
             dq_2 * l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3));

}

    Eigen::Vector3d LeggedRobotKinematics::GetHiptoFootPosition(int lnum,
																const Eigen::Vector3d &joint_val) {
        double r = joint_val[0];  // Roll angle
        double p = joint_val[1];  // Pitch angle
        double k = joint_val[2];  // Knee angle

        int rightleft = Robot::LegSign[2*lnum+1];
        int fronthind = Robot::LegSign[2*lnum];

        double l_1 = Robot::RollToPitch * rightleft;
        double l_2 = Robot::ThighLength;
        double l_3 = Robot::CalfLength + Robot::FootRadius;

        Eigen::Vector3d b_pos_hip_2_ft;
      b_pos_hip_2_ft[0] = -l_2 * sin(p) - l_3 * sin(p + k);
      b_pos_hip_2_ft[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
      b_pos_hip_2_ft[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));

        return b_pos_hip_2_ft;
    }


Eigen::Vector3d LeggedRobotKinematics::GetBodyToHipPosition(int lnum)
{
  int rightleft = Robot::LegSign[2*lnum+1];
  int fronthind = Robot::LegSign[2*lnum];
  Eigen::Vector3d b_pos_bd_2_hip;
  b_pos_bd_2_hip[0] = fronthind * Robot::PelvisToRoll[0];
  b_pos_bd_2_hip[1] = rightleft * Robot::PelvisToRoll[1];
  b_pos_bd_2_hip[2] = Robot::PelvisToRoll[2];
  return b_pos_bd_2_hip;

}

Eigen::Vector3d LeggedRobotKinematics::GetFootPosition(int lnum,
													   const Eigen::Vector3d &joint_val) {
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  int rightleft = Robot::LegSign[2*lnum+1];
  int fronthind = Robot::LegSign[2*lnum];


//  int rightleft = 2 * (lnum % 2) - 1; // -1 for right and 1 for left

  double l_1 = Robot::RollToPitch * rightleft;
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + Robot::FootRadius;

  Eigen::Vector3d b_pos_hip_2_ft;
  b_pos_hip_2_ft[0] = -l_2 * sin(p) - l_3 * sin(p + k);
  b_pos_hip_2_ft[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  b_pos_hip_2_ft[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));

  Eigen::Vector3d b_pos_bd_2_hip;
  b_pos_bd_2_hip[0] = fronthind * Robot::PelvisToRoll[0];
  b_pos_bd_2_hip[1] = rightleft * Robot::PelvisToRoll[1];
  b_pos_bd_2_hip[2] = Robot::PelvisToRoll[2];

  Eigen::Vector3d b_pos_bd_2_ft = b_pos_hip_2_ft + b_pos_bd_2_hip;
  return b_pos_bd_2_ft;
}

Eigen::Matrix<double,3,4> LeggedRobotKinematics::GetPelvisKneeFootPosition(int lnum, const Eigen::Matrix3d &rotation,
                                                                 const Eigen::Vector3d &joint_val) {
    return rotation*GetPelvisKneeFootPosition(lnum, joint_val);
}

Eigen::Matrix<double,3,4> LeggedRobotKinematics::GetPelvisKneeFootPosition(int lnum,
                                                                 const Eigen::Vector3d &joint_val) {
    Eigen::Matrix<double,3,4> ret_value;
    double r = joint_val[0];  // Roll angle
    double p = joint_val[1];  // Pitch angle
    double k = joint_val[2];  // Knee angle

    int rightleft = Robot::LegSign[2 * lnum + 1];
    int fronthind = Robot::LegSign[2 * lnum];


    double l_1 = Robot::RollToPitch * rightleft;
    double l_2 = Robot::ThighLength;
    double l_3 = Robot::CalfLength + Robot::FootRadius;

    Eigen::Vector3d b_pos_hip_2_pelvis;
    b_pos_hip_2_pelvis[0] = 0;
    b_pos_hip_2_pelvis[1] = l_1 * cos(r);
    b_pos_hip_2_pelvis[2] = l_1 * sin(r);

    Eigen::Vector3d b_pos_hip_2_knee;
    b_pos_hip_2_knee[0] = -l_2 * sin(p);
    b_pos_hip_2_knee[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p));
    b_pos_hip_2_knee[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p));

    Eigen::Vector3d b_pos_hip_2_ft;
    b_pos_hip_2_ft[0] = -l_2 * sin(p) - l_3 * sin(p + k);
    b_pos_hip_2_ft[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
    b_pos_hip_2_ft[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));

    Eigen::Vector3d b_pos_bd_2_hip;
    b_pos_bd_2_hip[0] = fronthind * Robot::PelvisToRoll[0];
    b_pos_bd_2_hip[1] = rightleft * Robot::PelvisToRoll[1];
    b_pos_bd_2_hip[2] = Robot::PelvisToRoll[2];

    ret_value.block(0, 0, 3, 1) = b_pos_bd_2_hip;
    ret_value.block(0, 1, 3, 1) = b_pos_bd_2_hip + b_pos_hip_2_pelvis;
    ret_value.block(0, 2, 3, 1) = b_pos_bd_2_hip + b_pos_hip_2_knee;
    ret_value.block(0, 3, 3, 1) = b_pos_bd_2_hip + b_pos_hip_2_ft;
    return ret_value;
}


void LeggedRobotKinematics::GetJacobian(int lnum,
										const Eigen::Vector3d &joint_val,
										double foot_radius,
										Eigen::Matrix3d &j) {
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  double l_1 = Robot::RollToPitch * (2 * (lnum % 2) - 1); // -1 for the right and 1 for the left
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + foot_radius;

  j(0, 0) = 0;
  j(0, 1) = -l_2 * cos(p) - l_3 * cos(p + k);
  j(0, 2) = -l_3 * cos(p + k);

  j(1, 0) = -l_1 * sin(r) + cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(1, 1) = -sin(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(1, 2) = -l_3 * sin(r) * sin(p + k);

  j(2, 0) = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(2, 1) = cos(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(2, 2) = l_3 * cos(r) * sin(p + k);
}

Eigen::Matrix3d LeggedRobotKinematics::GetJacobian(int lnum,
												   const Eigen::Vector3d &joint_val,
												   double foot_radius) {

  Eigen::Matrix3d j;
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  double l_1 = Robot::RollToPitch * (2 * (lnum % 2) - 1); // -1 for the right and 1 for the left
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + foot_radius;

  j(0, 0) = 0;
  j(0, 1) = -l_2 * cos(p) - l_3 * cos(p + k);
  j(0, 2) = -l_3 * cos(p + k);

  j(1, 0) = -l_1 * sin(r) + cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(1, 1) = -sin(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(1, 2) = -l_3 * sin(r) * sin(p + k);

  j(2, 0) = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  j(2, 1) = cos(r) * (l_2 * sin(p) + l_3 * sin(p + k));
  j(2, 2) = l_3 * cos(r) * sin(p + k);

  return j;
}


void LeggedRobotKinematics::GetJacobianDerivative(int lnum,
												  const Eigen::Vector3d &joint_pos,
												  const Eigen::Vector3d &joint_vel,
												  double foot_radius,
												  Eigen::Matrix3d &d_j) {
  int rightleft = Robot::LegSign[2*lnum+1];
  int fronthind = Robot::LegSign[2*lnum];

  double q_1 = joint_pos[0];  // Roll angle
  double q_2 = joint_pos[1];  // Pitch angle
  double q_3 = joint_pos[2];  // Knee angle

  double dq_1 = joint_vel[0];  // Roll angle
  double dq_2 = joint_vel[1];  // Pitch angle
  double dq_3 = joint_vel[2];  // Knee angle

  double l_1 = Robot::RollToPitch * rightleft; // -1 for the right and 1 for the left
  double l_2 = -Robot::ThighLength;
  double l_3 = -(Robot::CalfLength + foot_radius);

  d_j(0, 0) = 0;
  d_j(0, 1) = -dq_2 * (l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2)) + l_2 * sin(q_2)) -
	  dq_3 * l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2));
  d_j(0, 2) = -dq_2 * l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2)) - dq_3 * l_3 * (cos(q_2) * sin(q_3) + cos(q_3) * sin(q_2));
  d_j(1, 0) = dq_2 * (l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) + l_2 * cos(q_1) * sin(q_2)) -
	  dq_1 * (l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1)) + l_1 * cos(q_1) -
		  l_2 * cos(q_2) * sin(q_1)) + dq_3 * l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2));
  d_j(1, 1) = dq_1 * (l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) + l_2 * cos(q_1) * sin(q_2)) -
	  dq_2 * (l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1)) - l_2 * cos(q_2) * sin(q_1)) -
	  dq_3 * l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1));
  d_j(1, 2) = dq_1 * l_3 * (cos(q_1) * cos(q_2) * sin(q_3) + cos(q_1) * cos(q_3) * sin(q_2)) -
	  dq_2 * l_3 * (sin(q_1) * sin(q_2) * sin(q_3) -
		  cos(q_2) * cos(q_3) * sin(q_1)) -
	  dq_3 * l_3 * (sin(q_1) * sin(q_2) * sin(q_3) - cos(q_2) * cos(q_3) * sin(q_1));
  d_j(2, 0) = dq_2 * (l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2)) + l_2 * sin(q_1) * sin(q_2)) -
	  dq_1 * (l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3)) + l_1 * sin(q_1) +
		  l_2 * cos(q_1) * cos(q_2)) + dq_3 * l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2));
  d_j(2, 1) = dq_1 * (l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2)) + l_2 * sin(q_1) * sin(q_2)) -
	  dq_2 * (l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3)) + l_2 * cos(q_1) * cos(q_2)) -
	  dq_3 * l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3));
  d_j(2, 2) = dq_1 * l_3 * (cos(q_2) * sin(q_1) * sin(q_3) + cos(q_3) * sin(q_1) * sin(q_2)) -
	  dq_3 * l_3 * (cos(q_1) * cos(q_2) * cos(q_3) -
		  cos(q_1) * sin(q_2) * sin(q_3)) -
	  dq_2 * l_3 * (cos(q_1) * cos(q_2) * cos(q_3) - cos(q_1) * sin(q_2) * sin(q_3));

}

Eigen::Vector3d LeggedRobotKinematics::GetHiptoFootPosition(int lnum,
															const Eigen::Vector3d &joint_val,double foot_radius) {
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  int rightleft = Robot::LegSign[2*lnum+1];
  int fronthind = Robot::LegSign[2*lnum];

  double l_1 = Robot::RollToPitch * rightleft;
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + foot_radius;

  Eigen::Vector3d b_pos_hip_2_ft;
  b_pos_hip_2_ft[0] = -l_2 * sin(p) - l_3 * sin(p + k);
  b_pos_hip_2_ft[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  b_pos_hip_2_ft[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));

  return b_pos_hip_2_ft;
}


Eigen::Vector3d LeggedRobotKinematics::GetFootPosition(int lnum,
													   const Eigen::Vector3d &joint_val,double foot_radius) {
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  int rightleft = Robot::LegSign[2*lnum+1];
  int fronthind = Robot::LegSign[2*lnum];


//  int rightleft = 2 * (lnum % 2) - 1; // -1 for right and 1 for left

  double l_1 = Robot::RollToPitch * rightleft;
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + foot_radius;

  Eigen::Vector3d b_pos_hip_2_ft;
  b_pos_hip_2_ft[0] = -l_2 * sin(p) - l_3 * sin(p + k);
  b_pos_hip_2_ft[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  b_pos_hip_2_ft[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));

  Eigen::Vector3d b_pos_bd_2_hip;
  b_pos_bd_2_hip[0] = fronthind * Robot::PelvisToRoll[0];
  b_pos_bd_2_hip[1] = rightleft * Robot::PelvisToRoll[1];
  b_pos_bd_2_hip[2] = Robot::PelvisToRoll[2];

  Eigen::Vector3d b_pos_bd_2_ft = b_pos_hip_2_ft + b_pos_bd_2_hip;
  return b_pos_bd_2_ft;
}
Eigen::Matrix<double,3,4> LeggedRobotKinematics::GetPelvisKneeFootPosition(int lnum,
																		   const Eigen::Vector3d &joint_val,double foot_radius) {
  Eigen::Matrix<double,3,4> ret_value;
  double r = joint_val[0];  // Roll angle
  double p = joint_val[1];  // Pitch angle
  double k = joint_val[2];  // Knee angle

  int rightleft = Robot::LegSign[2 * lnum + 1];
  int fronthind = Robot::LegSign[2 * lnum];


  double l_1 = Robot::RollToPitch * rightleft;
  double l_2 = Robot::ThighLength;
  double l_3 = Robot::CalfLength + foot_radius;

  Eigen::Vector3d b_pos_hip_2_pelvis;
  b_pos_hip_2_pelvis[0] = 0;
  b_pos_hip_2_pelvis[1] = l_1 * cos(r);
  b_pos_hip_2_pelvis[2] = l_1 * sin(r);

  Eigen::Vector3d b_pos_hip_2_knee;
  b_pos_hip_2_knee[0] = -l_2 * sin(p);
  b_pos_hip_2_knee[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p));
  b_pos_hip_2_knee[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p));

  Eigen::Vector3d b_pos_hip_2_ft;
  b_pos_hip_2_ft[0] = -l_2 * sin(p) - l_3 * sin(p + k);
  b_pos_hip_2_ft[1] = l_1 * cos(r) + sin(r) * (l_2 * cos(p) + l_3 * cos(p + k));
  b_pos_hip_2_ft[2] = l_1 * sin(r) - cos(r) * (l_2 * cos(p) + l_3 * cos(p + k));

  Eigen::Vector3d b_pos_bd_2_hip;
  b_pos_bd_2_hip[0] = fronthind * Robot::PelvisToRoll[0];
  b_pos_bd_2_hip[1] = rightleft * Robot::PelvisToRoll[1];
  b_pos_bd_2_hip[2] = Robot::PelvisToRoll[2];

  ret_value.block(0, 0, 3, 1) = b_pos_bd_2_hip;
  ret_value.block(0, 1, 3, 1) = b_pos_bd_2_hip + b_pos_hip_2_pelvis;
  ret_value.block(0, 2, 3, 1) = b_pos_bd_2_hip + b_pos_hip_2_knee;
  ret_value.block(0, 3, 3, 1) = b_pos_bd_2_hip + b_pos_hip_2_ft;
  return ret_value;
}


Eigen::Vector3d LeggedRobotKinematics::GetFootVelocity(int lnum,
													   const Eigen::Vector3d &joint_val,
													   const Eigen::Vector3d &joint_speed) {
  Eigen::Matrix3d j;
  GetJacobian(lnum, joint_val, j);

  Eigen::Vector3d b_vel_bd_2_ft = j * joint_speed;
  return b_vel_bd_2_ft;
}

Eigen::Vector3d LeggedRobotKinematics::GetFootPosition(int lnum,
													   const Eigen::Matrix3d &rotation,
													   const Eigen::Vector3d &joint_val) {
  Eigen::Vector3d w_pos_bd_2_ft = rotation * GetFootPosition(lnum, joint_val);
  return w_pos_bd_2_ft;
}

void LeggedRobotKinematics::GetFootPosition(const Eigen::Vector3d &body_pos,
											const Eigen::Matrix3d &rotation,
											const Eigen::VectorXd &joint_val,
											Eigen::MatrixXd &foot_position) {
  foot_position.setZero(3, 4);
  for (int i = 0; i < 4; i++)
    foot_position.col(i) = body_pos + GetFootPosition(i, rotation,
                                                      joint_val.segment(3 * i, 3));
}

Eigen::Vector3d LeggedRobotKinematics::GetFootVelocity(int lnum,
													   const Eigen::Matrix3d &rotation,
													   const Eigen::Vector3d &w_ang_vel,
													   const Eigen::Vector3d &joint_val,
													   const Eigen::Vector3d &joint_speed) {
  Eigen::Vector3d w_pos_bd_2_ft = rotation * GetFootPosition(lnum, joint_val);
  Eigen::Vector3d w_vel_bd_2_ft = w_ang_vel.cross(w_pos_bd_2_ft) + rotation * GetFootVelocity(lnum, joint_val, joint_speed);

  return w_vel_bd_2_ft;
}

void LeggedRobotKinematics::GetFootVelocity(const Eigen::Vector3d &body_vel,
											const Eigen::Matrix3d &rotation,
											const Eigen::Vector3d &w_ang_vel,
											const Eigen::VectorXd &joint_val,
											const Eigen::VectorXd &joint_speed,
											Eigen::MatrixXd &foot_velocity) {
  foot_velocity.setZero(3, 4);
  for (int i = 0; i < 4; i++)
    foot_velocity.col(i) = body_vel + GetFootVelocity(i, rotation, w_ang_vel,
                                                      joint_val.segment(3 * i, 3),
                                                      joint_speed.segment(3 * i, 3));
}

Eigen::Vector3d LeggedRobotKinematics::GetImu2FootPosition(int lnum,
														   const Eigen::Matrix3d &rotation,
														   const Eigen::Vector3d &joint_val,
														   const Eigen::Vector3d &b_pos_imu_2_bd) {
  Eigen::Vector3d w_pos_bd_2_gr = GetFootPosition(lnum, rotation, joint_val);
  Eigen::Vector3d w_pos_imu_2_bd = rotation * b_pos_imu_2_bd;

  Eigen::Vector3d w_pos_imu_2_gr = w_pos_imu_2_bd + w_pos_bd_2_gr;
  return w_pos_imu_2_gr;
}

Eigen::Vector3d LeggedRobotKinematics::GetImu2FootPosition(int lnum,
														   const Eigen::Vector3d &joint_val,
														   const Eigen::Vector3d &b_pos_imu_2_bd) {
  Eigen::Vector3d b_pos_bd_2_gr = GetFootPosition(lnum, joint_val);

  Eigen::Vector3d b_pos_imu_2_gr = b_pos_imu_2_bd + b_pos_bd_2_gr;
  return b_pos_imu_2_gr;
}

Eigen::Vector3d LeggedRobotKinematics::GetImu2FootPosition(int lnum,    // used in smoother
														   const Eigen::Vector3d &joint_val,
														   double foot_radius,
														   const Eigen::Vector3d &b_pos_imu_2_bd
														   ) {
  Eigen::Vector3d b_pos_bd_2_gr = GetFootPosition(lnum, joint_val, foot_radius);

  Eigen::Vector3d b_pos_imu_2_gr = b_pos_imu_2_bd + b_pos_bd_2_gr;
  return b_pos_imu_2_gr;
}

Eigen::Vector3d LeggedRobotKinematics::GetImu2FootVelocity(int lnum,
														   const Eigen::Matrix3d &rotation,
														   const Eigen::Vector3d &w_ang_vel,
														   const Eigen::Vector3d &joint_val,
														   const Eigen::Vector3d &joint_vel,
														   const Eigen::Vector3d &b_pos_imu_2_bd) {
  Eigen::Matrix3d j;
  GetJacobian(lnum, joint_val, j);

  Eigen::Vector3d b_vel_imu_2_ft = j * joint_vel;

  Eigen::Vector3d w_pos_imu_2_ft = GetImu2FootPosition(lnum, rotation, joint_val, b_pos_imu_2_bd);

  Eigen::Vector3d w_vel_imu_2_gr = w_ang_vel.cross(w_pos_imu_2_ft) + rotation * b_vel_imu_2_ft;
  return w_vel_imu_2_gr;
}

bool LeggedRobotKinematics::InverseKinematics(int lnum,
											  const Eigen::Vector3d &task_pos,
											  Eigen::Vector3d& joint_pos) {
    Eigen::Vector3d foot_pos = task_pos;

    int rightleft = Robot::LegSign[2*lnum+1];
    int fronthind = Robot::LegSign[2*lnum];
    double l_1 = Robot::RollToPitch * rightleft;
    double l_2 = Robot::ThighLength;
    double l_3 = Robot::CalfLength;

    double y = foot_pos[1];
    double z = foot_pos[2];

    double l = sqrt(y * y + z * z - l_1 * l_1);

    Eigen::Matrix2d mat;
    mat << l_1, -l, -l, -l_1;
    Eigen::Vector2d cos_rsin_r = mat.inverse() * Eigen::Vector2d(y, z);
    double r = -atan2(cos_rsin_r[1], cos_rsin_r[0]);

    double x = foot_pos[0];
    double y_prime = l;

    double hip_2_ft = sqrt(x * x + y_prime * y_prime);

    double cos_p_k = (l_2 * l_2 + l_3 * l_3 - hip_2_ft * hip_2_ft) / (2 * l_2 * l_3);
    double k = -M_PI + acos(cos_p_k);

    mat << -l_3 * sin(k), -l_2 - l_3 * cos(k), l_2 + l_3 * cos(k), -l_3 * sin(k);
    Eigen::Vector2d cos_p_sin_p = mat.inverse() * Eigen::Vector2d(x, y_prime);
    double p = atan2(cos_p_sin_p[1], cos_p_sin_p[0]);

    joint_pos << r, p, k;
    for (int i = 0; i < 3; i++)
      if (isnanf(joint_pos[i]))
        return false;

    return true;
}

void LeggedRobotKinematics::Imu2Bd(const Eigen::Matrix3d &rotation,
								   const Eigen::Vector3d &w_ang_vel,
								   const Eigen::Vector3d &w_pos_imu,
								   const Eigen::Vector3d &w_vel_imu,
								   Eigen::Vector3d &w_pos_bd,
								   Eigen::Vector3d &w_vel_bd,
								   const Eigen::Vector3d &b_pos_imu_2_bd) {
  Eigen::Vector3d w_pos_imu_2_bd = rotation * b_pos_imu_2_bd;
  w_pos_bd = w_pos_imu + w_pos_imu_2_bd;
  w_vel_bd = w_vel_imu + w_ang_vel.cross(w_pos_imu_2_bd);
}

