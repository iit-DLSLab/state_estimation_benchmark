// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.


#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include "Models/RobotParameter.hpp"

class LeggedRobotKinematics {
 public:

  static Eigen::Matrix3d GetJacobian(int lnum,
									 const Eigen::Vector3d &joint_val);

  static Eigen::Vector3d GetFootPosition(int lnum,
										 const Eigen::Matrix3d &rotation,
										 const Eigen::Vector3d &joint_val);

  static Eigen::Matrix<double, 3, 4> GetPelvisKneeFootPosition(int lnum, const Eigen::Matrix3d &rotation,
															   const Eigen::Vector3d &joint_val);

  static Eigen::Matrix<double, 3, 4> GetPelvisKneeFootPosition(int lnum,
															   const Eigen::Vector3d &joint_val);

  static Eigen::Vector3d GetImu2FootPosition(int lnum,
											 const Eigen::Vector3d &joint_val,
											 const Eigen::Vector3d &b_pos_imu_2_bd);

  static Eigen::Vector3d GetFootVelocity(int lnum,
										 const Eigen::Matrix3d &rotation,
										 const Eigen::Vector3d &w_ang_vel,
										 const Eigen::Vector3d &joint_val,
										 const Eigen::Vector3d &joint_speed);

  static void GetFootPosition(const Eigen::Vector3d &body_pos,
							  const Eigen::Matrix3d &rotation,
							  const Eigen::VectorXd &joint_val,
							  Eigen::MatrixXd &foot_position);

  static void GetFootVelocity(const Eigen::Vector3d &body_vel,
							  const Eigen::Matrix3d &rotation,
							  const Eigen::Vector3d &w_ang_vel,
							  const Eigen::VectorXd &joint_val,
							  const Eigen::VectorXd &joint_speed,
							  Eigen::MatrixXd &foot_velocity);

  static Eigen::Vector3d GetImu2FootPosition(int lnum,
											 const Eigen::Matrix3d &rotation,
											 const Eigen::Vector3d &joint_val,
											 const Eigen::Vector3d &b_pos_imu_2_bd);

  static Eigen::Vector3d GetImu2FootVelocity(int lnum,
											 const Eigen::Matrix3d &rotation,
											 const Eigen::Vector3d &w_ang_vel,
											 const Eigen::Vector3d &joint_val,
											 const Eigen::Vector3d &joint_vel,
											 const Eigen::Vector3d &b_pos_imu_2_bd);

  static void Imu2Bd(const Eigen::Matrix3d &rotation,
					 const Eigen::Vector3d &w_ang_vel,
					 const Eigen::Vector3d &w_pos_imu,
					 const Eigen::Vector3d &w_vel_imu,
					 Eigen::Vector3d &w_pos_bd,
					 Eigen::Vector3d &w_vel_bd,
					 const Eigen::Vector3d &b_pos_imu_2_bd);

  static void GetJacobian(int lnum,
						  const Eigen::Vector3d &joint_val,
						  Eigen::Matrix3d &j);

  static void GetJacobianDerivative(int lnum,
									const Eigen::Vector3d &joint_pos,
									const Eigen::Vector3d &joint_vel,
									Eigen::Matrix3d &d_j);

  static Eigen::Vector3d GetHiptoFootPosition(int lnum,
											  const Eigen::Vector3d &joint_val);

  static Eigen::Vector3d GetBodyToHipPosition(int lnum);

  static Eigen::Vector3d GetFootPosition(int lnum,
										 const Eigen::Vector3d &joint_val);

  static Eigen::Vector3d GetFootVelocity(int lnum,
										 const Eigen::Vector3d &joint_val,
										 const Eigen::Vector3d &joint_speed);

  static bool InverseKinematics(int lnum,
								const Eigen::Vector3d &task_pos,
								Eigen::Vector3d &joint_pos);

  void GetJacobian(int lnum,
				   const Eigen::Vector3d &joint_val,
				   double foot_radius,
				   Eigen::Matrix3d &j);

  static Eigen::Matrix3d GetJacobian(int lnum,
									 const Eigen::Vector3d &joint_val,
									 double foot_radius);

  static void GetJacobianDerivative(int lnum,
									const Eigen::Vector3d &joint_pos,
									const Eigen::Vector3d &joint_vel,
									double foot_radius,
									Eigen::Matrix3d &d_j);
  static Eigen::Vector3d GetHiptoFootPosition(int lnum, const Eigen::Vector3d &joint_val, double foot_radius);
  static Eigen::Vector3d GetFootPosition(int lnum, const Eigen::Vector3d &joint_val, double foot_radius);
  static Eigen::Matrix<double, 3, 4> GetPelvisKneeFootPosition(int lnum,
															   const Eigen::Vector3d &joint_val,
															   double foot_radius);

  static Eigen::Vector3d GetImu2FootPosition(int lnum,
											 const Eigen::Vector3d &joint_val,
											 double foot_radius,
											 const Eigen::Vector3d &b_pos_imu_2_bd);

};  // class LeggedRobotKinematics
