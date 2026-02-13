// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.


#pragma once

// Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <unordered_map>

#include <set>

// Parameters
// #include "invariant_smoother/estimator/Estimator_Parameters.hpp"


typedef struct _ROBOT_STATES_ {


  Eigen::Matrix<double,3,3> Rotation;
  Eigen::Matrix<double,3,1> Velocity;
  Eigen::Matrix<double,3,1> Position;

  Eigen::Matrix<double, 12, 1> d;
  Eigen::Matrix<double, 12, 1> d_v;

  bool GPS_In = false;

  Eigen::Matrix<double,3,1> Bias_Gyro;
  Eigen::Matrix<double,3,1> Bias_Acc;

  Eigen::Matrix<bool, 4, 1> Contact;
  Eigen::Matrix<bool, 4, 1> Hard_Contact;
  Eigen::Matrix<bool, 4, 1> Slip;

  int contact_leg_num;

  int state_size;
  int state_idx;
  int para_size;
  int para_idx;

} ROBOT_STATES;

class kinematics_info {

 public:


  Eigen::Matrix<double,3,1> fk_kin;
  Eigen::Matrix<double,3,3> meas_primitive_sqrt_info;

  int leg_no;
  int leg_num_in_state;

  kinematics_info(int _leg_no) : leg_no(_leg_no) {}

  bool operator<(const kinematics_info &t) const {
    return leg_no < t.leg_no;
  }

};

class factor_info {

 public:



  const static int num_z = 30;

  Eigen::Matrix<double, num_z, 1> Z;
  std::set<kinematics_info> leg_info;

  Eigen::Matrix<double, 3, 1> X_GPS;
  Eigen::Matrix<double, 3, 1> sqrt_GPS;

  std::vector<Eigen::Matrix<double, num_z, 1>> z_buffer_;
//
//  Eigen::Vector3d Integrated_Gyro;
//  Eigen::Vector3d Integrated_Acc;

  Eigen::MatrixXd Mi;
  Eigen::MatrixXd Mj;

  Eigen::MatrixXd prop_primitive_sqrt_info;

  int shared_contact;

  //propagation
  int prop_para0_size;
  int prop_para1_size;
  int prop_res_size;


  //measurement

  int meas_para_size;

};

typedef struct _ROBOT_STATES_PSEUDO_ {

  Eigen::Matrix<double,3,3> Rotation;
  Eigen::Matrix<double,3,1> Velocity;
  Eigen::Matrix<double,3,1> Position;

  Eigen::Matrix<double, 12, 1> d_v;

  Eigen::Matrix<double,3,1> Bias_Gyro;
  Eigen::Matrix<double,3,1> Bias_Acc;

  Eigen::Matrix<bool, 4, 1> Contact;
  Eigen::Matrix<bool, 4, 1> Hard_Contact;
  Eigen::Matrix<bool, 4, 1> Slip;

} ROBOT_STATES_PSEUDO;

class factor_info_pseudo {

 public:



  const static int num_z = 30;
  Eigen::Matrix<double, num_z, 1> Z;

  std::set<kinematics_info> leg_info;

  Eigen::Matrix<double, 15, 15> prop_primitive_sqrt_info;

  std::vector<double> contact_cov_array;

};




typedef struct _MEAS_FORWARD_KINEMATICS_ {

std::vector<Eigen::Vector3d> forkin_position;
std::vector<Eigen::Matrix3d> forkin_jacobian;

} MEAS_FORWARD_KINEMATICS;