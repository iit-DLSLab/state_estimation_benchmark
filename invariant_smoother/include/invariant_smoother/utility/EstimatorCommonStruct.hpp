// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.

//

#pragma once
#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "invariant_smoother/utility/BasicFunctions.hpp"

const int leg_num = 4;

// Robot Model Class for KAIST Hound Robot
// Modifying the kinematics part(e.g. link length) will be needed for customized application of this Class

typedef struct _ESTIMATOR_COVARIANCES_
{
  Eigen::Vector3d cov_gyro_diagonal;
  Eigen::Vector3d cov_acc_diagonal;
  Eigen::Vector3d cov_slip_diagonal;
  Eigen::Vector3d cov_contact_diagonal;
  Eigen::Vector3d cov_enc_diagonal;
  Eigen::Vector3d cov_bias_gyro_diagonal;
  Eigen::Vector3d cov_bias_acc_diagonal;
  Eigen::Vector3d cov_prior_orientation_diagonal;
  Eigen::Vector3d cov_prior_velocity_diagonal;
  Eigen::Vector3d cov_prior_position_diagonal;
  Eigen::Vector3d cov_prior_bias_gyro_diagonal;
  Eigen::Vector3d cov_prior_bias_acc_diagonal;

  Eigen::Matrix<double,3,3> Covariance_Gyro;
  Eigen::Matrix<double,3,3> Covariance_Acc;
  Eigen::Matrix<double,3,3> Covariance_Contact;
  Eigen::Matrix<double,3,3> Covariance_Slip;

  Eigen::Matrix<double,3,3> Covariance_Encoder;
  Eigen::Matrix<double,3,3> Covariance_Bias_Gyro;
  Eigen::Matrix<double,3,3> Covariance_Bias_Acc;

  Eigen::Matrix<double,3,3> Covariance_Prior_Orientation;
  Eigen::Matrix<double,3,3> Covariance_Prior_Velocity;
  Eigen::Matrix<double,3,3> Covariance_Prior_Position;
  Eigen::Matrix<double,3,3> Covariance_Prior_Bias_Gyro;
  Eigen::Matrix<double,3,3> Covariance_Prior_Bias_Acc;

  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Gyro;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Acc ;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Contact;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Slip;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Encoder;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Gyro;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Acc;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Orientation;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Velocity;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Position;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Gyro;
  Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Acc;

  double contact_grf_threshold_ = 80.;
  double contact_grf_threshold_vel_const_ = 0.;

}EstimatorCovariances;

class EstimatorCommonStruct {

public:
    EstimatorCommonStruct();
    ~EstimatorCommonStruct();

    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Define Variable         /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    /*
    leg order : RH(Right Hind), LH(Left Hind), RF(Right Front), LF(Left Front)
    joint coordinate : ex, ey, ey

    RHHR = Right Hind Hip Roll
    RHHP = Right Hind Hip Pitch
    RHKP = Right Hind Knee Pitch
    RHF  = Right Hind Foot

    LHHR = Left Hind Hip Roll
    LHHP = Left Hind Hip Pitch
    LHKP = Left Hind Knee Pitch
    LHF  = Left Hind Foot

    RFHR = Right Front Hip Roll
    RFHP = Right Front Hip Pitch
    RFKP = Right Front Knee Pitch
    RFF  = Right Front Foot

    LFHR = Left Front Hip Roll
    LFHP = Left Front Hip Pitch
    LFKP = Left Front Knee Pitch
    LFF  = Left Front Foot
    */

    int leg_no=4;

    Eigen::Vector3d IMU2BD;


    double dt = 0.001;

  EstimatorCovariances estimator_covariances_;

    void Covariance_Reset();
    std::vector<Eigen::Vector3d> Variable_Contact_Cov(Eigen::Matrix<bool,-1,1> Contact, Eigen::Matrix<double,-1,1> dv);
      bool variable_contact_cov_mode=false;
      double cov_amplifier=10.0;
	  double gps_covariance_amplifier = 1.0;
      bool slip_rejection_mode=false;
      double slip_threshold =0 ;

    double long_term_v_threshold;
    double long_term_a_threshold;


    ///////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////     Define Function     /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

private:

};

