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



// Parameters
// #include "Models/estimator_parameters.hpp"
#include "iekf/estimator/RobotStateIEKF.hpp"

// CustomBasics

#include "iekf/InEKF/InEKF.h"
#include "iekf/utility/BasicFunctions.hpp"
// #include "Models/LeggedRobotKinematics.h"
#include "iekf/utility/EstimatorCommonStruct.hpp"
// #include "Models/RobotParameter.hpp"


class InvariantExtendedKalmanFilter {


public:

    const static int num_z = 30;
    MEAS_FORWARD_KINEMATICS forkin_meas_[2];

    InvariantExtendedKalmanFilter();
    ~InvariantExtendedKalmanFilter();

    //functions
    void Initialize(double _dt, EstimatorCovariances estimator_covariances, Eigen::Matrix<double, 16, 1> initial_condition);

    void new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i, const MEAS_FORWARD_KINEMATICS &forkin_set);

    void Propagate_Correct();
    std::vector<Eigen::Vector3d> Variable_Contact_Cov(int time);

    void send_states(ROBOT_STATES &state_);
    void sliding_window();

    void SAVE_onestep_Z1(int cnt);

    void Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i, const MEAS_FORWARD_KINEMATICS &forkin_set, ROBOT_STATES &state_);

    void DoSaveAll(std::string cov_info);


    int NUM_OF_TRASH_DATA = 1;

    //Necessary classes & variables
    EstimatorCommonStruct estimator_common_struct_;

    bool sliding_window_flag=false;
    int frame_count=0;
    int time_count=0;

    double dt = estimator_common_struct_.dt;
    Eigen::Matrix<double,3,1> gravity;




    //Call_FILE PARAMETER
    int gt_sd = 0;
    const static int MAX_FILE_COUNT =140000;
    double SensorData       [MAX_FILE_COUNT][30];
    double GroundTruth      [MAX_FILE_COUNT][27];
    int max_time = MAX_FILE_COUNT;
    int row_index = 0; // row index
    int column_index = 0; // column index


    //Inner variable
    inekf::InEKF filter;

    Eigen::Matrix<double,6,1> imu_measurement;
    Eigen::Matrix<double,6,1> imu_measurement_prev;
    // Estimated State Values Window Buffer
    Eigen::Matrix<double,3,1> Position_s[2];
    Eigen::Matrix3d Rotation_s[2];
    Eigen::Matrix<double,3,1> Velocity_s[2];
    Eigen::Matrix<double,12,1> d_v[2];

    Eigen::Matrix<double,3,1> Bias_Acc_s[2];
    Eigen::Matrix<double,3,1> Bias_Gyro_s[2];
    Eigen::Matrix<double, 6,1> Bias_s[2];


    // True Values and measurement Buffer
    Eigen::Matrix<bool, 4,1> HARD_CONTACT_t[2];
    Eigen::Matrix<bool, 4,1> CONTACT_t[2];
    Eigen::Matrix<bool, 4,1> SLIP_t[2];

    Eigen::Matrix<double, 3,1> IMU_Gyro[2];
    Eigen::Matrix<double, 3,1> IMU_Acc[2];
    Eigen::Matrix<double,12,1> ENCODER[2];
    Eigen::Matrix<double,12,1> ENCODERDOT[2];

    Eigen::Matrix3d lidar_rotation_;
    Eigen::Vector3d lidar_position_;



    //SAVE PARAMETER
    std::string estimator_info;
    std::string file_info;
    std::string initial_info;
    std::string time_size;
    std::string est_size;

    const static int SAVEMAX = 112;
    const static int SAVEMAXCNT = MAX_FILE_COUNT;
    double SAVE_BUFFER [SAVEMAX][SAVEMAXCNT];
    int SAVE_cnt = 0;

    int idx_TRUE_Rotation           = 0;
    int idx_TRUE_Velocity           = idx_TRUE_Rotation + 9;
    int idx_TRUE_Position           = idx_TRUE_Velocity + 3;
    int idx_TRUE_dv                  = idx_TRUE_Position + 3;
    int idx_TRUE_Bias_Gyro          = idx_TRUE_dv + 12;
    int idx_TRUE_Bias_Acc           = idx_TRUE_Bias_Gyro + 3;
    int idx_TRUE_Contact            = idx_TRUE_Bias_Acc + 3;
    int idx_TRUE_Slip               = idx_TRUE_Contact + 4;
    int idx_TRUE_Hard_Contact       = idx_TRUE_Slip+ 4;
    int idx_TRUE_rpy                = idx_TRUE_Hard_Contact + 4;

    int idx_ESTIMATED_Rotation      = idx_TRUE_rpy + 3;
    int idx_ESTIMATED_Velocity      = idx_ESTIMATED_Rotation + 9;
    int idx_ESTIMATED_Position      = idx_ESTIMATED_Velocity + 3;
    int idx_ESTIMATED_dv             = idx_ESTIMATED_Position + 3;
    int idx_ESTIMATED_Bias_Gyro     = idx_ESTIMATED_dv + 12;
    int idx_ESTIMATED_Bias_Acc      = idx_ESTIMATED_Bias_Gyro + 3;
    int idx_ESTIMATED_Contact       = idx_ESTIMATED_Bias_Acc + 3;
    int idx_ESTIMATED_Slip          = idx_ESTIMATED_Contact + 4;
    int idx_ESTIMATED_Hard_Contact  = idx_ESTIMATED_Slip + 4;
    int idx_ESTIMATED_rpy           = idx_ESTIMATED_Hard_Contact + 4;

    int idx_final_state_covariance = idx_ESTIMATED_rpy+3;
    int idx_end = idx_final_state_covariance+12;




    //Parameters
    bool textfile_flag = false;

    bool slip_rejection_mode = false;
    double slip_threshold=0.0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;


    //not used
    int Max_Iteration = 100;
    double Optimization_Epsilon = 1e-3;


private:


};
