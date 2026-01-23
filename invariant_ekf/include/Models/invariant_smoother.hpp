#ifndef INVARIANT_SMOOTHER_HPP
#define INVARIANT_SMOOTHER_HPP

#pragma once

// Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <malloc.h>

// Necessaries
#include "Models/invariant_factors.hpp"

class InvariantSmoother
{
public:

    InvariantSmoother();    // ROBOT_STATES state 	
    ~InvariantSmoother();

    const static int num_z_imu = 6;
    const static int num_z_encoder = 12;
    const static int num_z_encoderdot = 12;
    const static int num_z = 30;

    int NUM_OF_TRASH_DATA = 1;

    bool hasnan{false};
    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_zeta_Xi;
    Eigen::Matrix<double, Eigen::Dynamic, 1> perturbation;

    Eigen::Matrix<double, num_z * (WINDOW_SIZE + 1), 1> estimation_z;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> marginalized_H;
    Eigen::Matrix<double, Eigen::Dynamic, 1> marginalized_b;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> marginalized_hessian;
    Eigen::Matrix<double, Eigen::Dynamic, 1> marginalized_gradient;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> marginalized_hessian_bef;
    Eigen::Matrix<double, Eigen::Dynamic, 1> marginalized_gradient_bef;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> hessian_s;

    // functions
    void initialize
    (
        double dt_, EstimatorCovariances estimator_covariances,
        Eigen::Matrix<double, 16, 1> &initial_condition
    );

    void sensor_data_buffering
    (
        Eigen::Matrix<double, num_z, 1> sensor_i
    );

    Eigen::Matrix<double, 3, 1> x_lidar_i;
    Eigen::Matrix<double,3,3> R_lidar_i;
    bool lidar_in_i;

    Eigen::Matrix<double, 3, 1> x_gps_i;
    bool gps_in_i;

    void new_measurement
    (
        Eigen::Matrix<double, num_z, 1> sensor_i, 
        Eigen::Matrix<bool, 4, 1> contact_i,
        Eigen::Matrix<double,3,1> x_lidar_i,
        Eigen::Matrix<double,3,3> R_lidar_i,
        Eigen::Matrix<double,3,1> sqrt_lidar_i,
        bool lidar_in_i,
        Eigen::Matrix<double,3,1> x_gps_i,
        Eigen::Matrix<double,3,1> sqrt_gps_i,
        bool gps_in_i
    );

    void optimization_solve();
    void retract_manifold(int start_frame);
    void update_dv(int start_frame);
    void sliding_window();

    void send_states(ROBOT_STATES &state_);

    void save_one_step(int cnt);
    void save_all(const std::string& cov_info);

    void one_step
    (
        Eigen::Matrix<double, num_z, 1> sensor_i, Eigen::Matrix<bool, 4, 1> contact_i, ROBOT_STATES &state_,
        Eigen::Matrix<double, 3, 1> x_lid_i, Eigen::Matrix<double,3,3> R_lid_i, Eigen::Matrix<double, 3, 1> sqrt_lid_i, bool lid_in_i,Eigen::Matrix<double, 3, 1> x_gps_i,	Eigen::Matrix<double, 3, 1> sqrt_gps_i, bool gps_in_i
    );

    // necessary classes and variables
    EstimatorCommonStruct estimator_common_struct_;

    bool sliding_window_flag{false};
    bool marginalization_flag{false};

    int frame_count = 0;
    int64_t count_temp{0};
    int time_count{0};

    int leg_no = 4;

    double dt = 0.001;      // 0.005;

    Eigen::Matrix<double,3,1> gravity;

    // estimated state values and window buffer
    ROBOT_STATES rs[WINDOW_SIZE + 1];
    ROBOT_STATES rs_prior;

    std::vector<double> contact_score_array[WINDOW_SIZE + 1];
    factor_info fac_info[WINDOW_SIZE + 1];
    std::vector<Eigen::Matrix<double, num_z, 1>> z_buffer;
    Eigen::Matrix<int, 4, 1> contact_phase_count;

    // save parameters
    const static int MAX_FILE_COUNT = 140000;
    const static int SAVEMAX = 115;
    const static int SAVEMAXCNT = MAX_FILE_COUNT;
    double SAVE_BUFFER[SAVEMAX][SAVEMAXCNT];
    int SAVE_cnt = 0;

    // parameters    
    int iteration_number{0};
    int total_backppgn_number{0};
    double time_per_step;

    std::string estimator_info;
    std::string file_info;
    std::string initial_info;
    std::string time_size;
    std::string est_size;

    int idx_TRUE_Rotation     = 0;                                      // 0
    int idx_TRUE_Velocity     = idx_TRUE_Rotation + 9;                  // 9
    int idx_TRUE_Position     = idx_TRUE_Velocity + 3;                  // 12
    int idx_TRUE_dv           = idx_TRUE_Position + 3;                  // 15
    int idx_TRUE_Bias_Gyro    = idx_TRUE_dv + 12;                       // 27
    int idx_TRUE_Bias_Acc     = idx_TRUE_Bias_Gyro + 3;                 // 30
    int idx_TRUE_Contact      = idx_TRUE_Bias_Acc + 3;                  // 33
    int idx_TRUE_Slip         = idx_TRUE_Contact + 4;                   // 37
    int idx_TRUE_Hard_Contact = idx_TRUE_Slip + 4;                      // 41
    int idx_TRUE_rpy          = idx_TRUE_Hard_Contact + 4;              // 45

    int idx_ESTIMATED_Rotation     = idx_TRUE_rpy + 3;                  // 48
    int idx_ESTIMATED_Velocity     = idx_ESTIMATED_Rotation + 9;        // 57
    int idx_ESTIMATED_Position     = idx_ESTIMATED_Velocity + 3;        // 60
    int idx_ESTIMATED_dv           = idx_ESTIMATED_Position + 3;        // 63
    int idx_ESTIMATED_Bias_Gyro    = idx_ESTIMATED_dv + 12;             // 75
    int idx_ESTIMATED_Bias_Acc     = idx_ESTIMATED_Bias_Gyro + 3;       // 78
    int idx_ESTIMATED_Contact      = idx_ESTIMATED_Bias_Acc + 3;        // 81
    int idx_ESTIMATED_Slip         = idx_ESTIMATED_Contact + 4;         // 85
    int idx_ESTIMATED_Hard_Contact = idx_ESTIMATED_Slip + 4;            // 89
    int idx_ESTIMATED_rpy          = idx_ESTIMATED_Hard_Contact + 4;    // 93

    int idx_iteration_No  = idx_ESTIMATED_rpy + 3;                      // 96
    int idx_backppgn_No   = idx_iteration_No + 1;                       // 97
    int idx_time_per_step = idx_backppgn_No + 1;                        // 98
    int idx_cost          = idx_time_per_step + 1;                      // 99

    int idx_final_state_covariance = idx_cost + 4;
    int idx_end = idx_final_state_covariance + 12;

    // parameters setting
    bool retract_all_flag{false};
    int max_iteration = 100;
    double optimization_epsilon = 1e-3;
    int max_backpropagate_num = 3;

    //To turn off backpropagation, set num=0
    double backppgn_rate = 0.9;
    bool slip_rejection_mode{false};
    double slip_threshold{0};

    double long_term_v_threshold{0};
    double long_term_a_threshold{0};

    bool variable_contact_cov_mode{false};

    bool preintegration_mode{false};
    double cov_amplifier = 100;
    double lidar_covariance_amplifier = 1.0;

    // others
    // const double ALPHA{0};

    std::string convergence_cond;
    std::string mode;
    Eigen::Vector4d initial_quaternion;

    int prop_para0_size;
    int prop_para1_size;
    int prop_res_size;
    Eigen::MatrixXd Mi, Mj;

    Eigen::MatrixXd sqrt_info_prop_primitive;

    std::vector<Eigen::Vector3d> contact_cov_array_temp;
    Eigen::Matrix3d sqrt_contact_info_temp;

    Eigen::Vector3d enc;

    Eigen::Vector3d imu_gyro_prev, imu_acc_prev, imu_gyro;
    Eigen::Matrix<double, 12, 1> encoder, encoder_dot;

    double cost_before{0}, cost{0};
    double cost_temp, t;
    int backpropagate_count{0};

    Eigen::Matrix<double, Eigen::Dynamic, 1> gradient;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> hessian;

    int max_time = MAX_FILE_COUNT;

    double SensorData[MAX_FILE_COUNT][num_z];
    double GroundTruth[MAX_FILE_COUNT][27]; // maybe unused

    // Eigen::Matrix<double, -1, 1> delta_zeta_Xi_temp;
    Eigen::Matrix<double,Eigen::Dynamic, 1> delta_zeta_Xi_temp;

    int m{0}, n{0};

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HMM;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HMR;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HRR;

    Eigen::VectorXd s_vec, s_vec_inv, s_vec_sqrt, s_vec_inv_sqrt;

    Eigen::MatrixXd X_s;

    Eigen::Vector3d w_pos_imu2bd;
    Eigen::Vector3d w_gyro;

    Eigen::Vector3d temp_eul;

    ROBOT_STATES state_;

    // ROBOT_STATES getState();


private:
    
    double eps = 1e-8;

};

#endif