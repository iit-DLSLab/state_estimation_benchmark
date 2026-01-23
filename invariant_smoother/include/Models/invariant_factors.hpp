#ifndef INVARIANT_FACTORS_HPP
#define INVARIANT_FACTORS_HPP

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
#include "Models/robot_state.hpp"
#include "Models/BasicFunctions.hpp"
#include "Models/smoother_struct.hpp"
#include "Models/estimator_parameters.hpp"
#include "Models/LeggedRobotKinematics.h"


class InvFactors 
{
public:

    InvFactors(); 	
    ~InvFactors();

    bool hasnan=false;

    ROBOT_STATES *rs_temp_;
    ROBOT_STATES *rs_temp_lidar_;
    factor_info *fac_info_;

    Eigen::Matrix<double,3,1> gravity;
    double dt = 0.001;

    EstimatorCommonStruct estimator_common_struct_;
    
    int frame_count;
    int num_z = 30;

    // initialization
    void batch_initialize
    (   
        ROBOT_STATES *rs, const ROBOT_STATES &rs_prior,
        const Eigen::Matrix<double,Eigen::Dynamic,1> &estimation_z,
        factor_info *fac_info,
        const EstimatorCommonStruct &estimator_common_struct
    );

    // prior
    Eigen::Matrix<double, -1, -1> X_prior;
    Eigen::Matrix<double,6,1> bias_prior;
    Eigen::Matrix<double,9,9> sqrt_info_prior; // SQRT_INFO_Prior
    Eigen::Matrix<double,6,6> sqrt_info_prior_bias;
    Eigen::Matrix<double,-1,-1> X0_bar;
    Eigen::Matrix<double,6,1> bias0_bar;
    Eigen::Matrix<double,5,5> Xhat; // current estimate
    Eigen::MatrixXd X_prior_inv;
    Eigen::Matrix<double,9,1> X_prior_hat_inv_ret;
    Eigen::Matrix<double,9,9> jac_left_inv;
    Eigen::Matrix<double,9,1> residual_prior; 
    Eigen::Matrix<double,9,9> partial_xi0;
    Eigen::Matrix<double,6,1> residual_prior_bias;
    Eigen::Matrix<double,6,6> partial_bias0;

    void invariant_prior_rvp_factor
    (
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );

    void invariant_prior_bias_factor
    (
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );


    // propagation
    Eigen::MatrixXd A;
    Eigen::MatrixXd A_lidar;
    Eigen::Vector3d d;
    Eigen::MatrixXd Adj_inv_aug;
    Eigen::Vector3d d_adj;
    Eigen::MatrixXd sqrt_info_prop;
    Eigen::VectorXd residual_prop;
    Eigen::VectorXd residual_lidar_prop;
    Eigen::MatrixXd f_X_i;
    Eigen::MatrixXd f_X_lidar_i;
    Eigen::Vector3d imu_gyro_i, imu_acc_i;
    Eigen::Vector3d integrated_acc, integrated_gyro, integrated_acc2, integrated_vel;
    Eigen::Vector3d integ_comp_acc, integ_comp_gyro, integ_comp_acc2, integ_comp_vel;
    int buffer_size{0};
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0,p0,bg0,ba0;
    Eigen::MatrixXd Xj_bar, Xj_bar_inv;
    Eigen::Matrix<double,-1,1> delta;
    Eigen::MatrixXd I;
    Eigen::MatrixXd IAdt;
    Eigen::MatrixXd partial_i, partial_j;

    void invariant_propagation_factor
    (
        int i, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost, 
        bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
    );

    void invariant_lidar_propagation_factor
    (
        int i, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost, 
        Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
    );

    // observation
    Eigen::Matrix3d sqrt_info_meas;
    Eigen::Vector3d residual_meas;
    Eigen::Vector3d residual_t;
    Eigen::MatrixXd partial_xi_i;
    // Eigen::Vector3d position_residual;
    Eigen::Matrix3d sqrt_info_lidar;

    void invariant_measurement_factor
    (   
        int i, int leg_num, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );

    void invariant_lidar_observation_cost
    (
        int j, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );

    void invariant_lidar_measurement_factor
    (
        int i, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );

    void invariant_gps_measurement_factor
    (
        int i, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );

    
    // marginalization & preintegration
    bool marginalization_flag_{false};
    bool preintegration_mode_{false};
    double cost_unchanged;

    // gradient, hessian
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> hessian_marg_factor;
    Eigen::Matrix<double,Eigen::Dynamic,1> gradient_marg_factor;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> H_matrix;
    Eigen::Matrix<double,Eigen::Dynamic,1> b_vec;

    void marg_initialize
    (
        ROBOT_STATES *rs, const Eigen::Matrix<double,Eigen::Dynamic,1> &estimation_z,
        const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> H,
        const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> hessian_marg,
        const Eigen::Matrix<double,Eigen::Dynamic,1> b,
        Eigen::Matrix<double,Eigen::Dynamic,1> gradient_marg,
        factor_info *fac_info, const EstimatorCommonStruct &estimator_common_struct
    );

    // long_term_stationary_foot_factor    
    Eigen::Matrix3d sqrt_info_long;
    Eigen::Vector3d residual_long;
    Eigen::MatrixXd partial_xi_j;

    void long_term_stationary_foot_factor
    (
        int start, int end, int xyz, int leg_num,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
    );

    // marg_update_nget_grad_hess_cost_debug
    std::vector<Eigen::Vector3d> contact_cov_temp;
    Eigen::Matrix3d sqrt_contact_info_temp;
    Eigen::Matrix<double, -1, 1> Xi0;
    Eigen::Matrix<double, -1, 1> bias_Xbar_vee_inv_ret;
    Eigen::Matrix<double, -1, 1> residual_marg_debug;  
    double temp_cost;
    double cond;
    double v_threshold{0};
    double a_threshold{0};
    int start{0};
    int end{0};
    int contact_count{0};
    double dv_mag_i{0};
    double da_mag_i{0};


    // batch_update_nget_grad_hess_cost
    std::vector<Eigen::Vector3d> contact_cov_array_temp;

    void batch_update_nget_grad_hess_cost
    (
        bool is_for_marginalization, ROBOT_STATES *rs, 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost,
        bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
    );

    // marg_update_nget_grad_hess_cost
    Eigen::Matrix<double,-1,-1> residual_update;

    void marg_update_nget_grad_hess_cost
    (
        bool is_for_marginalization, ROBOT_STATES *rs,
        Eigen::Matrix<double, -1, 1> zeta_xi,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost,
        bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
    );
    
    void marg_update_nget_grad_hess_cost_debug
    (
        bool is_for_marginalization, ROBOT_STATES *rs,
        Eigen::Matrix<double, -1, 1> zeta_xi,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost,
        bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i											
    );

}; // end class InvFactors

#endif