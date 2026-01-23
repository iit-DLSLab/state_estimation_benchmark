// Copyright (c) 2023-2024. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2024.



#pragma once
// Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>

// // Necessaries
// #include "estimator/Estimator_Parameters.hpp" ---
// #include "estimator/RobotState_Smoother.hpp" ---
// #include "BasicFunctions.hpp" ---
// #include "utility/EstimatorCommonStruct.hpp" ---
// #include "LeggedRobotKinematics.h" ---
// #include "RobotParameter.hpp"

// Necessaries
#include "Models/robot_state.hpp"
#include "Models/BasicFunctions.hpp"
#include "Models/smoother_struct.hpp"
#include "Models/estimator_parameters.hpp"
#include "Models/LeggedRobotKinematics.h"
#include "Models/RobotParameter.hpp"

class Inv_Factors {

 public:

  bool hasnan=false;

  ROBOT_STATES *RS_temp;
  factor_info *fac_info;

  Eigen::Matrix<double,3,1> gravity;
  double dt;

  EstimatorCommonStruct estimator_common_struct_;
//  RobotModel estimator_common_struct_;
  int frame_count;

  int num_z = 30;



  //prior
  Eigen::Matrix<double, -1, -1> X_Prior;
  Eigen::Matrix<double, 6, 1> Bias_Prior;

  Eigen::Matrix<double, 9, 9> SQRT_INFO_Prior;
  Eigen::Matrix<double, 6, 6> SQRT_INFO_Prior_BIAS;

  Eigen::Matrix<double, -1, -1> X_0_bar;
  Eigen::Matrix<double, 6, 1> Bias_0_bar;

  //gra, hess
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Hessian_Marg;
  Eigen::Matrix<double, Eigen::Dynamic, 1> gradient_Marg;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H;
  Eigen::Matrix<double, Eigen::Dynamic, 1> b;
  double cost_unchanged;

  ~Inv_Factors() {
//        delete[] RS_temp;
//        delete[] fac_info;
  }

  bool marginalization_flag = false;
  bool preintegration_mode_ = false;

  void Batch_Initialize(ROBOT_STATES *_RS, const ROBOT_STATES &RS_Prior,
                        const Eigen::Matrix<double, Eigen::Dynamic, 1> &_Estimation_Z,
                        factor_info *_fac_info,
						const EstimatorCommonStruct& estimator_common_struct);

  void Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES *_RS,
                                             Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                             Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient,
                                             double &cost);

  void Marg_Initialize(ROBOT_STATES *_RS,
                       const Eigen::Matrix<double, Eigen::Dynamic, 1> &_Estimation_Z,
                       const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _H,
					   const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Marg_Hessian,
                       const Eigen::Matrix<double, Eigen::Dynamic, 1> _b,
					   const Eigen::Matrix<double, Eigen::Dynamic, 1> Marg_Gradient,
                       factor_info *_fac_info,
					   const EstimatorCommonStruct& estimator_common_struct);

  void Marg_Update_n_Get_Gradient_Hess_Cost_Debug(bool is_for_marginalization,
											ROBOT_STATES *_RS,
											Eigen::Matrix<double, -1, 1> Zeta_Xi,
											Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
											Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient,
											double &cost);

  void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization,
                                            ROBOT_STATES *_RS,
                                            Eigen::Matrix<double, -1, 1> Zeta_Xi,
                                            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                            Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient,
                                            double &cost);

  void Invariant_Propagation_Factor(int j,
                                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

  void Invariant_LVM_Measurement_Factor(int j, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
																Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

  void Invariant_Measurement_Factor(int i, int leg_num,
                                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

  void Invariant_GPS_Measurement_Factor(int i,
                                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

  void Long_Term_Stationary_Foot_Factor(int start, int end, int xyz, int leg_num,
                                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                        Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

  void Invariant_Prior_RVP_Factor(
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
      Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

  void Invariant_Prior_Bias_Factor(
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
      Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost);

 private:

};

class InvariantSmoother {

 public:

  const static int num_z_imu = 6;
  const static int num_z_encoder = 12;
  const static int num_z_encoderdot = 12;
  const static int num_z = 30;

  int NUM_OF_TRASH_DATA = 1;

  Eigen::MatrixXd readMatrix(const char *filename);

  bool hasnan=false;

  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_Zeta_Xi; // delta_bias_gyro, delta_bias_acc, delta_orientation, delta_velocity, delta_position, delta_foot_position
  Eigen::Matrix<double, Eigen::Dynamic, 1> perturbation; // delta_bias_gyro, delta_bias_acc, delta_orientation, delta_velocity, delta_position, delta_foot_position

  Eigen::Matrix<double, num_z * (WINDOW_SIZE + 1), 1> Estimation_Z;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Marginalized_H;
  Eigen::Matrix<double, Eigen::Dynamic, 1> Marginalized_b;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Marginalized_Hessian;
  Eigen::Matrix<double, Eigen::Dynamic, 1> Marginalized_Gradient;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Marginalized_Hessian_bef;
  Eigen::Matrix<double, Eigen::Dynamic, 1> Marginalized_Gradient_bef;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Hessian_S;
  //functions
  void Call_File(double _dt, const std::string& file_name);
  //void Initialize(double _dt, Eigen::Matrix<double,23,1> &COV_IC_setting);

  void Initialize(double _dt,
				  EstimatorCovariances estimator_covariances,
                  Eigen::Matrix<double, 16, 1> &initial_condition);

  void SensorDataBuffering(Eigen::Matrix<double, num_z, 1> Sensor_i);

  void new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i);
  void new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i,
                       Eigen::Matrix<bool, 4, 1> Contact_i,
                       Eigen::Matrix<double, 3, 1> X_GPS_i,
                       Eigen::Matrix<double, 3, 1> sqrt_GPS_i,
                       bool GPS_In_i);

  void new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i,
					   Eigen::Matrix<bool, 4, 1> Contact_i,
					   Eigen::Matrix<double, 3, 1> X_GPS_i,
					   Eigen::Matrix<double, 3, 1> sqrt_GPS_i,
					   bool GPS_In_i,
					   Eigen::Matrix<double, 3, 1> X_LVM_i,
					   Eigen::Matrix<double, 3, 1> sqrt_LVM_i,
					   bool LVM_In_i);

  void Optimization_Solve();
  void retract_manifold(int start_frame);
  void update_dv(int start_frame);
  void sliding_window();

  void send_states(ROBOT_STATES &state_);

  void SaveOneStep(int cnt);
  void DoSaveAll(const std::string& cov_info);

  void Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i, ROBOT_STATES &state_);
  void Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i,
               Eigen::Matrix<bool, 4, 1> Contact_i,
               ROBOT_STATES &state_,
               Eigen::Matrix<double, 3, 1> X_GPS_i,
               Eigen::Matrix<double, 3, 1> sqrt_GPS_i,
               bool GPS_In_i);

  void Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i,
			   Eigen::Matrix<bool, 4, 1> Contact_i,
			   ROBOT_STATES &state_,
			   Eigen::Matrix<double, 3, 1> X_GPS_i,
			   Eigen::Matrix<double, 3, 1> sqrt_GPS_i,
			   bool GPS_In_i,
			   Eigen::Matrix<double, 3, 1> X_LVM_i,
			   Eigen::Matrix<double, 3, 1> sqrt_LVM_i,
			   bool LVM_In_i);

  //Necessary classes & variables
  EstimatorCommonStruct estimator_common_struct_;

  bool sliding_window_flag = false;
  bool marginalization_flag = false;

  int frame_count = 0;
  int64_t temp_count = 0;
  int time_count = 0;

  int leg_no=4;

  double dt = 0.005;
  Eigen::Matrix<double,3,1> gravity;

  // Estimated State Values Window Buffer
  ROBOT_STATES RS[WINDOW_SIZE + 1];
  ROBOT_STATES RS_Pri;

  std::vector<double> contact_score_array[WINDOW_SIZE + 1];

  factor_info fac_info[WINDOW_SIZE + 1];

  std::vector<Eigen::Matrix<double, num_z, 1>> z_buffer_;

  Eigen::Matrix<int, 4, 1> contact_phase_count;

  //Call_FILE PARAMETER
  int gt_sd = 0;
  int row_index = 0; // row index
  int column_index = 0; // column index

  const static int MAX_FILE_COUNT = 140000;
  int max_time = MAX_FILE_COUNT;

  double SensorData[MAX_FILE_COUNT][num_z];
  double GroundTruth[MAX_FILE_COUNT][27];

  //SAVE PARAMETER
  const static int SAVEMAX = 115;
  const static int SAVEMAXCNT = MAX_FILE_COUNT;
  double SAVE_BUFFER[SAVEMAX][SAVEMAXCNT];
  int SAVE_cnt = 0;

  int iteration_number = 0;
  int total_backppgn_number = 0;
  double time_per_step;

  std::string estimator_info;
  std::string file_info;
  std::string initial_info;
  std::string time_size;
  std::string est_size;

  int idx_TRUE_Rotation = 0;//0
  int idx_TRUE_Velocity = idx_TRUE_Rotation + 9;//9
  int idx_TRUE_Position = idx_TRUE_Velocity + 3;//12
  int idx_TRUE_dv = idx_TRUE_Position + 3;//15
  int idx_TRUE_Bias_Gyro = idx_TRUE_dv + 12;//27
  int idx_TRUE_Bias_Acc = idx_TRUE_Bias_Gyro + 3;//30
  int idx_TRUE_Contact = idx_TRUE_Bias_Acc + 3;//33
  int idx_TRUE_Slip = idx_TRUE_Contact + 4;//37
  int idx_TRUE_Hard_Contact = idx_TRUE_Slip + 4;//41
  int idx_TRUE_rpy = idx_TRUE_Hard_Contact + 4;//45

  int idx_ESTIMATED_Rotation = idx_TRUE_rpy + 3;//48
  int idx_ESTIMATED_Velocity = idx_ESTIMATED_Rotation + 9;//57
  int idx_ESTIMATED_Position = idx_ESTIMATED_Velocity + 3;//60
  int idx_ESTIMATED_dv = idx_ESTIMATED_Position + 3;//63
  int idx_ESTIMATED_Bias_Gyro = idx_ESTIMATED_dv + 12;//75
  int idx_ESTIMATED_Bias_Acc = idx_ESTIMATED_Bias_Gyro + 3;//78
  int idx_ESTIMATED_Contact = idx_ESTIMATED_Bias_Acc + 3;//81
  int idx_ESTIMATED_Slip = idx_ESTIMATED_Contact + 4;//85
  int idx_ESTIMATED_Hard_Contact = idx_ESTIMATED_Slip + 4;//89
  int idx_ESTIMATED_rpy = idx_ESTIMATED_Hard_Contact + 4;//93

  int idx_iteration_No = idx_ESTIMATED_rpy + 3;//96
  int idx_backppgn_No = idx_iteration_No + 1;//97
  int idx_time_per_step = idx_backppgn_No + 1;//98
  int idx_cost = idx_time_per_step + 1;//99

  int idx_final_state_covariance = idx_cost + 4;
  int idx_end = idx_final_state_covariance + 12;

  //Parameters Setting
  bool textfile_flag = false;

  bool Retract_All_flag = false;

  int Max_Iteration = 100;
  double Optimization_Epsilon = 1e-3;

  int Max_backpropagate_num = 3;
  //To turn off backpropagation, set num=0
  double backppgn_rate = 0.5;

  bool slip_rejection_mode = false;
  double slip_threshold = 0.3;

  double long_term_v_threshold = 0;
  double long_term_a_threshold = 0;

  bool variable_contact_cov_mode = false;

  bool preintegration_mode_ = false;
  double cov_amplifier = 100;
  double gps_covariance_amplifier = 1.0;

 private:


  double eps = 1e-8;
};


