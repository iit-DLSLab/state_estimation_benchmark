#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>

#include "Models/invariant_smoother.hpp"
#include "Models/smoother_struct.hpp"
#include "Models/robot_state.hpp"
#include "Models/InvariantExtendedKalmanFilter.hpp"

#include <chrono>

#include "rotations.h"

#include <unsupported/Eigen/MatrixFunctions> 

int main() {

    // Initializing the smoother
    double dt           = 0.001;         // indoor: 0.001;   // outdoor: 0.01;
    double gyro_exp     = -5;           // indoor:  -5;     // outdoor: -4;
    double acc_exp      = -1; 
    double slip_exp     = -1.3; 
    double contact_exp  = -3;           // indoor:  -3;     // outdoor: -2;    
    double encoder_exp  = -4;           // indoor:  -4;     // outdoor: -3;
    double bg_exp       = -10; 
    double ba_exp       = -10;
    double pri_ori_exp  = -8; 
    double pri_vel_exp  = -8; 
    double pri_pos_exp  = -8;
    double pri_bg_exp   = -10; 
    double pri_ba_exp   = -10;

    double convergence_cond = 1e-3;         // ANYmal: 1e-5;      // Hound: 1e-3
    int max_it_no           = 1;            // ANYmal: 100        // Hound: 1;
    int max_backpp_no       = 1000;         // ANYmal: 10         // Hound: 1000;
    double backpp_rate      = 0.7;          // ANYmal: 0.9        // Hound: 0.7
    int starting_point      = 0;
    bool SR                 = true;
    double slip_thr         = 0.3;           // ANYmal 0.48        // Hound: 0.3;   
    bool VCC                = false;    
    double cov_amplifier    = 1.0;           // ANYmal 5.0         // Hound: 1.0;

    double slip_rejection_threshold_in_meter_per_seconds                   = 0.2;
    double contact_loop_velocity_threshold_in_meter_per_seconds            = 0.3;
    double contact_loop_acceleration_threshold_in_meter_per_seconds_square = 0.0;

    int time_length = 25500;

    ROBOT_STATES state_;
    state_.Rotation = Eigen::Matrix<double,3,3>::Identity();
    state_.Position << 0.0, 0.0, 0.51; // 0.245282898, -0.283101349, 0.879358643;           // 0.0, 0.0, 0.48; -8.3900    0.6460    0.9683

    Eigen::Matrix<double, 16, 1> initial_condition;
    initial_condition << 0.0, 0.0, 0.51,       // 0.245282898, -0.283101349, 0.879358643, 	
                         1.0, 0.0, 0.0, 0.0, 	// quaternion w,x,y,z
                         0.0, 0.0, 0.0,     	// vx, vy, vz
                         0.0, 0.0, 0.0,     	// bgx, bgy, bgz
                         0.0, 0.0, 0.0;    		// bax, bay, baz

    EstimatorCovariances estimator_covariances_;
    EstimatorCommonStruct estimator_common_struct_;
    InvariantSmoother *invariant_smoother_;
    InvariantExtendedKalmanFilter *estimator_IEKF_;

    estimator_covariances_.cov_gyro_diagonal << pow(10, gyro_exp), pow(10, gyro_exp), pow(10, gyro_exp);
    estimator_covariances_.cov_acc_diagonal << pow(10, acc_exp), pow(10, acc_exp), pow(10, acc_exp);
    estimator_covariances_.cov_slip_diagonal << pow(10, slip_exp), pow(10, slip_exp), pow(10, slip_exp);
    estimator_covariances_.cov_contact_diagonal << pow(10, contact_exp), pow(10, contact_exp), pow(10, contact_exp);
    estimator_covariances_.cov_enc_diagonal << pow(10, encoder_exp), pow(10, encoder_exp), pow(10, encoder_exp);
    estimator_covariances_.cov_bias_gyro_diagonal << pow(10, bg_exp), pow(10, bg_exp), pow(10, bg_exp);
    estimator_covariances_.cov_bias_acc_diagonal << pow(10, ba_exp), pow(10, ba_exp), pow(10, ba_exp);
    estimator_covariances_.cov_prior_orientation_diagonal << pow(10, pri_ori_exp), pow(10, pri_ori_exp), pow(10, pri_ori_exp);
    estimator_covariances_.cov_prior_velocity_diagonal << pow(10, pri_vel_exp), pow(10, pri_vel_exp), pow(10, pri_vel_exp);
    estimator_covariances_.cov_prior_position_diagonal << pow(10, pri_pos_exp), pow(10, pri_pos_exp), pow(10, pri_pos_exp);
    estimator_covariances_.cov_prior_bias_gyro_diagonal << pow(10, pri_bg_exp), pow(10, pri_bg_exp), pow(10, pri_bg_exp);
    estimator_covariances_.cov_prior_bias_acc_diagonal << pow(10, pri_ba_exp), pow(10, pri_ba_exp), pow(10, pri_ba_exp);

    invariant_smoother_ = new InvariantSmoother();
    estimator_IEKF_ = new InvariantExtendedKalmanFilter();

    invariant_smoother_->estimator_common_struct_.leg_no = 4;
    invariant_smoother_->backppgn_rate = backpp_rate;
    invariant_smoother_->NUM_OF_TRASH_DATA = starting_point;
    invariant_smoother_->slip_rejection_mode = false; // SR;
    invariant_smoother_->slip_threshold = slip_thr;
    invariant_smoother_->variable_contact_cov_mode = VCC;
    invariant_smoother_->cov_amplifier = cov_amplifier;
    invariant_smoother_->long_term_v_threshold = contact_loop_velocity_threshold_in_meter_per_seconds;
    invariant_smoother_->long_term_a_threshold = contact_loop_acceleration_threshold_in_meter_per_seconds_square;

    invariant_smoother_->optimization_epsilon = convergence_cond;
    invariant_smoother_->max_iteration = max_it_no;
    invariant_smoother_->max_backpropagate_num = max_backpp_no;
    invariant_smoother_->retract_all_flag = false;

    invariant_smoother_->initialize(dt, estimator_covariances_, initial_condition);

    estimator_IEKF_->estimator_common_struct_.leg_no = 4;
    estimator_IEKF_->Optimization_Epsilon = 1e-3;
    estimator_IEKF_->Max_Iteration = max_it_no;

    estimator_IEKF_->NUM_OF_TRASH_DATA = starting_point;
    estimator_IEKF_->slip_rejection_mode = true;
    estimator_IEKF_->slip_threshold = slip_thr;
    estimator_IEKF_->variable_contact_cov_mode=VCC;
    estimator_IEKF_->cov_amplifier=cov_amplifier;

    estimator_IEKF_->Initialize(dt, estimator_covariances_, initial_condition);
  

    // Open the CSV files for reading
    std::ifstream measurements_file("/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/measurements.csv");

    // Open the output file for writing
    std::ofstream output_file("/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/results_no_timestamp/accuracy/einekf_pose.csv");
    std::ofstream velocity_file("/home/iit.local/ynistico/dls_ws_home/invariant_smoother_data/hound/outdoor_with_gps/results_no_timestamp/velocities/pis_vel_vs1.csv");

    // Check if all files are open
    if (!measurements_file.is_open()) {
        std::cerr << "Failed to open one or more files." << std::endl;
        return 1;
    }}

    if (!output_file.is_open() || !velocity_file.is_open()) {
        std::cerr << "Failed to open output files." << std::endl;
        return 1;
    }

    // Read the sensors data and contacts from the CSV file
    Eigen::Matrix<double, 30, 1> sensors;
    Eigen::Matrix<bool, 4, 1> contacts;
    Eigen::Matrix<double,3,1> x_lidar_i;
    Eigen::Matrix<double,9,1> rot_lidar;
    Eigen::Matrix<bool, 1, 1> lidar_update;
    Eigen::Matrix<double,3,1> x_gps_i;
    Eigen::Matrix<bool, 1, 1> gps_update;

    std::string line;

    int cnt_is = 0 ;
    while (std::getline(measurements_file, line)) 
    {
        std::istringstream iss(line);
        std::string token;
        for (int i = 0; i < 30; i++) {
            if (!std::getline(iss, token, ',')) {
                std::cerr << "Error: Unexpected end of line\n";
            break;
            }
            try {
                sensors[i] = std::stod(token);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error converting token to double: " << e.what() << '\n';
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range error converting token to double: " << e.what() << '\n';
            }
        }

        for (int i = 30; i < 34; i++) {
            std::getline(iss, token, ',');
            contacts(i-30) = std::stoi(token);
        }

        for (int i = 34; i < 37; i++) {
            if (!std::getline(iss, token, ',')) {
                std::cerr << "Error: Unexpected end of line\n";
            break;
            }
            try {
                x_lidar_i[i-34] = std::stod(token);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error converting token to double: " << e.what() << '\n';
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range error converting token to double: " << e.what() << '\n';
            }
        }

        for (int i = 37; i < 46; i++) {
            if (!std::getline(iss, token, ',')) {
                std::cerr << "Error: Unexpected end of line\n";
            break;
            }
            try {
                rot_lidar[i-37] = std::stod(token);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error converting token to double: " << e.what() << '\n';
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range error converting token to double: " << e.what() << '\n';
            }
        }

        for (int i = 46; i < 47; i++) {
            std::getline(iss, token, ',');
            lidar_update(i-46) = std::stoi(token);
        }

        for (int i = 47; i < 50; i++) {
            if (!std::getline(iss, token, ',')) {
                std::cerr << "Error: Unexpected end of line\n";
            break;
            }
            try {
                x_gps_i[i-47] = std::stod(token);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error converting token to double: " << e.what() << '\n';
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range error converting token to double: " << e.what() << '\n';
            }
        }

        for (int i = 50; i < 51; i++) {
            std::getline(iss, token, ',');
            gps_update(i-50) = std::stoi(token);
        }

        // Position of the base given by the lidar
        Eigen::Matrix<double,3,1> trial;
        trial << -0.319, 0.0 , -(0.072/2+0.078); // indoor: 0.0, 0.0, 0.48; // outdoor: -0.319, 0.0 , -(0.072/2+0.078)
        Eigen::Matrix<double,3,1> x_lidar_base;
        x_lidar_base = x_lidar_i + trial;
        // Rotation of the base given by the lidar
        Eigen::Matrix<double,3,3> R_lidar_i;
        R_lidar_i << rot_lidar[0], rot_lidar[1], rot_lidar[2],
                     rot_lidar[3], rot_lidar[4], rot_lidar[5],
                     rot_lidar[6], rot_lidar[7], rot_lidar[8];


        // Call the invariant smoother main function
        Eigen::Matrix<double,3,1> sqrt_lidar_i; sqrt_lidar_i <<  0.6, 0.2, 0.004; // 0.3,0.3,0.3;
        Eigen::Matrix<double,3,1> sqrt_gps_i; sqrt_gps_i << 1.0,1.0,1.0;
        Eigen::Matrix<double,3,3> R_lidar_trial = Eigen::Matrix<double,3,3>::Identity();
        int num = lidar_update[0];
        bool lidar_in_i = num;   //num; //false;
        bool gps_in_i = true;
        // x_gps_i = x_lidar_i;
        cnt_is++;
        if(cnt_is%2 == 0){
            auto start_time = std::chrono::high_resolution_clock::now();
            // std::cout << "lidar_in_i: " << lidar_in_i << std::endl;
            // invariant_smoother_->one_step(sensors, contacts, state_, x_lidar_base, R_lidar_i, sqrt_lidar_i, lidar_in_i,x_gps_i, sqrt_gps_i, gps_in_i);
            estimator_IEKF_->Onestep(sensors, contacts, state_,x_lidar_base, R_lidar_i, sqrt_lidar_i, lidar_in_i, x_gps_i, sqrt_gps_i, gps_in_i);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            // std::cout << " =============== estimates ================ "<< std::endl;
            // std::cout << "Execution time of one_step: " << duration.count() << " microseconds" << std::endl;
            std::cout << " output - state ... "<< state_.Position.transpose()<< std::endl;
        }
        // std::cout << " run is ... " << cnt_is << std::endl;

        // Get the position, velocity and rotation from the estimated state
		Eigen::Vector3d position = state_.Position;
		Eigen::Vector3d velocity = state_.Velocity;
		Eigen::Matrix<double,3,3> rotation = state_.Rotation;
		Eigen::Quaterniond est_q;   // estimated quaternion
		est_q =  iit::commons::rotMatToQuat(rotation); 
        Eigen::Vector3d est_rpy;    // estimated roll, pitch, yaw
        est_rpy = iit::commons::rotTorpy(rotation);

        
        // Write position vector and quaternion to the first output file
        for (int i = 0; i < 3; ++i) {
            output_file << position(i);
            if (i < 2) {
                output_file << ",";
            }
        }

        output_file << ","; // Separate position and quaternion

        output_file << est_q.w() << "," << est_q.x() << "," << est_q.y() << "," << est_q.z() << std::endl;


        // Write velocity vector to the second output file
        for (int i = 0; i < 3; ++i) {
            velocity_file << velocity(i);
            if (i < 2) {
                velocity_file << ",";
            }
        }
        velocity_file << std::endl;
    }

        // Close the files
        measurements_file.close();

        output_file.close();
        velocity_file.close();

        return 0;
    }