// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "iekf/estimator/InvariantExtendedKalmanFilter.hpp"

#define DT_MIN 1e-6
#define DT_MAX 1

using std::cout;
using std::endl;
using namespace inekf;

InvariantExtendedKalmanFilter::InvariantExtendedKalmanFilter() = default;
InvariantExtendedKalmanFilter::~InvariantExtendedKalmanFilter() = default;


std::vector<Eigen::Vector3d> InvariantExtendedKalmanFilter::Variable_Contact_Cov(int time){
  std::vector<Eigen::Vector3d> contact_cov_array;
  contact_cov_array.clear();

  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
    Eigen::Vector3d contact_cov = cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_contact_diagonal;
    double dv_abs;

    if (HARD_CONTACT_t[time](k)) {
      if (variable_contact_cov_mode) {
        for (int iter_joint = 0; iter_joint < 3; iter_joint++) {
          dv_abs = std::abs(d_v[time](3 * k + iter_joint));
          contact_cov(iter_joint) = (1 + dv_abs) * cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_contact_diagonal(iter_joint);
          if (contact_cov(iter_joint) > cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_slip_diagonal(iter_joint)) {
            contact_cov(iter_joint) = cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_slip_diagonal(iter_joint);
          }
        }
      } else if (slip_rejection_mode) {
        if ((d_v[time].block(3 * k, 0, 3, 1).norm() > slip_threshold)) {
          contact_cov = cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_slip_diagonal;
        } else {
          contact_cov = cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_contact_diagonal;
        }
      } else {
        contact_cov = cov_amplifier * estimator_common_struct_.estimator_covariances_.cov_contact_diagonal;
      }
    }
    contact_cov_array.push_back(contact_cov);
  }

  return contact_cov_array;
}


void InvariantExtendedKalmanFilter::Initialize(double _dt, EstimatorCovariances estimator_covariances, Eigen::Matrix<double, 16, 1> initial_condition)
{

    //  ---- Initialize invariant extended Kalman filter ----- //

    filter.Reset();

    sliding_window_flag = false;
    frame_count = 0;
    time_count = 0;
	estimator_common_struct_.estimator_covariances_ = estimator_covariances;
    estimator_common_struct_.Covariance_Reset();
    estimator_common_struct_.dt = _dt;

    estimator_common_struct_.variable_contact_cov_mode = variable_contact_cov_mode;
    estimator_common_struct_.cov_amplifier = cov_amplifier;
    // estimator_common_struct_.gps_covariance_amplifier = gps_covariance_amplifier;
    estimator_common_struct_.slip_rejection_mode = slip_rejection_mode;
    estimator_common_struct_.slip_threshold = slip_threshold;
    // estimator_common_struct_.long_term_v_threshold = long_term_v_threshold;
    // estimator_common_struct_.long_term_a_threshold = long_term_a_threshold;
	estimator_common_struct_.estimator_covariances_ = estimator_covariances;
    estimator_common_struct_.Covariance_Reset();

    dt = estimator_common_struct_.dt;
    gravity << 0,0, -9.81;


    // std::string mode;

    //covariance setting
    Eigen::Matrix<double,15,15> ForP;
    ForP.setZero();
    ForP.block(0,0,3,3) = estimator_common_struct_.estimator_covariances_.Covariance_Prior_Orientation;
	ForP.block(3,3,3,3) = estimator_common_struct_.estimator_covariances_.Covariance_Prior_Velocity;
	ForP.block(6,6,3,3) = estimator_common_struct_.estimator_covariances_.Covariance_Prior_Position;
	ForP.block(9,9,3,3) = estimator_common_struct_.estimator_covariances_.Covariance_Prior_Bias_Gyro;
	ForP.block(12,12,3,3) = estimator_common_struct_.estimator_covariances_.Covariance_Prior_Bias_Acc;


    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;

    inekf::RobotState initial_state(ForP);

    Eigen::Vector4d initial_quaternion = initial_condition.block(3,0,4,1);
    R0=Quaternion_to_Rotation_Matrix(initial_quaternion);;
    v0.setZero(); // initial velocity
    p0=initial_condition.block(0,0,3,1);
    bg0.setZero(); // initial gyroscope bias
    ba0.setZero(); // initial accelerometer bias


    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);


    imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();

    // Initialize state covariance
    inekf::NoiseParams noise_params;

	noise_params.setGyroscopeNoise(estimator_common_struct_.estimator_covariances_.Covariance_Gyro);
    noise_params.setAccelerometerNoise(estimator_common_struct_.estimator_covariances_.Covariance_Acc);
    noise_params.setGyroscopeBiasNoise(estimator_common_struct_.estimator_covariances_.Covariance_Bias_Gyro);
    noise_params.setAccelerometerBiasNoise(estimator_common_struct_.estimator_covariances_.Covariance_Bias_Acc);
    noise_params.setContactNoise(estimator_common_struct_.estimator_covariances_.Covariance_Contact);
    noise_params.setLandmarkNoise(estimator_common_struct_.estimator_covariances_.Covariance_Contact);

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);

    // Initialize LiDAR data
    lidar_rotation_ = Eigen::Matrix3d::Identity();
    lidar_position_ = Eigen::Vector3d::Zero();

}


void InvariantExtendedKalmanFilter::new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i,
                                                    Eigen::Matrix<bool, 4, 1> Contact_i,
                                                    const MEAS_FORWARD_KINEMATICS &forkin_set)
{
    // salvo misure nel buffer corrente (frame_count può essere 0 o 1)
    int idx = std::min(frame_count, 1);
    IMU_Gyro[idx] = Sensor_i.block(0,0,3,1);
    IMU_Acc[idx]  = Sensor_i.block(3,0,3,1);

    ENCODER[idx]    = Sensor_i.block(6,0,12,1);
    ENCODERDOT[idx] = Sensor_i.block(18,0,12,1);

    CONTACT_t[idx] = Contact_i;

    // salva forkin (assicurati che forkin_meas_ sia sized >= 2)
    forkin_meas_[idx] = forkin_set;

    imu_measurement << IMU_Gyro[idx], IMU_Acc[idx];

    // ---- Se primo frame: inizializza slip/hard contact e esci
    if (frame_count == 0) {
        for (int k = 0; k < 4; ++k) {
            SLIP_t[0](k) = false;
            HARD_CONTACT_t[0](k) = CONTACT_t[0](k);
            // d_v[0] può rimanere zero
            d_v[0].block<3,1>(3*k,0).setZero();
        }
        return;
    }

    // ---------- altrimenti propago (questo è solo calcolo di d_v predetta) ----------
    const Eigen::Vector3d bg = filter.getState().getGyroscopeBias();
    const Eigen::Vector3d ba = filter.getState().getAccelerometerBias();
    const Eigen::Matrix3d R0 = filter.getState().getRotation();
    const Eigen::Vector3d v0 = filter.getState().getVelocity();

    // predicted rotation/velocity using imu curr (usiamo idx corrente)
    Eigen::Vector3d phi = (imu_measurement.block(0,0,3,1) - bg) * dt;
    Rotation_s[1] = R0 * Expm_Vec(phi);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Rotation_s[1], Eigen::ComputeFullU | Eigen::ComputeFullV);
    Rotation_s[1] = svd.matrixU() * svd.matrixV().transpose();

    Velocity_s[1] = v0 + (Rotation_s[1] * (imu_measurement.block(3,0,3,1) - ba) + gravity) * dt;

    // compute predicted d_v for each leg (use idx current)
    for (int k = 0; k < 4; ++k) {
        HARD_CONTACT_t[1](k) = CONTACT_t[1](k);

        const Eigen::Vector3d w_b = (IMU_Gyro[1] - bg); // gyros in base (ok: IMU_Gyro[1] valid for frame_count>=1)
        const Eigen::Vector3d p_b = forkin_meas_[1].forkin_position[k];
        const Eigen::Matrix3d J_b = forkin_meas_[1].forkin_jacobian[k];
        const Eigen::Vector3d dq_leg = ENCODERDOT[1].block<3,1>(3*k,0);

        d_v[1].block<3,1>(3*k,0) =
              Velocity_s[1]
            + Rotation_s[1] * (J_b * dq_leg)
            + Rotation_s[1] * (Hat_so3(w_b) * p_b);
    }

    // slip rejection come prima
    for (int p = 0; p <= 1; ++p) {
        for (int k = 0; k < estimator_common_struct_.leg_no; ++k) {
            if (slip_rejection_mode && CONTACT_t[p](k) &&
                (d_v[p].block<3,1>(3*k,0).norm() > estimator_common_struct_.slip_threshold)) {
                SLIP_t[p](k) = true;
            } else {
                SLIP_t[p](k) = false;
            }
        }
    }
}


void InvariantExtendedKalmanFilter::Propagate_Correct()
{
    // Propagate only se ho un frame precedente (come prima)
    if (frame_count > 0) {
        filter.Propagate(imu_measurement_prev, dt, Variable_Contact_Cov(0));
    }
    imu_measurement_prev = imu_measurement;

    std::cout << "Propagate_Correct: A" << std::endl;

    // set contacts: usa frame corrente (frame_count può essere 0 o 1), scegli idx sicuro
    int idx = std::min(frame_count, 1);
    std::vector<std::pair<int,bool>> contacts;
    for (int i = 0; i < 4; ++i) {
        contacts.emplace_back(i, HARD_CONTACT_t[idx](i));
    }
    filter.setContacts(contacts);

    std::cout << "Propagate_Correct: B" << std::endl;

    // build kinematic measurements using il buffer idx (quello attuale)
    inekf::vectorKinematics measured_kinematics;
    measured_kinematics.reserve(4);

    for (int k = 0; k < 4; ++k) {
        int id = k;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double,6,6> covariance = Eigen::Matrix<double,6,6>::Zero();

        pose.block<3,1>(0,3) = forkin_meas_[idx].forkin_position[k];
        Eigen::Matrix3d J_b = forkin_meas_[idx].forkin_jacobian[k];
        Eigen::Matrix3d CovEncoder = estimator_common_struct_.estimator_covariances_.Covariance_Encoder;
        covariance.block<3,3>(3,3) = J_b * CovEncoder * J_b.transpose();

        inekf::Kinematics frame(id, pose, covariance);
        measured_kinematics.push_back(frame);
    }

    // Correction
    bool flag = true;
    if (time_count % 10 == 5) flag = true;
    filter.CorrectKinematics(measured_kinematics, flag);

    // update filtered d_v (use idx)
    for (int k = 0; k < estimator_common_struct_.leg_no; ++k) {
        const Eigen::Vector3d bg = filter.getState().getGyroscopeBias();
        const Eigen::Vector3d w_b = (IMU_Gyro[idx] - bg);
        const Eigen::Vector3d p_b = forkin_meas_[idx].forkin_position[k];
        const Eigen::Matrix3d J_b = forkin_meas_[idx].forkin_jacobian[k];
        const Eigen::Vector3d dq_leg = ENCODERDOT[idx].block<3,1>(3*k,0);

        d_v[idx].block<3,1>(3*k,0) =
              filter.getState().getVelocity()
            + filter.getState().getRotation() * (J_b * dq_leg)
            + filter.getState().getRotation() * (Hat_so3(w_b) * p_b);
    }
}


void InvariantExtendedKalmanFilter::send_states(ROBOT_STATES &state_){


    //Sending Estimated States----------------------------------------------------------------------------

    Eigen::Vector3d W_pos_imu2bd = filter.getState().getRotation() * estimator_common_struct_.IMU2BD;

    state_.Position = filter.getState().getPosition() + W_pos_imu2bd;

    Eigen::Vector3d w_gyro;
    w_gyro = filter.getState().getRotation()*(IMU_Gyro[1] - filter.getState().getGyroscopeBias());


    state_.Velocity = filter.getState().getVelocity() + w_gyro.cross(W_pos_imu2bd);
    state_.Bias_Gyro = filter.getState().getGyroscopeBias();
    state_.Bias_Acc = filter.getState().getAccelerometerBias();
    state_.Rotation= filter.getState().getRotation();


    if(frame_count == 0)
    {
        state_.Hard_Contact = HARD_CONTACT_t[frame_count];
        state_.Contact = CONTACT_t[frame_count];
        state_.Slip = SLIP_t[frame_count];
        state_.d_v = d_v[frame_count];
    }
    else
    {
        state_.Hard_Contact = HARD_CONTACT_t[frame_count-1];
        state_.Contact = CONTACT_t[frame_count-1];
        state_.Slip = SLIP_t[frame_count-1];
        state_.d_v = d_v[frame_count-1];
    }

}



void InvariantExtendedKalmanFilter::SAVE_onestep_Z1(int cnt){

    if(cnt<SAVEMAXCNT)
    {
        for(int i=0; i<estimator_common_struct_.leg_no; i++)
        {
            SAVE_BUFFER[idx_ESTIMATED_Contact+i][cnt] = CONTACT_t[frame_count](i);
            SAVE_BUFFER[idx_ESTIMATED_Slip+i][cnt] = SLIP_t[frame_count](i);
            SAVE_BUFFER[idx_ESTIMATED_Hard_Contact+i][cnt] = HARD_CONTACT_t[frame_count](i);

        }

        Eigen::Vector3d temp_eul = Rotation_to_EulerZYX(filter.getState().getRotation());
        for(int i=0;i<3;i++)
        {
            SAVE_BUFFER[idx_ESTIMATED_Position+i][cnt] = filter.getState().getPosition()(i);
            SAVE_BUFFER[idx_ESTIMATED_Velocity+i][cnt] = filter.getState().getVelocity()(i);
            SAVE_BUFFER[idx_ESTIMATED_Bias_Gyro+i][cnt] = filter.getState().getGyroscopeBias()(i);
            SAVE_BUFFER[idx_ESTIMATED_Bias_Acc+i][cnt] = filter.getState().getAccelerometerBias()(i);
            SAVE_BUFFER[idx_ESTIMATED_rpy+i][cnt] = temp_eul(i);
        }

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SAVE_BUFFER[idx_ESTIMATED_Rotation+i*3+j][cnt] = filter.getState().getRotation()(i,j);
            }
        }

    }
    else
    {
        cnt = SAVEMAXCNT-1;
        //        overcnt = true;
        cout<<"over Max SAVE_cnt!!"<<endl;
    }

}


void InvariantExtendedKalmanFilter::sliding_window(){

    if (frame_count>0){
        std::swap(HARD_CONTACT_t[0],HARD_CONTACT_t[1]);
        std::swap(CONTACT_t[0],CONTACT_t[1]);
        std::swap(SLIP_t[0],SLIP_t[1]);
        std::swap(ENCODER[0],ENCODER[1]);
        d_v[0].swap(d_v[1]);
    }

}

void InvariantExtendedKalmanFilter::Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i,
                                            Eigen::Matrix<bool, 4, 1> Contact_i,
                                            const MEAS_FORWARD_KINEMATICS &forkin_set,
                                            ROBOT_STATES &state_)
{
    clock_t start = clock();

    std::cout << "A" << std::endl;

    new_measurement(Sensor_i, Contact_i, forkin_set);
    std::cout << "B" << std::endl;
    Propagate_Correct();
    std::cout << "C" << std::endl;

    send_states(state_);
    std::cout << "D" << std::endl;
    SAVE_onestep_Z1(time_count);
    std::cout << "E" << std::endl;

    clock_t finish = clock();
    std::cout << "F" << std::endl;
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    std::cout << "G" << std::endl;
    SAVE_BUFFER[96][time_count] = 1;
    std::cout << "H" << std::endl;
    SAVE_BUFFER[97][time_count] = duration;
    std::cout << "I" << std::endl;

    frame_count++;
    if(frame_count > 1) {
        frame_count = 1;
        sliding_window_flag = true;
    }
    sliding_window();
}

