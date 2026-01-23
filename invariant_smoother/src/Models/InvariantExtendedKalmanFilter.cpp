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
#include "Models/InvariantExtendedKalmanFilter.hpp"

#define DT_MIN 1e-6
#define DT_MAX 1

using std::cout;
using std::endl;
using namespace inekf;

std::vector<Eigen::Vector3d> InvariantExtendedKalmanFilter::Variable_Contact_Cov(int time){

  std::vector<Eigen::Vector3d> contact_cov_array;
  contact_cov_array.clear();

  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	Eigen::Vector3d contact_cov = cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_contact_diagonal;
	double dv_abs;

	if (HARD_CONTACT_t[time](k)) {
	  if (variable_contact_cov_mode) {

		for(int iter_joint=0; iter_joint < 3; iter_joint++){

		  dv_abs = abs(d_v[time](3 * k + iter_joint));
		  contact_cov(iter_joint) = (1+dv_abs)*cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_contact_diagonal(iter_joint);
		  if (contact_cov(iter_joint) > cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_slip_diagonal(iter_joint)) {
			contact_cov(iter_joint) = cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_slip_diagonal(iter_joint);
		  }

		}

	  } else if (slip_rejection_mode) {
		if ((d_v[time].block(3 * k, 0, 3, 1).norm() > slip_threshold)) {
		  contact_cov = cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_slip_diagonal;
		} else {
		  contact_cov = cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_contact_diagonal;
		}
	  } else {
		contact_cov = cov_amplifier*estimator_common_struct_.estimator_covariances_.cov_contact_diagonal;
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

    dt = estimator_common_struct_.dt;
    gravity << 0,0, -Robot::Gravity;


    std::string mode;

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


void InvariantExtendedKalmanFilter::new_measurement
(
    Eigen::Matrix<double, 30, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i, Eigen::Matrix<double,3,1> sqrt_lidar_i, bool lidar_in_i,
    Eigen::Matrix<double,3,1> x_gps_i, Eigen::Matrix<double,3,1> sqrt_gps_i, bool gps_in_i
)
{

    IMU_Gyro[frame_count] = Sensor_i.block(0,0,3,1);
    IMU_Acc[frame_count] = Sensor_i.block(3,0,3,1);

    ENCODER[frame_count] = Sensor_i.block(6,0,12,1);
    //ENCODER[frame_count] << Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1);

    ENCODERDOT[frame_count] = Sensor_i.block(18,0,12,1);
    CONTACT_t[frame_count] = Contact_i;

    imu_measurement <<IMU_Gyro[frame_count], IMU_Acc[frame_count];
    
    //propagate-----------------------------------------------------------------------------

    //Slip Rejection-------------------------------------------------------------------------------

    if ( frame_count==0 ){

        for (int k=0; k<4; k++){
            SLIP_t[0](k) = false;
            HARD_CONTACT_t[0](k)  = CONTACT_t[0](k)-SLIP_t[0](k);
        }

    }else{

        Eigen::Vector3d phi = (imu_measurement.block(0,0, 3,1)-filter.getState().getGyroscopeBias())*dt;
        Rotation_s[1] = filter.getState().getRotation() * Expm_Vec(phi);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(Rotation_s[frame_count], Eigen::ComputeFullU|Eigen::ComputeFullV);
        Rotation_s[1] = svd.matrixU() * svd.matrixV().transpose();

        Velocity_s[1] = filter.getState().getVelocity() + (Rotation_s[1]*(imu_measurement.block(3,0, 3,1)-filter.getState().getAccelerometerBias()) + gravity)*dt;


        double contact_dv_mean = 0;
        int count = 0;

        for (int k=0; k<4; k++){

            d_v[1].block(3*k,0,3,1) = Velocity_s[1]
                    + Rotation_s[1]*LeggedRobotKinematics::GetJacobian(k,ENCODER[1].block<3,1>(3*k,0))*ENCODERDOT[1].block<3,1>(3*k,0)
                    + Rotation_s[1]*Hat_so3(IMU_Gyro[1] - Bias_Gyro_s[1])*LeggedRobotKinematics::GetImu2FootPosition(k,ENCODER[1].block<3, 1>(3*k, 0),Robot::FootRadiusForEstimator,estimator_common_struct_.IMU2BD);

            HARD_CONTACT_t[1](k)  = CONTACT_t[1](k);

            if ( slip_rejection_mode == true && CONTACT_t[1](k) == true &&
                 !SLIP_t[1](k) ){
                contact_dv_mean += (d_v[0].block(3*k,0, 3,1).norm() - contact_dv_mean)/(count+1);
                count++;
            }

//            HARD_CONTACT_t[1](k)  = CONTACT_t[1](k)-SLIP_t[1](k);
        }

        //estimator_common_struct_.slip_threshold = contact_dv_mean*2;

        for(int p=0; p<=1; p++){

            for (int k=0; k<estimator_common_struct_.leg_no; k++) {

                if ((slip_rejection_mode == true) && (CONTACT_t[p](k) == true) &&
                    (d_v[p].block(3 * k, 0, 3, 1).norm() > estimator_common_struct_.slip_threshold)) {
                    SLIP_t[p](k) = true;
                }
            }
        }

    }

}



void InvariantExtendedKalmanFilter::Propagate_Correct
(
    Eigen::Matrix<double, 3, 1> x_lid_i, Eigen::Matrix<double,3,3> R_lid_i, bool lidar_in_i,
    Eigen::Matrix<double, 3, 1> x_gps_i, bool gps_in_i
)
{
    if(frame_count>0) {
        filter.Propagate(imu_measurement_prev, dt, Variable_Contact_Cov(0));
    }

    imu_measurement_prev = imu_measurement;



    std::vector<std::pair<int,bool> > contacts;

        for (int i=0; i<4; i++) {
            int id = i;
            bool indicator = HARD_CONTACT_t[frame_count](i);

            contacts.push_back(std::pair<int,bool> (id, indicator));
        }


    filter.setContacts(contacts);

    int id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    inekf::vectorKinematics measured_kinematics;


    for (int k=0; k<4; k++) {
        id = k;
        pose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        pose.block<3,1>(0,3) = LeggedRobotKinematics::GetImu2FootPosition(k,ENCODER[frame_count].block(3*k,0,3,1),Robot::FootRadiusForEstimator,estimator_common_struct_.IMU2BD);

        Eigen::Matrix3d FK_Jacobian;
        Eigen::Matrix3d Covariance_Encoder_leg;
        FK_Jacobian = LeggedRobotKinematics::GetJacobian(k,ENCODER[frame_count].block(3*k,0,3,1));
        Covariance_Encoder_leg = estimator_common_struct_.estimator_covariances_.Covariance_Encoder;
        covariance.block(3,3, 3,3) = FK_Jacobian * Covariance_Encoder_leg * FK_Jacobian.transpose();

        inekf::Kinematics frame(id, pose, covariance);
        measured_kinematics.push_back(frame);
    }
    // std::cout << "measured_kinematics.size(): ";
    // std::cout << measured_kinematics.size() << std::endl;

    // Correct state using kinematic measurements

    bool flag = true;
    if(time_count%10==5){
        flag = true;
    }

    // std::cout << "InEKF!" << std::endl;

    filter.CorrectKinematicsAndLidar(measured_kinematics, flag, lidar_in_i, x_lid_i,gps_in_i, x_gps_i);
    // filter.CorrectKinematics(measured_kinematics,flag, lidar_in_i, x_lid_i);        

    for (int k=0; k<estimator_common_struct_.leg_no; k++){
        d_v[1].block(3*k,0,3,1) = filter.getState().getVelocity()
                + filter.getState().getRotation()*LeggedRobotKinematics::GetJacobian(k,ENCODER[1].block<3,1>(3*k,0))*ENCODERDOT[1].block<3,1>(3*k,0)
                + filter.getState().getRotation()*Hat_so3(IMU_Gyro[1] - filter.getState().getGyroscopeBias())*LeggedRobotKinematics::GetImu2FootPosition(k,ENCODER[1].block<3, 1>(3*k, 0),Robot::FootRadiusForEstimator,estimator_common_struct_.IMU2BD);
//        //Test Junny
//        SLIP_t[1](k) = false;
//
//        if (slip_rejection_mode == true && CONTACT_t[1](k) == true &&
//            d_v[1].block(3*k,0, 3,1).norm() > slip_threshold){
//            SLIP_t[1](k) = true;
//        }
//        HARD_CONTACT_t[1](k)  = CONTACT_t[1](k);

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

void InvariantExtendedKalmanFilter::Onestep
(
    Eigen::Matrix<double, 30, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i,    ROBOT_STATES &state_,
    Eigen::Matrix<double, 3, 1> x_lid_i, Eigen::Matrix<double,3,3> R_lid_i, Eigen::Matrix<double, 3, 1> sqrt_lid_i, bool lid_in_i, Eigen::Matrix<double, 3, 1> x_gps_i, Eigen::Matrix<double, 3, 1> sqrt_gps_i, bool gps_in_i
){

    clock_t start, finish;
    double duration;
    start = clock();

    // std::cout << "invariant ekf" << std::endl;

    new_measurement(Sensor_i, Contact_i, x_lid_i, R_lid_i, sqrt_lid_i, lid_in_i, x_gps_i, sqrt_gps_i, gps_in_i);
    Propagate_Correct(x_lid_i, R_lid_i,lid_in_i, x_gps_i, gps_in_i);

    send_states(state_);

    SAVE_onestep_Z1(time_count);

    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    SAVE_BUFFER[96][time_count] = 1;
    SAVE_BUFFER[97][time_count] = duration;

    if(filter.getState().getVelocity().norm()>100 && dt!=0)
    {
        cout<<"Velocity is too high!!"<<endl;
    }


    frame_count++;
    if(frame_count>1)
    {
        frame_count = 1;
        sliding_window_flag = true;
    }

    sliding_window();
}
