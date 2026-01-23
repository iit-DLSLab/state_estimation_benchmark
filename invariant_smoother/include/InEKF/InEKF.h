// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.

/**
 *  @file   InEKF.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#ifndef INEKF_H
#define INEKF_H 
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#if INEKF_USE_MUTEX
#include <mutex>
#endif
#include <algorithm>
#include "InEKF/RobotState.h"
#include "InEKF/NoiseParams.h"
#include "InEKF/LieGroup.h"

namespace inekf {

class Kinematics {
    public:

        Kinematics(int id_in, Eigen::Matrix4d pose_in, Eigen::Matrix<double,6,6> covariance_in) : id(id_in), pose(pose_in), covariance(covariance_in) { }

        int id;
        Eigen::Matrix4d pose;
        Eigen::Matrix<double,6,6> covariance;
};

class Landmark {
    public:

        Landmark(int id_in, Eigen::Vector3d position_in) : id(id_in), position(position_in) { }

        int id;
        Eigen::Vector3d position;
};

typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > > mapIntVector3d;
typedef std::map<int,Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Eigen::Vector3d> > >::iterator mapIntVector3dIterator;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > vectorLandmarks;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> >::const_iterator vectorLandmarksIterator;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> > vectorKinematics;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> >::const_iterator vectorKinematicsIterator;

class Observation {

    public:

        Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI);
        bool empty();

        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        friend std::ostream& operator<<(std::ostream& os, const Observation& o);  
};


class InEKF {
    
    public:

        InEKF();
        InEKF(NoiseParams params);
        InEKF(RobotState state);
        InEKF(RobotState state, NoiseParams params);

        RobotState getState();
        NoiseParams getNoiseParams();
        mapIntVector3d getPriorLandmarks();
        std::map<int,int> getEstimatedLandmarks();
        std::map<int,bool> getContacts();
        std::map<int,int> getEstimatedContactPositions();
        void setState(RobotState state);
        void setLidarOdometry(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& position);
        void setNoiseParams(NoiseParams params);
        void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
        void setContacts(std::vector<std::pair<int,bool> > contacts);

        void Propagate(const Eigen::Matrix<double,6,1>& m, double dt, std::vector<Eigen::Vector3d> contact_cov_array);
        void Correct(const Observation& obs);
        void updateWithLidar(const Eigen::Vector3d& lidar_position, const Eigen::Matrix3d& lidar_rotation);
        void CorrectLandmarks(const vectorLandmarks& measured_landmarks);
        void CorrectKinematics(const vectorKinematics& measured_kinematics, bool flag, bool lidar_available, Eigen::Vector3d lidar_position);
        void CorrectKinematicsAndLidar(const vectorKinematics& measured_kinematics, bool flag, bool lidar_available, Eigen::Vector3d lidar_position, bool gps_available, Eigen::Vector3d gps_position);
        void Reset();


    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; // Gravity
        Eigen::Matrix3d lidar_rotation_;
        Eigen::Vector3d lidar_position_;
        mapIntVector3d prior_landmarks_;
        std::map<int,int> estimated_landmarks_;
        std::map<int,bool> contacts_;
        std::map<int,int> estimated_contact_positions_;
#if INEKF_USE_MUTEX
        std::mutex estimated_contacts_mutex_;
        std::mutex estimated_landmarks_mutex_;
#endif
};

} // end inekf namespace
#endif 
