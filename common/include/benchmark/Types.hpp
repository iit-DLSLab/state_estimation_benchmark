#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace benchmark {

struct MeasurementSample {
    double t;
    Eigen::Vector3d omega;
    Eigen::Vector3d acc;
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::Matrix<int, 4, 1> contact;
};

struct GroundTruthSample {
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
};

using MeasurementSequence = std::vector<MeasurementSample>;
using GroundTruthSequence = std::vector<GroundTruthSample>;

} // namespace benchmark
