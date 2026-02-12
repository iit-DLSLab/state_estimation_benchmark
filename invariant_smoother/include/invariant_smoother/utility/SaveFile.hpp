#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "invariant_smoother/estimator/RobotState_Smoother.hpp"

class SaveFile {
public:
    // times_abs: same length of states. times_rel can be computed inside.
    static void writeCSV(const std::string& filename,
                         const std::vector<ROBOT_STATES>& states,
                         const std::vector<double>& t_abs)
    {
        if (states.size() != t_abs.size()) {
            throw std::runtime_error("SaveFile::writeCSV: states.size() != t_abs.size()");
        }

        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + filename);
        }

        file.setf(std::ios::fixed);
        file << std::setprecision(9);

        // Header (MUSE-like)
        file << "t_rel,t_abs,px,py,pz,vx,vy,vz,qw,qx,qy,qz\n";

        if (states.empty()) {
            file.close();
            std::cout << "Saved empty CSV to " << filename << "\n";
            return;
        }

        const double t0 = t_abs.front();

        for (size_t i = 0; i < states.size(); ++i) {
            const auto& s = states[i];

            const double t_rel = t_abs[i] - t0;

            // Rotation -> Quaternion
            Eigen::Matrix3d R = s.Rotation;
            // optional: enforce orthonormal (small numerical fix)
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            R = svd.matrixU() * svd.matrixV().transpose();

            Eigen::Quaterniond q(R);
            q.normalize(); // just in case

            file << t_rel << "," << t_abs[i] << ","
                 << s.Position(0) << "," << s.Position(1) << "," << s.Position(2) << ","
                 << s.Velocity(0) << "," << s.Velocity(1) << "," << s.Velocity(2) << ","
                 << q.w() << "," << q.x() << "," << q.y() << "," << q.z()
                 << "\n";
        }

        file.close();
        std::cout << "Data saved to " << filename << "\n";
    }
};
