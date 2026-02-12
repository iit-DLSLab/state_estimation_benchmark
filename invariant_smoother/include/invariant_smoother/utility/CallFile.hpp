
#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
struct CSVData {
    Eigen::Vector3d imu_ang_vel;
    Eigen::Vector3d imu_acc;
    Eigen::Vector<double,12> jnt_pos;
    Eigen::Vector<double,12> jnt_vel;
    std::vector<Eigen::Vector3d> forkin_position; // 4 legs
    std::vector<Eigen::Matrix3d> forkin_jacobian; // 4 legs
    Eigen::Vector4d contact_state; // 4 legs

    CSVData() {
        imu_ang_vel.setZero();
        imu_acc.setZero();
        jnt_pos.setZero();
        jnt_vel.setZero();
        forkin_position.resize(4);
        forkin_jacobian.resize(4);
        contact_state.setZero();
    }
};

class CallFile {
public:
    static std::vector<CSVData> readCSV(const std::string& filename) {
        std::vector<CSVData> dataList;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return dataList;
        }

        std::string line;
        
        // Skip header
        if (std::getline(file, line)) {
            // Header skipped
        }

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            CSVData data;
            std::vector<double> row_values;

            while (std::getline(ss, cell, ',')) {
                 if (!cell.empty()) {
                    try {
                        row_values.push_back(std::stod(cell));
                    } catch (const std::exception& e) {
                        // Handle possible empty or malformed cells if necessary
                        row_values.push_back(0.0);
                    }
                }
            }
            
            // Expected columns: 3 (gyro) + 3 (acc) + 12 (jnt pos) + 12 (jnt vel) + 4 (contact) + 4*3 (foot pos) + 4*9 (foot jac) 
            // = 6 + 24 + 4 + 12 + 36 = 82
            if (row_values.size() < 82) {
                continue; // Skip incomplete rows
            }

            int idx = 0;

            // IMU Angular Velocity (3)
            data.imu_ang_vel(0) = row_values[idx++];
            data.imu_ang_vel(1) = row_values[idx++];
            data.imu_ang_vel(2) = row_values[idx++];

            // IMU Acceleration (3)
            data.imu_acc(0) = row_values[idx++];
            data.imu_acc(1) = row_values[idx++];
            data.imu_acc(2) = row_values[idx++];

            // Joint Positions (12)
            for (int i = 0; i < 12; ++i) {
                data.jnt_pos(i) = row_values[idx++];
            }

            // Joint Velocities (12)
            for (int i = 0; i < 12; ++i) {
                data.jnt_vel(i) = row_values[idx++];
            }

            // Contact State (4)
            data.contact_state(0) = row_values[idx++];
            data.contact_state(1) = row_values[idx++];
            data.contact_state(2) = row_values[idx++];
            data.contact_state(3) = row_values[idx++];

            // Four Foot Positions (3x1 each)
            for (int i = 0; i < 4; ++i) {
                data.forkin_position[i](0) = row_values[idx++];
                data.forkin_position[i](1) = row_values[idx++];
                data.forkin_position[i](2) = row_values[idx++];
            }

            // Four Foot Jacobians (3x3 each)
            for (int i = 0; i < 4; ++i) {
                // Assuming row-major order in CSV: r1c1, r1c2, r1c3, r2c1...
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        data.forkin_jacobian[i](r, c) = row_values[idx++];
                    }
                }
            }

            dataList.push_back(data);
        }

        file.close();
        return dataList;
    }
};
