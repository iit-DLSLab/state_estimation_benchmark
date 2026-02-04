#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// === MUSE plugin core dependencies ===
#include "Models/attitude_bias_XKF.hpp"
#include "Models/attitude_bias_NLO.hpp"
#include "lib.hpp"
#include "rotations.h"

// -----------------------------
// Utils: CSV
// -----------------------------
    static std::vector<std::string> splitCSVLine(const std::string& line)
    {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream ss(line);
    while (std::getline(ss, token, ',')) tokens.push_back(token);
    return tokens;
    }

    static std::unordered_map<std::string, int> headerIndex(const std::vector<std::string>& header)
    {
    std::unordered_map<std::string, int> m;
    for (int i = 0; i < (int)header.size(); ++i) {
        m[header[i]] = i;
    }
    return m;
    }

    static double getColAsDouble(const std::vector<std::string>& row,
                                const std::unordered_map<std::string, int>& idx,
                                const std::string& name)
    {
    auto it = idx.find(name);
    if (it == idx.end()) throw std::runtime_error("Missing column: " + name);
    const int j = it->second;
    if (j < 0 || j >= (int)row.size()) throw std::runtime_error("Bad index for: " + name);
    return std::stod(row[j]);
    }

    // -----------------------------
    // Utils: rotations (equivalenti a iit::commons::*)
    // -----------------------------
    // static inline Eigen::Matrix3d quatToRotMat(const Eigen::Quaterniond& q)
    // {
    //     return q.normalized().toRotationMatrix();
    // }

    // roll-pitch-yaw (X-Y-Z, roll about x, pitch about y, yaw about z)
    // static inline Eigen::Vector3d quatToRPY(const Eigen::Quaterniond& q)
    // {
    // const Eigen::Matrix3d R = quatToRotMat(q);

    // // standard aerospace RPY from rotation matrix:
    // // roll  = atan2(R(2,1), R(2,2))
    // // pitch = asin(-R(2,0))
    // // yaw   = atan2(R(1,0), R(0,0))
    // Eigen::Vector3d rpy;
    // rpy(0) = std::atan2(R(2,1), R(2,2));
    // rpy(1) = std::asin(std::clamp(-R(2,0), -1.0, 1.0));
    // rpy(2) = std::atan2(R(1,0), R(0,0));
    // return rpy;
    // }

    // omega = quatToOmega(quat, quat_dot)
    // Using: qdot = 0.5 * q ⊗ [0,omega]  =>  omega = 2 * (q^{-1} ⊗ qdot).vec
    // static inline Eigen::Vector3d quatToOmega(const Eigen::Quaterniond& q,
    //                                         const Eigen::Quaterniond& qdot)
    // {
    // Eigen::Quaterniond wq = q.conjugate() * qdot; // q^{-1} ⊗ qdot
    // return 2.0 * wq.vec();
    // }

    // -----------------------------
    // MAIN
    // -----------------------------
    int main(int argc, char** argv)
    {
    std::string dataset_root = "../../data/anymalD_grandtour";
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string out_csv    = dataset_root + "/muse/attitude_estimate_muse.csv";

    std::cout << "MUSE attitude (plugin -> offline exe)\n";
    std::cout << "  Sensor CSV: " << sensor_csv << "\n";
    std::cout << "  Out CSV   : " << out_csv << "\n";

    std::ifstream in(sensor_csv);
    if (!in.is_open()) {
        std::cerr << "Error: cannot open " << sensor_csv << "\n";
        return 1;
    }

    std::ofstream out(out_csv);
    if (!out.is_open()) {
        std::cerr << "Error: cannot open " << out_csv << " for writing\n";
        return 1;
    }

    out.setf(std::ios::fixed);
    out << std::setprecision(9);

    // Output header (più ricco del messaggio ROS)
    out << "t_rel,t_abs,"
        << "qw,qx,qy,qz,"
        << "bgx,bgy,bgz,"
        << "roll_deg,pitch_deg,yaw_deg,"
        << "omega_filt_x,omega_filt_y,omega_filt_z\n";

    // ---- Read header
    std::string line;
    if (!std::getline(in, line)) {
        std::cerr << "Error: empty file?\n";
        return 1;
    }

    auto header = splitCSVLine(line);
    auto idx = headerIndex(header);

    // We accept either "t" or "timestamp" for time column
    const bool has_t = idx.count("t") > 0;
    const bool has_timestamp = idx.count("timestamp") > 0;
    if (!has_t && !has_timestamp) {
        std::cerr << "Error: sensor_data.csv must have column 't' or 'timestamp'\n";
        return 1;
    }

    // Required IMU columns (as in your dataset screenshot)
    // imu_wx, imu_wy, imu_wz, imu_ax, imu_ay, imu_az
    const std::vector<std::string> required = {
        "imu_wx","imu_wy","imu_wz","imu_ax","imu_ay","imu_az"
    };
    for (const auto& c : required) {
        if (idx.count(c) == 0) {
        std::cerr << "Error: missing required column '" << c << "'\n";
        return 1;
        }
    }

    // -----------------------------
    // Plugin parameters (offline defaults)
    // -----------------------------
    double ki = 0.02;
    double kp = 10.0;

    Eigen::Matrix3d b_R_imu;
    b_R_imu << -1, 0, 0,
                0, 1, 0,
                0, 0, -1;

    // north_vector default in plugin:
    Eigen::Vector3d m_n(1.0/std::sqrt(3.0), 1.0/std::sqrt(3.0), 1.0/std::sqrt(3.0));

    // gravity_vector default in plugin:
    Eigen::Vector3d f_n(0.0, 0.0, 9.81);

    // Covariances P0/Q/R (plugin expects 6x6). Here we set sane defaults.
    // Se vuoi identico al plugin, puoi caricarli da YAML in futuro.
    Eigen::Matrix<double,6,6> P0 = Eigen::Matrix<double,6,6>::Identity() * 1e-2;
    Eigen::Matrix<double,6,6> Q  = Eigen::Matrix<double,6,6>::Identity() * 1e-4;
    Eigen::Matrix<double,6,6> R  = Eigen::Matrix<double,6,6>::Identity() * 1e-2;

    // -----------------------------
    // Plugin state init (identico)
    // -----------------------------
    double t0 = 0.0;

    Eigen::Matrix<double,7,1> xhat_estimated;
    xhat_estimated << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    xhat_estimated.head<4>() /= xhat_estimated.head<4>().norm();

    // Create filter (identico al plugin)
    state_estimator::AttitudeBiasXKF attitude(t0, xhat_estimated, P0, Q, R, f_n, m_n, ki, kp);

    // -----------------------------
    // Loop CSV
    // -----------------------------
    bool first = true;
    double t_begin_abs = 0.0;
    double t_prev_abs  = 0.0;

    std::size_t count = 0;
    std::size_t skipped = 0;

    while (std::getline(in, line)) {
        if (line.empty()) continue;

        auto row = splitCSVLine(line);
        if ((int)row.size() < (int)header.size()) {
        // allow short rows but must contain needed cols
        // we'll just rely on indexing checks
        }

        try {
        const double t_abs =
            has_t ? getColAsDouble(row, idx, "t")
                : getColAsDouble(row, idx, "timestamp");

        const Eigen::Vector3d omega(
            getColAsDouble(row, idx, "imu_wx"),
            getColAsDouble(row, idx, "imu_wy"),
            getColAsDouble(row, idx, "imu_wz")
        );

        const Eigen::Vector3d acc(
            getColAsDouble(row, idx, "imu_ax"),
            getColAsDouble(row, idx, "imu_ay"),
            getColAsDouble(row, idx, "imu_az")
        );

        if (first) {
            t_begin_abs = t_abs;
            t_prev_abs  = t_abs;
            first = false;
        }

        const double t_rel = t_abs - t_begin_abs;
        const double dt = std::max(0.0, t_abs - t_prev_abs);
        t_prev_abs = t_abs;

        // === Attitude init ===
        Eigen::Quaterniond quat_est;
        quat_est.w() = xhat_estimated(0);
        quat_est.vec() << xhat_estimated(1), xhat_estimated(2), xhat_estimated(3);

        Eigen::Vector3d f_b = b_R_imu * acc;
        Eigen::Vector3d m_b = iit::commons::quatToRotMat(quat_est) * m_n;

        Eigen::Matrix<double,6,1> z;
        z << f_b, m_b;

        attitude.update(t_rel, b_R_imu * omega, z);

        xhat_estimated = attitude.getX();

        Eigen::Matrix<double,7,1> xdot = attitude.calc_f(t_rel, xhat_estimated, b_R_imu * omega);

        Eigen::Quaterniond quat_dot;
        quat_dot.w() = xdot(0);
        quat_dot.vec() << xdot(1), xdot(2), xdot(3);

        Eigen::Vector3d omega_filt = iit::commons::quatToOmega(quat_est, quat_dot);

        const Eigen::Vector3d euler_radians = iit::commons::quatToRPY(quat_est);
        const Eigen::Vector3d euler_degrees = euler_radians * (180.0 / M_PI);

        // gyro bias from state (elements 4..6)
        const Eigen::Vector3d bg(xhat_estimated(4), xhat_estimated(5), xhat_estimated(6));

        // write output
        out << t_rel << "," << t_abs << ","
            << quat_est.w() << "," << quat_est.x() << "," << quat_est.y() << "," << quat_est.z() << ","
            << bg.x() << "," << bg.y() << "," << bg.z() << ","
            << euler_degrees(0) << "," << euler_degrees(1) << "," << euler_degrees(2) << ","
            << omega_filt(0) << "," << omega_filt(1) << "," << omega_filt(2)
            << "\n";

        (void)dt; // dt disponibile se vuoi stamparlo/debug
        count++;
        }
        catch (const std::exception&) {
        skipped++;
        continue;
        }
    }

    std::cout << "Done. Wrote " << count << " rows to " << out_csv
                << " (skipped: " << skipped << ")\n";
    return 0;
}
