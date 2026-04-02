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

// ---- Attitude core
#include "Models/attitude_bias_XKF.hpp"
#include "lib.hpp"
#include "rotations.h"

// ---- Sensor fusion (offline KF)
#include "Models/SensorFusion.hpp"   // state_estimator::KFSensorFusion

#include <chrono>
#include <limits>

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
    for (int i = 0; i < (int)header.size(); ++i) m[header[i]] = i;
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

static bool getColAsBoolTF01(const std::vector<std::string>& row,
                             const std::unordered_map<std::string, int>& idx,
                             const std::string& name)
{
    auto it = idx.find(name);
    if (it == idx.end()) throw std::runtime_error("Missing column: " + name);
    const int j = it->second;
    if (j < 0 || j >= (int)row.size()) throw std::runtime_error("Bad index for: " + name);

    std::string s = row[j];
    // trim spaces
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);

    if (s == "True" || s == "true" || s == "1") return true;
    if (s == "False" || s == "false" || s == "0") return false;

    // sometimes it can be "1.0"/"0.0"
    try {
        const double v = std::stod(s);
        return (v != 0.0);
    } catch (...) {
        throw std::runtime_error("Cannot parse bool for " + name + ": '" + s + "'");
    }
}

// -----------------------------
// Feet kinematics row (from feet_kinematics.csv)
// -----------------------------
struct FeetKinRow {
    double t_abs{};
    Eigen::Vector3d pLF, pRF, pLH, pRH;  // positions (same frame as computed in pinocchio export)
    Eigen::Vector3d vLF, vRF, vLH, vRH;  // linear velocities (LOCAL_WORLD_ALIGNED)
};

static FeetKinRow parseFeetKinRow(const std::vector<std::string>& row,
                                  const std::unordered_map<std::string,int>& idx)
{
    FeetKinRow fk;
    fk.t_abs = getColAsDouble(row, idx, "t");

    auto P = [&](const std::string& pre)->Eigen::Vector3d{
        return Eigen::Vector3d(
            getColAsDouble(row, idx, pre + "_x"),
            getColAsDouble(row, idx, pre + "_y"),
            getColAsDouble(row, idx, pre + "_z")
        );
    };
    fk.pLF = P("p_LF");
    fk.pRF = P("p_RF");
    fk.pLH = P("p_LH");
    fk.pRH = P("p_RH");

    fk.vLF = P("v_LF");
    fk.vRF = P("v_RF");
    fk.vLH = P("v_LH");
    fk.vRH = P("v_RH");
    return fk;
}

// -----------------------------
// MAIN
// -----------------------------
int main(int argc, char** argv)
{
    std::string dataset_root = "../../data/anymalD_grandtour";
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string feet_csv   = dataset_root + "/feet_kinematics.csv";

    const std::string out_att_csv = dataset_root + "/muse/attitude_estimate_muse.csv";
    const std::string out_lo_csv  = dataset_root + "/muse/leg_odometry.csv";
    // const std::string out_fs_csv  = dataset_root + "/muse/fused_state.csv";
    const std::string out_fs_csv  = dataset_root + "/muse/fused_state_bad_init_ori.csv";

    std::cout << "MUSE OFFLINE (single executable)\n";
    std::cout << "  sensor_data : " << sensor_csv << "\n";
    std::cout << "  feet_kin    : " << feet_csv << "\n";
    std::cout << "  out attitude: " << out_att_csv << "\n";
    std::cout << "  out legodom : " << out_lo_csv  << "\n";
    std::cout << "  out fused   : " << out_fs_csv  << "\n";

    std::ifstream in(sensor_csv);
    if (!in.is_open()) { std::cerr << "Error: cannot open " << sensor_csv << "\n"; return 1; }

    std::ifstream fin(feet_csv);
    if (!fin.is_open()) { std::cerr << "Error: cannot open " << feet_csv << "\n"; return 1; }

    std::ofstream out_att(out_att_csv), out_lo(out_lo_csv), out_fs(out_fs_csv);
    if (!out_att.is_open()) { std::cerr << "Error: cannot open " << out_att_csv << " for writing\n"; return 1; }
    if (!out_lo.is_open())  { std::cerr << "Error: cannot open " << out_lo_csv  << " for writing\n"; return 1; }
    if (!out_fs.is_open())  { std::cerr << "Error: cannot open " << out_fs_csv  << " for writing\n"; return 1; }

    out_att.setf(std::ios::fixed); out_att << std::setprecision(9);
    out_lo .setf(std::ios::fixed); out_lo  << std::setprecision(9);
    out_fs .setf(std::ios::fixed); out_fs  << std::setprecision(9);

    // -----------------------------
    // Read headers
    // -----------------------------
    std::string line, fline;

    if (!std::getline(in, line)) { std::cerr << "Error: empty sensor_data.csv?\n"; return 1; }
    auto header = splitCSVLine(line);
    auto idx = headerIndex(header);

    if (!std::getline(fin, fline)) { std::cerr << "Error: empty feet_kinematics.csv?\n"; return 1; }
    auto fheader = splitCSVLine(fline);
    auto fidx = headerIndex(fheader);

    // Time col in sensor
    const bool has_t = idx.count("t") > 0;
    const bool has_timestamp = idx.count("timestamp") > 0;
    if (!has_t && !has_timestamp) {
        std::cerr << "Error: sensor_data.csv must have column 't' or 'timestamp'\n";
        return 1;
    }

    // Required IMU columns
    const std::vector<std::string> required_imu = {"imu_wx","imu_wy","imu_wz","imu_ax","imu_ay","imu_az"};
    for (const auto& c : required_imu) {
        if (idx.count(c) == 0) { std::cerr << "Error: missing '" << c << "' in sensor_data.csv\n"; return 1; }
    }

    // Required contacts
    const std::vector<std::string> required_contacts = {"contact_LF","contact_RF","contact_LH","contact_RH"};
    for (const auto& c : required_contacts) {
        if (idx.count(c) == 0) { std::cerr << "Error: missing '" << c << "' in sensor_data.csv\n"; return 1; }
    }

    // feet required
    const std::vector<std::string> required_feet = {
        "t",
        "p_LF_x","p_LF_y","p_LF_z","p_RF_x","p_RF_y","p_RF_z","p_LH_x","p_LH_y","p_LH_z","p_RH_x","p_RH_y","p_RH_z",
        "v_LF_x","v_LF_y","v_LF_z","v_RF_x","v_RF_y","v_RF_z","v_LH_x","v_LH_y","v_LH_z","v_RH_x","v_RH_y","v_RH_z"
    };
    for (const auto& c : required_feet) {
        if (fidx.count(c) == 0) { std::cerr << "Error: missing '" << c << "' in feet_kinematics.csv\n"; return 1; }
    }

    // -----------------------------
    // OUTPUT headers
    // -----------------------------
    out_att << "t_rel,t_abs,"
            << "qw,qx,qy,qz,"
            << "bgx,bgy,bgz,"
            << "roll_deg,pitch_deg,yaw_deg,"
            << "omega_filt_x,omega_filt_y,omega_filt_z\n";

    out_lo << "t_rel,t_abs,"
           << "v_base_b_x,v_base_b_y,v_base_b_z,"
           << "v_base_w_x,v_base_w_y,v_base_w_z\n";

    out_fs << "t_rel,t_abs,px,py,pz,vx,vy,vz,qw,qx,qy,qz\n";

    // -----------------------------
    // ATTITUDE PARAMETERS / INIT 
    // -----------------------------
    double ki = 0.02; // 0.05;
    double kp = 10.0;// 5.0;

    Eigen::Matrix3d b_R_imu;
    Eigen::Quaterniond b_quat_imu;
    b_quat_imu.w() = 0.0;
    b_quat_imu.x() = 0.0;
    b_quat_imu.y() = 1.0;
    b_quat_imu.z() = 0.0;
    b_R_imu = iit::commons::quatToRotMat(b_quat_imu.normalized()).transpose();

    Eigen::Vector3d m_n(1.0/std::sqrt(3.0), 1.0/std::sqrt(3.0), 1.0/std::sqrt(3.0));
    Eigen::Vector3d f_n(0.0, 0.0, 9.81);

    Eigen::Matrix<double,6,6> P0 = Eigen::Matrix<double,6,6>::Identity() * 1e-12;
    Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Identity() * 1e-15;
    Eigen::Matrix<double,6,6> Ratt = Eigen::Matrix<double,6,6>::Identity() * 1e-6;

    double t0_att = 0.0;
    Eigen::Matrix<double,7,1> xhat_estimated;
    xhat_estimated << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0; // bad initialization
    // xhat_estimated << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // q(w,x,y,z) in this order is the correct initialization
    xhat_estimated.head<4>() /= xhat_estimated.head<4>().norm();

    state_estimator::AttitudeBiasXKF attitude(t0_att, xhat_estimated, P0, Q, Ratt, f_n, m_n, ki, kp);

    // -----------------------------
    // SENSOR FUSION INIT (KF)
    // -----------------------------
    double t0 = 0.0;
    Eigen::Matrix<double,6,1> x0; x0.setZero();
    x0 (0) = -0.25565; // initial position (x) from dataset
    x0 (1) = 0.00255;  // initial position (y) from dataset
    x0 (2) = 0.07672;  // initial position (z) from dataset
    x0 (3) = 0.0;      // initial velocity (vx) from dataset
    x0 (4) = 0.0;      // initial velocity (vy) from dataset
    x0 (5) = 0.0;      // initial velocity (vz) from dataset

    // Tune these (or read from YAML).
    Eigen::Matrix<double,6,6> Psf = Eigen::Matrix<double,6,6>::Identity() * 1e-14;
    Eigen::Matrix<double,6,6> Qsf = Eigen::Matrix<double,6,6>::Identity() * 1e-14;
    Eigen::Matrix<double,3,3> Rsf = Eigen::Matrix<double,3,3>::Identity() * 5e-16;

    state_estimator::KFSensorFusion kf(t0, x0, Psf, Qsf, Rsf, false, false);

    // -----------------------------
    // Loop
    // -----------------------------
    bool first = true;
    double t_begin_abs = 0.0;
    double t_prev_abs  = 0.0;

    std::size_t count = 0;
    std::size_t skipped = 0;

    double total_step_us = 0.0;
    double min_step_us = std::numeric_limits<double>::infinity();
    double max_step_us = 0.0;

    while (std::getline(in, line)) {
        if (line.empty()) continue;

        // lockstep feet row
        if (!std::getline(fin, fline)) break;
        if (fline.empty()) continue;

        auto row  = splitCSVLine(line);
        auto frow = splitCSVLine(fline);

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

            const bool stance_lf = getColAsBoolTF01(row, idx, "contact_LF");
            const bool stance_rf = getColAsBoolTF01(row, idx, "contact_RF");
            const bool stance_lh = getColAsBoolTF01(row, idx, "contact_LH");
            const bool stance_rh = getColAsBoolTF01(row, idx, "contact_RH");

            // parse feet row
            FeetKinRow fk = parseFeetKinRow(frow, fidx);
            (void)fk.t_abs; // we assume lockstep, but you can assert |fk.t_abs - t_abs| < eps

            auto start = std::chrono::high_resolution_clock::now();

            if (first) {
                t_begin_abs = t_abs;
                t_prev_abs  = t_abs;
                first = false;
            }

            const double t_rel = t_abs - t_begin_abs;
            const double dt = std::max(0.0, t_abs - t_prev_abs);
            t_prev_abs = t_abs;
            (void)dt;

            // =========================================================
            // 1) ATTITUDE
            // =========================================================
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

            const Eigen::Vector3d bg(xhat_estimated(4), xhat_estimated(5), xhat_estimated(6));

            // write attitude CSV
            out_att << t_rel << "," << t_abs << ","
                    << quat_est.w() << "," << quat_est.x() << "," << quat_est.y() << "," << quat_est.z() << ","
                    << bg.x() << "," << bg.y() << "," << bg.z() << ","
                    << euler_degrees(0) << "," << euler_degrees(1) << "," << euler_degrees(2) << ","
                    << omega_filt(0) << "," << omega_filt(1) << "," << omega_filt(2)
                    << "\n";

            // =========================================================
            // 2) LEG ODOMETRY (using precomputed feet_kinematics.csv)
            //      omega_b = b_R_imu * omega
            //      v_leg_b = -(vfoot + omega_b x pfoot)
            //      v_base_b = weighted avg by stance
            //      v_base_w = w_R_b * v_base_b
            // =========================================================
            const Eigen::Vector3d omega_b = b_R_imu * omega;

            const Eigen::Vector3d v_lf_b = -(fk.vLF + omega_b.cross(fk.pLF));
            const Eigen::Vector3d v_rf_b = -(fk.vRF + omega_b.cross(fk.pRF));
            const Eigen::Vector3d v_lh_b = -(fk.vLH + omega_b.cross(fk.pLH));
            const Eigen::Vector3d v_rh_b = -(fk.vRH + omega_b.cross(fk.pRH));

            const double w_lf = stance_lf ? 1.0 : 0.0;
            const double w_rf = stance_rf ? 1.0 : 0.0;
            const double w_lh = stance_lh ? 1.0 : 0.0;
            const double w_rh = stance_rh ? 1.0 : 0.0;
            double sum = w_lf + w_rf + w_lh + w_rh;
            if (sum < 1.0) sum = 1.0; // avoid crazy spikes when flying; keep last would be better, but this is safe

            const Eigen::Vector3d v_base_b =
                (w_lf*v_lf_b + w_rf*v_rf_b + w_lh*v_lh_b + w_rh*v_rh_b) / (sum + 1e-5);

            const Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
            const Eigen::Vector3d v_base_w = w_R_b * v_base_b;

            out_lo << t_rel << "," << t_abs << ","
                   << v_base_b.x() << "," << v_base_b.y() << "," << v_base_b.z() << ","
                   << v_base_w.x() << "," << v_base_w.y() << "," << v_base_w.z()
                   << "\n";

            // =========================================================
            // 3) SENSOR FUSION (KF)
            //      f_b = b_R_imu * acc
            //      u_w = w_R_b * f_b + gravity(0,0,-9.81)
            //      z   = v_base_w
            // =========================================================
            Eigen::Vector3d gravity(0.0, 0.0, -9.81);
            const Eigen::Vector3d u_w = w_R_b * f_b + gravity;

            kf.predict(t_rel, u_w);
            kf.update(t_rel, v_base_w);

            const auto xhat = kf.getX(); // [px py pz vx vy vz]

            out_fs << t_rel << "," << t_abs << ","
                   << xhat(0) << "," << xhat(1) << "," << xhat(2) << ","
                   << xhat(3) << "," << xhat(4) << "," << xhat(5) << ","
                   << quat_est.w() << "," << quat_est.x() << "," << quat_est.y() << "," << quat_est.z()
                   << "\n";

            auto end = std::chrono::high_resolution_clock::now();
            const double elapsed_us = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(end - start).count();

            total_step_us += elapsed_us;
            min_step_us = std::min(min_step_us, elapsed_us);
            max_step_us = std::max(max_step_us, elapsed_us);

            count++;
        }
        catch (const std::exception&) {
            skipped++;
            continue;
        }
    }

    if (count > 0) {
    const double avg_step_us = total_step_us / static_cast<double>(count);
    std::cout << std::fixed << std::setprecision(3)
              << "Step timing [us]: avg=" << avg_step_us
              << ", min=" << min_step_us
              << ", max=" << max_step_us << "\n"
              << "Step timing [ms]: avg=" << (avg_step_us / 1000.0)
              << ", min=" << (min_step_us / 1000.0)
              << ", max=" << (max_step_us / 1000.0) << "\n";
    }   
    return 0;
}
