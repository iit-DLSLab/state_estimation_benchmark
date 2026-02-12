// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.

#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "invariant_smoother/estimator/InvariantSmoother.hpp"
#include "invariant_smoother/utility/SaveFile.hpp"

// -----------------------------
// Utils CSV
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

static int getColAsBool01(const std::vector<std::string>& row,
                          const std::unordered_map<std::string, int>& idx,
                          const std::string& name)
{
    auto it = idx.find(name);
    if (it == idx.end()) throw std::runtime_error("Missing column: " + name);
    const int j = it->second;
    if (j < 0 || j >= (int)row.size()) throw std::runtime_error("Bad index for: " + name);

    const std::string& s = row[j];
    if (s == "True" || s == "true" || s == "1") return 1;
    if (s == "False" || s == "false" || s == "0") return 0;
    return (std::stod(s) != 0.0) ? 1 : 0;
}

// -----------------------------
// FeetStream: legge feet_kinematics.csv (lockstep)
// -----------------------------
struct FeetRow {
    double t_abs = 0.0;
    std::array<double, 12> p{};   // LF,RF,LH,RH (3 ciascuno)
    std::array<double, 36> J{};   // LF,RF,LH,RH (9 ciascuno, row-major 3x3)
};

class FeetStream {
public:
    explicit FeetStream(const std::string& path)
    {
        in_.open(path);
        if (!in_.is_open()) throw std::runtime_error("Cannot open feet CSV: " + path);

        std::string line;
        if (!std::getline(in_, line)) throw std::runtime_error("Empty feet CSV: " + path);
        header_ = splitCSVLine(line);
        idx_ = headerIndex(header_);
    }

    bool next(FeetRow& fr)
    {
        std::string line;
        if (!std::getline(in_, line)) return false;
        if (line.empty()) return next(fr);

        auto row = splitCSVLine(line);

        fr.t_abs = getColAsDouble(row, idx_, "t");

        const char* legs[4] = {"LF","RF","LH","RH"};

        int k = 0;
        for (int leg = 0; leg < 4; ++leg) {
            fr.p[k++] = getColAsDouble(row, idx_, std::string("p_") + legs[leg] + "_x");
            fr.p[k++] = getColAsDouble(row, idx_, std::string("p_") + legs[leg] + "_y");
            fr.p[k++] = getColAsDouble(row, idx_, std::string("p_") + legs[leg] + "_z");
        }

        int j = 0;
        for (int leg = 0; leg < 4; ++leg) {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    fr.J[j++] = getColAsDouble(
                        row, idx_,
                        std::string("J_") + legs[leg] + "_" + std::to_string(r) + std::to_string(c)
                    );
                }
            }
        }

        return true;
    }

private:
    std::ifstream in_;
    std::vector<std::string> header_;
    std::unordered_map<std::string,int> idx_;
};

// -----------------------------
// Remap: da (sensor row + feet row) -> strutture IS
// Il loro main usa:
//  Sensor_(0:2)=gyro, Sensor_(3:5)=acc, Sensor_(6:17)=jnt_pos, Sensor_(18:29)=jnt_vel
//  Contact_ (4x1 bool)
//  forkin_set_.forkin_position (size 4) e forkin_set_.forkin_jacobian (size 4)
// -----------------------------
static void fillInvariantSmootherInputs(
    const std::vector<std::string>& srow,
    const std::unordered_map<std::string,int>& sidx,
    const FeetRow& feet,
    Eigen::Matrix<double,30,1>& Sensor_,
    Eigen::Matrix<bool,4,1>& Contact_,
    MEAS_FORWARD_KINEMATICS& forkin_set_)
{
    Sensor_.setZero();
    Contact_.setZero();

    // --- IMU
    Sensor_.segment<3>(0) <<
        getColAsDouble(srow, sidx, "imu_wx"),
        getColAsDouble(srow, sidx, "imu_wy"),
        getColAsDouble(srow, sidx, "imu_wz");

    Sensor_.segment<3>(3) <<
        getColAsDouble(srow, sidx, "imu_ax"),
        getColAsDouble(srow, sidx, "imu_ay"),
        getColAsDouble(srow, sidx, "imu_az");

    // --- JOINTS: nel tuo sensor_data.csv hai LF RF LH RH.
    // Tu vuoi alimentare lo smoother nel suo ordine (come da screenshot): RR, RL, FL, FR.
    // RR=RH, RL=LH, FL=LF, FR=RF
    const char* JO_POS[12] = {
        "joint_pos_RH_HAA","joint_pos_RH_HFE","joint_pos_RH_KFE", // RR
        "joint_pos_LH_HAA","joint_pos_LH_HFE","joint_pos_LH_KFE", // RL
        "joint_pos_LF_HAA","joint_pos_LF_HFE","joint_pos_LF_KFE", // FL
        "joint_pos_RF_HAA","joint_pos_RF_HFE","joint_pos_RF_KFE"  // FR
    };
    const char* JO_VEL[12] = {
        "joint_vel_RH_HAA","joint_vel_RH_HFE","joint_vel_RH_KFE", // RR
        "joint_vel_LH_HAA","joint_vel_LH_HFE","joint_vel_LH_KFE", // RL
        "joint_vel_LF_HAA","joint_vel_LF_HFE","joint_vel_LF_KFE", // FL
        "joint_vel_RF_HAA","joint_vel_RF_HFE","joint_vel_RF_KFE"  // FR
    };

    for (int i = 0; i < 12; ++i) Sensor_(6  + i) = getColAsDouble(srow, sidx, JO_POS[i]);
    for (int i = 0; i < 12; ++i) Sensor_(18 + i) = getColAsDouble(srow, sidx, JO_VEL[i]);

    // --- CONTACTS: RR,RL,FL,FR => RH,LH,LF,RF
    const bool c_rr = getColAsBool01(srow, sidx, "contact_RH") != 0;
    const bool c_rl = getColAsBool01(srow, sidx, "contact_LH") != 0;
    const bool c_fl = getColAsBool01(srow, sidx, "contact_LF") != 0;
    const bool c_fr = getColAsBool01(srow, sidx, "contact_RF") != 0;

    Contact_(0) = c_rr;
    Contact_(1) = c_rl;
    Contact_(2) = c_fl;
    Contact_(3) = c_fr;

    // --- FORKIN POS + JAC
    // feet.p è LF,RF,LH,RH. Rimappa in RR,RL,FL,FR => RH,LH,LF,RF
    auto pickP = [&](int legLF_RF_LH_RH) -> Eigen::Vector3d {
        return Eigen::Vector3d(
            feet.p[legLF_RF_LH_RH*3 + 0],
            feet.p[legLF_RF_LH_RH*3 + 1],
            feet.p[legLF_RF_LH_RH*3 + 2]
        );
    };

    auto pickJ = [&](int legLF_RF_LH_RH) -> Eigen::Matrix3d {
        Eigen::Matrix3d J;
        const int base = legLF_RF_LH_RH * 9;
        int k = 0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                J(r,c) = feet.J[base + k++];
        return J;
    };

    // Attenzione: i nomi "forkin_position/jacobian" sono quelli del loro struct.
    // Io assumo che siano std::vector con size=4.
    forkin_set_.forkin_position.resize(4);
    forkin_set_.forkin_jacobian.resize(4);

    forkin_set_.forkin_position[0] = pickP(3); // RR <- RH
    forkin_set_.forkin_position[1] = pickP(2); // RL <- LH
    forkin_set_.forkin_position[2] = pickP(0); // FL <- LF
    forkin_set_.forkin_position[3] = pickP(1); // FR <- RF

    forkin_set_.forkin_jacobian[0] = pickJ(3); // RR <- RH
    forkin_set_.forkin_jacobian[1] = pickJ(2); // RL <- LH
    forkin_set_.forkin_jacobian[2] = pickJ(0); // FL <- LF
    forkin_set_.forkin_jacobian[3] = pickJ(1); // FR <- RF
}

// -----------------------------
// MAIN
// -----------------------------
static const std::string DEFAULT_DATASET_ROOT = "../../data/anymalD_grandtour";

InvariantSmoother estimator_IS;

int main(int argc, char** argv)
{
    std::string dataset_root = DEFAULT_DATASET_ROOT;
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string feet_csv   = dataset_root + "/feet_kinematics.csv";

    const std::string out_dir    = dataset_root + "/invariant_smoother";
    const std::string out_csv    = out_dir + "/fused_state.csv";

    std::cout << "Invariant Smoother (offline, using remapped inputs)\n"
              << "  Sensor: " << sensor_csv << "\n"
              << "  Feet  : " << feet_csv   << "\n"
              << "  Out   : " << out_csv    << "\n";

    std::ifstream in(sensor_csv);
    if (!in.is_open()) {
        std::cerr << "Error: cannot open " << sensor_csv << "\n";
        return 1;
    }

    FeetStream feet(feet_csv);
    FeetRow fr;

    std::string line;
    if (!std::getline(in, line)) {
        std::cerr << "Error: empty sensor_data.csv\n";
        return 1;
    }
    auto sheader = splitCSVLine(line);
    auto sidx = headerIndex(sheader);

    // --- Parametri IS (copiati dal loro main)
    double dt = 0.005;
    int starting_point = 1;
    bool SR = false;
    double slip_thr = 0.48;
    bool VCC = false;
    double cov_amplifier = 1;

    int max_backpp_no = 1;
    double backpp_rate = 0.5;
    int max_it_no = 1;
    double convergence_cond = 1e-3;

    double gyro_exp = -6, acc_exp = -2, slip_exp = -1.3, contact_exp = -4, encoder_exp = -8;
    double bg_exp = -10, ba_exp = -10;
    double pri_ori_exp = -8, pri_vel_exp = -8, pri_pos_exp = -8;
    double pri_bg_exp = -10, pri_ba_exp = -10;

    EstimatorCovariances cov;
    cov.cov_gyro_diagonal << std::pow(10, gyro_exp), std::pow(10, gyro_exp), std::pow(10, gyro_exp);
    cov.cov_acc_diagonal  << std::pow(10, acc_exp),  std::pow(10, acc_exp),  std::pow(10, acc_exp);
    cov.cov_slip_diagonal << std::pow(10, slip_exp), std::pow(10, slip_exp), std::pow(10, slip_exp);
    cov.cov_contact_diagonal << std::pow(10, contact_exp), std::pow(10, contact_exp), std::pow(10, contact_exp);
    cov.cov_enc_diagonal  << std::pow(10, encoder_exp), std::pow(10, encoder_exp), std::pow(10, encoder_exp);
    cov.cov_bias_gyro_diagonal << std::pow(10, bg_exp), std::pow(10, bg_exp), std::pow(10, bg_exp);
    cov.cov_bias_acc_diagonal  << std::pow(10, ba_exp), std::pow(10, ba_exp), std::pow(10, ba_exp);
    cov.cov_prior_orientation_diagonal << std::pow(10, pri_ori_exp), std::pow(10, pri_ori_exp), std::pow(10, pri_ori_exp);
    cov.cov_prior_velocity_diagonal    << std::pow(10, pri_vel_exp), std::pow(10, pri_vel_exp), std::pow(10, pri_vel_exp);
    cov.cov_prior_position_diagonal    << std::pow(10, pri_pos_exp), std::pow(10, pri_pos_exp), std::pow(10, pri_pos_exp);
    cov.cov_prior_bias_gyro_diagonal   << std::pow(10, pri_bg_exp), std::pow(10, pri_bg_exp), std::pow(10, pri_bg_exp);
    cov.cov_prior_bias_acc_diagonal    << std::pow(10, pri_ba_exp), std::pow(10, pri_ba_exp), std::pow(10, pri_ba_exp);

    Eigen::Matrix<double,16,1> x0;
    x0 << 0.0,0.0,0.0,   // px py pz
          1.0,0.0,0.0,0.0, // q(w,x,y,z)
          0.0,0.0,0.0,   // vx, vy, vz
          0.0,0.0,0.0,   // bgx, bgy, bgz
          0.0,0.0,0.0;   // bax, bay, baz

    estimator_IS.estimator_common_struct_.leg_no = 4;
    estimator_IS.Optimization_Epsilon = convergence_cond;
    estimator_IS.Max_Iteration = max_it_no;
    estimator_IS.Max_backpropagate_num = max_backpp_no;
    estimator_IS.backppgn_rate = backpp_rate;

    estimator_IS.NUM_OF_TRASH_DATA = starting_point;
    estimator_IS.slip_rejection_mode = SR;
    estimator_IS.slip_threshold = slip_thr;
    estimator_IS.variable_contact_cov_mode = VCC;
    estimator_IS.cov_amplifier = cov_amplifier;

    estimator_IS.Retract_All_flag = false;
    estimator_IS.Initialize(dt, cov, x0);

    // --- Loop
    Eigen::Matrix<double,30,1> Sensor_;
    Eigen::Matrix<bool,4,1> Contact_;
    ROBOT_STATES state_;
    MEAS_FORWARD_KINEMATICS forkin_set_;

    // NEW

    std::vector<ROBOT_STATES> state_history;
    std::vector<double> t_abs_history;

    state_history.reserve(200000);
    t_abs_history.reserve(200000);

    std::size_t n = 0;
    std::size_t skipped = 0;

    while (std::getline(in, line)) {
        if (line.empty()) continue;

        if (!feet.next(fr)) {
            std::cerr << "Feet stream ended early.\n";
            break;
        }

        auto srow = splitCSVLine(line);

        try {
            // timestamp dal sensor csv
            const double t_abs = getColAsDouble(srow, sidx, "t");
            t_abs_history.push_back(t_abs);

            // rimappa e chiama smoother
            fillInvariantSmootherInputs(srow, sidx, fr, Sensor_, Contact_, forkin_set_);
            estimator_IS.Onestep(Sensor_, Contact_, forkin_set_, state_);

            state_history.push_back(state_);
            n++;
        } catch (const std::exception& e) {
            skipped++;
            continue;
        }
    }

    // --- Save
    std::filesystem::create_directories(out_dir);
    SaveFile::writeCSV(out_csv, state_history, t_abs_history);

    std::cout << "Done. Processed " << n << " samples (skipped " << skipped << ").\n"
              << "Saved: " << out_csv << "\n";
    return 0;
}
