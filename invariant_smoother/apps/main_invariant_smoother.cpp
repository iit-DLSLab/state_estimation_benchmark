/* 
    Offline Invariant Smoother implementation for the Anymal D Grand Tour dataset.
    Original code of the Invariant Smoother: https://github.com/DrcdKAIST/invariant_smoother   
*/

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
// FeetStream: reading feet_kinematics.csv (lockstep)
// -----------------------------
struct FeetRow {
    double t_abs = 0.0;
    std::array<double, 12> p{};   // LF,RF,LH,RH (3 each)
    std::array<double, 36> J{};   // LF,RF,LH,RH (9 each, row-major 3x3)
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
//  Remap: from (sensor row + feet row) -> structure IS
//  Original main uses:
//  Sensor_(0:2)=gyro, Sensor_(3:5)=acc, Sensor_(6:17)=jnt_pos, Sensor_(18:29)=jnt_vel
//  Contact_ (4x1 bool)
//  forkin_set_.forkin_position (size 4) and forkin_set_.forkin_jacobian (size 4)
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
        -getColAsDouble(srow, sidx, "imu_wx"),
        getColAsDouble(srow, sidx, "imu_wy"),
        -getColAsDouble(srow, sidx, "imu_wz");

    Sensor_.segment<3>(3) <<
        -getColAsDouble(srow, sidx, "imu_ax"),
        getColAsDouble(srow, sidx, "imu_ay"),
        -getColAsDouble(srow, sidx, "imu_az");

    // --- JOINTS: in our sensor_data.csv we have LF RF LH RH.
    // To align to the smoother order: RR, RL, FL, FR (as per convention of the smoother in EstimatorCommonStruct).
    // RR=RH, RL=LH, FL=LF, FR=RF
    const char* JO_POS[12] = {
        "joint_pos_RH_HAA","joint_pos_RH_HFE","joint_pos_RH_KFE",  // RR
        "joint_pos_LH_HAA","joint_pos_LH_HFE","joint_pos_LH_KFE",  // RL
        "joint_pos_RF_HAA","joint_pos_RF_HFE","joint_pos_RF_KFE",  // FR
        "joint_pos_LF_HAA","joint_pos_LF_HFE","joint_pos_LF_KFE"   // FL
    };
    const char* JO_VEL[12] = {
        "joint_vel_RH_HAA","joint_vel_RH_HFE","joint_vel_RH_KFE",  // RR
        "joint_vel_LH_HAA","joint_vel_LH_HFE","joint_vel_LH_KFE",  // RL
        "joint_vel_RF_HAA","joint_vel_RF_HFE","joint_vel_RF_KFE",  // FR
        "joint_vel_LF_HAA","joint_vel_LF_HFE","joint_vel_LF_KFE",  // FL
    };

    // contacts: RH, LH, RF, LF
    Contact_(0) = (getColAsBool01(srow, sidx, "contact_RH") != 0);
    Contact_(1) = (getColAsBool01(srow, sidx, "contact_LH") != 0);
    Contact_(2) = (getColAsBool01(srow, sidx, "contact_LF") != 0);
    Contact_(3) = (getColAsBool01(srow, sidx, "contact_RF") != 0);


    for (int i = 0; i < 12; ++i) Sensor_(6  + i) = getColAsDouble(srow, sidx, JO_POS[i]);
    for (int i = 0; i < 12; ++i) Sensor_(18 + i) = getColAsDouble(srow, sidx, JO_VEL[i]);


    // --- FORKIN POS + JAC
    // feet.p is LF,RF,LH,RH. Remap in RR,RL,FL,FR => RH,LH,LF,RF
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

    // Remapping legs: RR,RL,FL,FR => RH,LH,LF,RF 
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

    // --- IS Parameters 
    double dt_init = 0.0025; // 400Hz
    int starting_point = 1;
    bool SR = false;
    double slip_thr = 0.48;
    bool VCC = false;
    double cov_amplifier = 1;

    int max_backpp_no = 1;      //10;
    double backpp_rate = 0.5;
    int max_it_no = 1;          //10;
    double convergence_cond = 1e-3;

    double gyro_exp = -4, acc_exp = -1, slip_exp = -1.3, contact_exp = -4, encoder_exp = -5;    // smooth
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
    x0 << 0.0,0.0,0.0,          // px py pz
          1.0,0.0,0.0,0.0,      // q(w,x,y,z) 
          0.0,0.0,0.0,          // vx, vy, vz
          0.0,0.0,0.0,          // bgx, bgy, bgz
          0.0,0.0,0.0;          // bax, bay, baz

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
    estimator_IS.Initialize(dt_init, cov, x0);

    double t_prev = -1.0;
    bool first = true;

    // --- Loop
    Eigen::Matrix<double,30,1> Sensor_;
    Eigen::Matrix<bool,4,1> Contact_;
    ROBOT_STATES state_;
    MEAS_FORWARD_KINEMATICS forkin_set_;

    std::vector<ROBOT_STATES> state_history;
    std::vector<double> t_abs_history;

    state_history.reserve(200000);
    t_abs_history.reserve(200000);

    std::size_t n = 0;
    std::size_t skipped = 0;
    double total_onestep_us = 0.0;
    double min_onestep_us = std::numeric_limits<double>::infinity();
    double max_onestep_us = 0.0;

    while (std::getline(in, line)) {
        if (line.empty()) continue;

        if (!feet.next(fr)) {
            std::cerr << "Feet stream ended early.\n";
            break;
        }
       
        auto srow = splitCSVLine(line);

        double t_current = getColAsDouble(srow, sidx, "t");

        if (first) {
            t_prev = t_current;
            first = false;
            // continue;
        }

        double dt = t_current - t_prev;
        t_prev = t_current;

        try {
            // timestamp from sensor_data csv
            const double t_abs = getColAsDouble(srow, sidx, "t");
            t_abs_history.push_back(t_abs);

            // remap and call the smoother
            fillInvariantSmootherInputs(srow, sidx, fr, Sensor_, Contact_, forkin_set_);
            estimator_IS.estimator_common_struct_.dt = dt;

            auto start = std::chrono::high_resolution_clock::now();
            estimator_IS.Onestep(Sensor_, Contact_, forkin_set_, state_);
            auto end = std::chrono::high_resolution_clock::now();
            const double elapsed_us =
                std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(end - start).count();
            total_onestep_us += elapsed_us;
            min_onestep_us = std::min(min_onestep_us, elapsed_us);
            max_onestep_us = std::max(max_onestep_us, elapsed_us);

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

    std::cout << "Done. Processed " << n << " samples (skipped " << skipped << ").\n";
    if (n > 0) {
        const double avg_onestep_us = total_onestep_us / static_cast<double>(n);
        std::cout << std::fixed << std::setprecision(3)
                  << "Onestep timing [us]: avg=" << avg_onestep_us
                  << ", min=" << min_onestep_us
                  << ", max=" << max_onestep_us << "\n"
                  << "Onestep timing [ms]: avg=" << (avg_onestep_us / 1000.0)
                  << ", min=" << (min_onestep_us / 1000.0)
                  << ", max=" << (max_onestep_us / 1000.0) << "\n";
    }
    std::cout << "Saved: " << out_csv << "\n";
    return 0;
}
