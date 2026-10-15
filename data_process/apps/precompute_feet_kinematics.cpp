#include "benchmark/DatasetIO.hpp"
#include "benchmark/FootKinematics.hpp"

#include <Eigen/Core>

#include <algorithm>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

// =====================
// Defaults (edit once)
// =====================
static const std::string DEFAULT_DATASET_ROOT = "../../data/anymalD_grandtour";
static const std::string DEFAULT_URDF_PATH    = "../../models/anymalD/anymal.urdf"; 
static const std::string DEFAULT_SENSOR_CSV   = DEFAULT_DATASET_ROOT + "/sensor_data.csv";
static const std::string DEFAULT_OUT_CSV      = DEFAULT_DATASET_ROOT + "/feet_kinematics.csv"; // need to check if we need another one for the smoother version

static void write_header(std::ofstream& out)
{
    out
      << "t,"
      << "p_LF_x,p_LF_y,p_LF_z,p_RF_x,p_RF_y,p_RF_z,p_LH_x,p_LH_y,p_LH_z,p_RH_x,p_RH_y,p_RH_z,"
      << "v_LF_x,v_LF_y,v_LF_z,v_RF_x,v_RF_y,v_RF_z,v_LH_x,v_LH_y,v_LH_z,v_RH_x,v_RH_y,v_RH_z,"
      << "J_LF_00,J_LF_01,J_LF_02,J_LF_10,J_LF_11,J_LF_12,J_LF_20,J_LF_21,J_LF_22,"
      << "J_RF_00,J_RF_01,J_RF_02,J_RF_10,J_RF_11,J_RF_12,J_RF_20,J_RF_21,J_RF_22,"
      << "J_LH_00,J_LH_01,J_LH_02,J_LH_10,J_LH_11,J_LH_12,J_LH_20,J_LH_21,J_LH_22,"
      << "J_RH_00,J_RH_01,J_RH_02,J_RH_10,J_RH_11,J_RH_12,J_RH_20,J_RH_21,J_RH_22"
      << "\n";
}

int main(int argc, char** argv)
{
    try {
        // --------------------------
        // "Smart defaults" override
        // --------------------------
        // Usage:
        //   ./precompute_feet_kinematics
        //   ./precompute_feet_kinematics <sensor_csv>
        //   ./precompute_feet_kinematics <sensor_csv> <urdf_path>
        //   ./precompute_feet_kinematics <sensor_csv> <urdf_path> <out_csv>
        std::string sensor_csv = DEFAULT_SENSOR_CSV;
        std::string urdf_path  = DEFAULT_URDF_PATH;
        std::string out_csv    = DEFAULT_OUT_CSV;

        if (argc > 1) sensor_csv = argv[1];
        if (argc > 2) urdf_path  = argv[2];
        if (argc > 3) out_csv    = argv[3];

        std::cout << "Sensor CSV: " << sensor_csv << "\n";
        std::cout << "URDF path : " << urdf_path  << "\n";
        std::cout << "Out CSV   : " << out_csv    << "\n";

        // ---- EDIT THESE to match your URDF ----
        benchmark::FootKinematicsConfig kin_config;
        kin_config.urdf_path = urdf_path;
        // Foot frames (must exist as FRAME names in URDF)
        kin_config.foot_frames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
        // Joint names in the SAME ORDER as CSV:
        // LF(HAA,HFE,KFE), RF(...), LH(...), RH(...)
        kin_config.joint_names = {
            "LF_HAA", "LF_HFE", "LF_KFE",
            "RF_HAA", "RF_HFE", "RF_KFE",
            "LH_HAA", "LH_HFE", "LH_KFE",
            "RH_HAA", "RH_HFE", "RH_KFE"
        };
        // Results expressed in the fixed-base model world frame (URDF root link).
        kin_config.base_frame = "";

        // Load measurements (12 joints in CSV)
        auto meas = benchmark::loadMeasurements(sensor_csv, 12);
        if (meas.empty()) throw std::runtime_error("No measurements loaded from sensor CSV.");

        std::cout << "Loaded " << meas.size() << " measurements\n";
        for (size_t i = 0; i < std::min<size_t>(5, meas.size()); ++i) {
            std::cout << "meas[" << i << "].t = " << std::setprecision(17) << meas[i].t << "\n";
        }

        // Load Pinocchio model (fixed-base); throws with the list of available
        // frames/joints if a name is wrong.
        benchmark::FootKinematicsModel kinematics(kin_config);

        // Open output file
        std::ofstream out(out_csv);
        if (!out.is_open()) throw std::runtime_error("Cannot open output CSV: " + out_csv);
        write_header(out);

        out.setf(std::ios::fixed);
        out << std::setprecision(9);

        std::cout << "Computing kinematics for " << meas.size() << " samples...\n";

        benchmark::FootKinematicsSample fk;
        Eigen::Matrix<double, 12, 1> q, dq;

        // Main loop
        for (const auto& s : meas)
        {
            q = s.q.head<12>();
            dq = s.dq.head<12>();
            kinematics.compute(q, dq, fk);

            // Write row
            out << s.t;

            // positions (12)
            for (int leg = 0; leg < 4; ++leg) {
                out << "," << fk.position[leg].x() << "," << fk.position[leg].y() << "," << fk.position[leg].z();
            }

            // velocities (12)
            for (int leg = 0; leg < 4; ++leg) {
                out << "," << fk.velocity[leg].x() << "," << fk.velocity[leg].y() << "," << fk.velocity[leg].z();
            }

            // Jacobians (36), row-major per leg
            for (int leg = 0; leg < 4; ++leg) {
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        out << "," << fk.jacobian[leg](r,c);
                    }
                }
            }

            out << "\n";
        }

        std::cout << "Done. Wrote: " << out_csv << "\n";
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Usage:\n"
                  << "  ./precompute_feet_kinematics\n"
                  << "  ./precompute_feet_kinematics <sensor_csv>\n"
                  << "  ./precompute_feet_kinematics <sensor_csv> <urdf_path>\n"
                  << "  ./precompute_feet_kinematics <sensor_csv> <urdf_path> <out_csv>\n";
        return 1;
    }
}