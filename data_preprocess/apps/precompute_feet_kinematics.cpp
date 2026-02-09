#include "benchmark/DatasetIO.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <Eigen/Core>

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
static const std::string DEFAULT_OUT_CSV      = DEFAULT_DATASET_ROOT + "/feet_kinematics.csv";

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

static void print_available_frames(const pinocchio::Model& model)
{
    std::cerr << "Available frames:\n";
    for (size_t i = 0; i < model.frames.size(); ++i) {
        std::cerr << "  [" << i << "] " << model.frames[i].name << "\n";
    }
}

static void print_available_joints(const pinocchio::Model& model)
{
    std::cerr << "Available joints:\n";
    for (size_t i = 0; i < model.names.size(); ++i) {
        std::cerr << "  [" << i << "] " << model.names[i] << "\n";
    }
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
        // Foot frames (must exist as FRAME names in URDF)
        const std::vector<std::string> foot_frames = {
            "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"
        };

        // Joint names in the SAME ORDER as your CSV:
        // LF(HAA,HFE,KFE), RF(...), LH(...), RH(...)
        const std::vector<std::string> joint_names = {
            "LF_HAA", "LF_HFE", "LF_KFE",
            "RF_HAA", "RF_HFE", "RF_KFE",
            "LH_HAA", "LH_HFE", "LH_KFE",
            "RH_HAA", "RH_HFE", "RH_KFE"
        };

        if (foot_frames.size() != 4 || joint_names.size() != 12) {
            throw std::runtime_error("Expected 4 foot frames and 12 joint names.");
        }

        // Load measurements (12 joints in CSV)
        auto meas = benchmark::loadMeasurements(sensor_csv, 12);
        if (meas.empty()) throw std::runtime_error("No measurements loaded from sensor CSV.");

        std::cout << "Loaded " << meas.size() << " measurements\n";
        for (size_t i = 0; i < std::min<size_t>(5, meas.size()); ++i) {
            std::cout << "meas[" << i << "].t = " << std::setprecision(17) << meas[i].t << "\n";
        }


        // Load Pinocchio model (fixed-base)
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        // Map foot frames -> ids
        std::vector<pinocchio::FrameIndex> foot_ids(4);
        for (int i = 0; i < 4; ++i) {
            foot_ids[i] = model.getFrameId(foot_frames[i]);
            if (foot_ids[i] == (pinocchio::FrameIndex)(-1)) {
                std::cerr << "ERROR: Foot frame not found: " << foot_frames[i] << "\n";
                print_available_frames(model);
                throw std::runtime_error("Fix foot frame names.");
            }
        }

        // Map joints -> idx_q and idx_v
        std::vector<int> idx_q(12, -1);
        std::vector<int> idx_v(12, -1);

        for (int i = 0; i < 12; ++i) {
            const auto jid = model.getJointId(joint_names[i]);
            if (jid == 0) {
                std::cerr << "ERROR: Joint not found: " << joint_names[i] << "\n";
                print_available_joints(model);
                throw std::runtime_error("Fix joint names.");
            }
            idx_q[i] = model.joints[jid].idx_q();
            idx_v[i] = model.joints[jid].idx_v();
        }

        // Per-leg v indices (3 joints per leg)
        int leg_v_cols[4][3] = {
            { idx_v[0],  idx_v[1],  idx_v[2]  }, // LF
            { idx_v[3],  idx_v[4],  idx_v[5]  }, // RF
            { idx_v[6],  idx_v[7],  idx_v[8]  }, // LH
            { idx_v[9],  idx_v[10], idx_v[11] }  // RH
        };

        // Open output file
        std::ofstream out(out_csv);
        if (!out.is_open()) throw std::runtime_error("Cannot open output CSV: " + out_csv);
        write_header(out);

        out.setf(std::ios::fixed);
        out << std::setprecision(9);

        // Allocate vectors
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
        Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, model.nv);

        std::cout << "Computing kinematics for " << meas.size() << " samples...\n";

        // Main loop
        for (const auto& s : meas)
        {
            // Fill q,v only for these 12 joints
            for (int i = 0; i < 12; ++i) {
                q[idx_q[i]] = s.q[i];
                v[idx_v[i]] = s.dq[i];
            }

            // Kinematics
            pinocchio::forwardKinematics(model, data, q, v);
            pinocchio::updateFramePlacements(model, data);

            // OLD
            // Foot positions (root frame)
            // Eigen::Vector3d p_foot[4];
            // for (int leg = 0; leg < 4; ++leg) {
            //     p_foot[leg] = data.oMf[foot_ids[leg]].translation();
            // }

            // Jacobians and foot velocities
            // Eigen::Matrix3d Jleg[4];
            // Eigen::Vector3d vfoot[4];

            // for (int leg = 0; leg < 4; ++leg) {
            //     pinocchio::computeFrameJacobian(
            //         model, data, q, foot_ids[leg],
            //         pinocchio::ReferenceFrame::WORLD, J6);

            //     const Eigen::MatrixXd Jv_all = J6.topRows<3>(); // 3 x nv

            //     // slice 3 cols -> 3x3
            //     for (int r = 0; r < 3; ++r) {
            //         for (int c = 0; c < 3; ++c) {
            //             Jleg[leg](r,c) = Jv_all(r, leg_v_cols[leg][c]);
            //         }
            //     }

            //     Eigen::Vector3d dq_leg;
            //     dq_leg << s.dq[leg*3 + 0], s.dq[leg*3 + 1], s.dq[leg*3 + 2];
            //     vfoot[leg] = Jleg[leg] * dq_leg;
            // }
            // OLD

            // NEW: compute foot velocity as J*v (not just leg joints, but all)
            // Jacobians and foot velocities
            // Foot positions (world == base, but expressed in WORLD-aligned coords)
            Eigen::Vector3d p_foot[4];
            Eigen::Vector3d vfoot[4];
            Eigen::Matrix3d Jleg[4];

            for (int leg = 0; leg < 4; ++leg) {

                const auto fid = foot_ids[leg];

                // Position of the foot frame (translation)
                // NOTE: in fixed-base model this is expressed in the model "world" frame
                p_foot[leg] = data.oMf[fid].translation();

                // Frame velocity expressed in LOCAL_WORLD_ALIGNED:
                // - origin: at the frame
                // - axes: aligned with WORLD
                pinocchio::Motion v6 = pinocchio::getFrameVelocity(
                    model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED);

                vfoot[leg] = v6.linear();

                // --- Jacobian in the SAME reference frame (consistent with v6) ---
                J6.setZero();
                pinocchio::computeFrameJacobian(
                    model, data, q, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6);

                const Eigen::MatrixXd Jv_all = J6.topRows<3>(); // 3 x nv (linear part)

                // slice the 3 columns corresponding to that leg joints (3x3)
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        Jleg[leg](r,c) = Jv_all(r, leg_v_cols[leg][c]);
                    }
                }
            }
            // NEW


            // Write row
            out << s.t;

            // positions (12)
            for (int leg = 0; leg < 4; ++leg) {
                out << "," << p_foot[leg].x() << "," << p_foot[leg].y() << "," << p_foot[leg].z();
            }

            // velocities (12)
            for (int leg = 0; leg < 4; ++leg) {
                out << "," << vfoot[leg].x() << "," << vfoot[leg].y() << "," << vfoot[leg].z();
            }

            // Jacobians (36), row-major per leg
            for (int leg = 0; leg < 4; ++leg) {
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        out << "," << Jleg[leg](r,c);
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
