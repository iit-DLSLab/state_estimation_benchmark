#include "Streams.hpp"
#include "LegOdometry.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    std::string dataset_root = "../../data/anymalD_grandtour";
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string feet_csv   = dataset_root + "/feet_kinematics.csv";
    const std::string att_csv    = dataset_root + "/muse/attitude_estimate_muse.csv";
    const std::string out_csv    = dataset_root + "/muse/leg_odometry.csv";

    std::cout << "Sensor: " << sensor_csv << "\nFeet: " << feet_csv
              << "\nAtt: " << att_csv << "\nOut: " << out_csv << "\n";

    muse_offline::SensorStream sensor(sensor_csv);
    muse_offline::FeetStream   feet(feet_csv);
    muse_offline::AttitudeStream att(att_csv);

    muse_offline::LegOdometry lo(muse_offline::b_R_imu_anymalD());

    std::ofstream out(out_csv);
    out.setf(std::ios::fixed);
    out << std::setprecision(9);

    out << "t_rel,t_abs,v_base_b_x,v_base_b_y,v_base_b_z,v_base_w_x,v_base_w_y,v_base_w_z\n";

    muse_offline::SensorSample s;
    double t_att_abs = 0.0;
    Eigen::Quaterniond q_est(1,0,0,0);

    bool first = true;
    double t0 = 0.0;

    while (sensor.next(s)) {
        if (!att.next(t_att_abs, q_est)) break;   // lockstep: una riga attitude per ogni riga sensor

        if (first) { t0 = s.t_abs; first = false; }
        const double t_rel = s.t_abs - t0;

        const auto& fk = feet.at(s.t_abs);

        const auto res = lo.step(s, fk, q_est);

        static int k = 0;
        if (k < 5) {
        std::cout << std::setprecision(12)
                    << "q_est(wxyz)= " << q_est.w() << " "
                    << q_est.x() << " " << q_est.y() << " " << q_est.z() << "\n";
        std::cout << "v_base_b = " << res.v_base_b.transpose() << "\n";
        std::cout << "v_base_w = " << res.v_base_w.transpose() << "\n";
        std::cout << "diff     = " << (res.v_base_w - res.v_base_b).transpose() << "\n\n";
        }
        k++;

        // rotation: 
        // x: 0.9999999991989279
        // y: 0.0
        // z: 0.0
        // w: 4.0026794885936924e-05

        Eigen::Quaterniond base_quat_box;
        base_quat_box.w() = 0.0;
        base_quat_box.x() = 1.0;
        base_quat_box.y() = 0.0;
        base_quat_box.z() = 0.0;
        base_quat_box.normalize();
        Eigen::Matrix3d base_R_box = iit::commons::quatToRotMat(base_quat_box).transpose();

        // std::cout << "base_R_box:\n" << base_R_box << "\n";

        Eigen::Vector3d v_base_b_box = base_R_box * res.v_base_b;
        Eigen::Vector3d v_base_w_box = base_R_box * res.v_base_w;

        // out << t_rel << "," << s.t_abs << ","
        //     << v_base_b_box.x() << "," << v_base_b_box.y() << "," << v_base_b_box.z() << ","
        //     << v_base_w_box.x() << "," << v_base_w_box.y() << "," << v_base_w_box.z() << "\n";

        out << t_rel << "," << s.t_abs << ","
            << res.v_base_b.x() << "," << -res.v_base_b.y() << "," << res.v_base_b.z() << ","
            << res.v_base_w.x() << "," << -res.v_base_w.y() << "," << res.v_base_w.z() << "\n";     // this is the correct one



    }

    std::cout << "Done.\n";
    return 0;
}
