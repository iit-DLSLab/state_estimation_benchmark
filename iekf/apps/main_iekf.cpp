#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

static const std::string DEFAULT_DATASET_ROOT = "../../data/anymalD_grandtour";

int main(int argc, char** argv)
{
    std::string dataset_root = DEFAULT_DATASET_ROOT;
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string gt_csv     = dataset_root + "/groundtruth.csv";
    const std::string feet_csv   = dataset_root + "/feet_kinematics.csv";
    const std::string out_csv    = dataset_root + "/iekf/iekf_state.csv";

    std::cout << "IEKF (offline) - skeleton\n"
                << "  Sensor: " << sensor_csv << "\n"
                << "  GT    : " << gt_csv << "\n"
                << "  Feet  : " << feet_csv << "\n"
                << "  Out   : " << out_csv << "\n";

    std::ofstream out(out_csv);
    if (!out.is_open()) {
        std::cerr << "Error: cannot open " << out_csv << " for writing.\n"
                << "Create folder: " << dataset_root << "/iekf\n";
        return 1;
    }

    // out.setf(std::ios::fixed);
    // out << std::setprecision(9);

    // out << "t_abs,px,py,pz,vx,vy,vz,qw,qx,qy,qz\n";

    // // Stato dummy (per ora)
    // Eigen::Vector3d p = Eigen::Vector3d::Zero();
    // Eigen::Vector3d v = Eigen::Vector3d::Zero();
    // Eigen::Quaterniond q(1,0,0,0);

    // double t = 0.0; // Dummy timestamp
    // out << t;

    std::cout << "Done. Wrote " << 1 << " rows to " << out_csv << "\n";
  return 0;
}
