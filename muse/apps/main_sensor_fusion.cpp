#include "Models/SensorFusion.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "rotations.h"

static const std::string DEFAULT_DATASET_ROOT = "../../data/anymalD_grandtour";

struct CsvRow {
  std::vector<std::string> v;
};

static std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(item);
  return out;
}

class CsvReader {
public:
    explicit CsvReader(const std::string& path) : in_(path) {
        if (!in_) throw std::runtime_error("Cannot open: " + path);
        std::string header;
        if (!std::getline(in_, header)) throw std::runtime_error("Empty file: " + path);
        auto cols = split_csv_line(header);
        for (size_t i=0;i<cols.size();++i) col_[cols[i]] = i;
    }

    bool next(CsvRow& r) {
        std::string line;
        if (!std::getline(in_, line)) return false;
        r.v = split_csv_line(line);
        return true;
  }

    bool has(const std::string& name) const { return col_.count(name) > 0; }
    size_t idx(const std::string& name) const {
        auto it = col_.find(name);
        if (it == col_.end()) throw std::runtime_error("Missing column: " + name);
        return it->second;
  }

private:
    std::ifstream in_;
    std::unordered_map<std::string,size_t> col_;
};

static double stod_safe(const std::string& s) {
    // gestisce eventuali spazi
    size_t p=0;
    return std::stod(s, &p);
}

int main(int argc, char** argv) {
    std::string dataset_root = DEFAULT_DATASET_ROOT;
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string att_csv    = dataset_root + "/muse/attitude_estimate_muse.csv";
    const std::string leg_csv    = dataset_root + "/muse/leg_odometry.csv";
    const std::string out_csv    = dataset_root + "/muse/fused_state.csv";

    std::cout << "Sensor: " << sensor_csv << "\n"
                << "Att   : " << att_csv << "\n"
                << "Leg   : " << leg_csv << "\n"
                << "Out   : " << out_csv << "\n";

    CsvReader sensor(sensor_csv);
    CsvReader att(att_csv);
    CsvReader leg(leg_csv);

    // base_R_imu per ANYmalD (quello che stavi usando)
    Eigen::Matrix3d base_R_imu;
    // base_R_imu << -1, 0, 0,
    //                0, 1, 0,
    //                0, 0,-1;
    Eigen::Quaterniond b_quat_imu;
    b_quat_imu.w() = 3.749399456654644e-33;
    b_quat_imu.x() = 6.123233995736766e-17;
    b_quat_imu.y() = 1.0;
    b_quat_imu.z() = 6.123233995736766e-17;
    base_R_imu = iit::commons::quatToRotMat(b_quat_imu.normalized()).transpose();
    std::cout << "base_R_imu:\n" << base_R_imu << "\n";

    // Inizializzazione KF come plugin
    // x: -0.25565
    // y: 0.00255
    // z: 0.07672
    const double t0 = 0.0;
    Eigen::Matrix<double,6,1> x0; x0.setZero();
    x0(0) = -0.25565; // posizione iniziale (x)
    x0(1) = 0.00255;  // posizione iniziale (y)
    x0(2) = 0.07672;  // posizione iniziale (z)
    x0(3) = 0.0;      // velocità iniziale (vx)
    x0(4) = 0.0;      // velocità iniziale (vy)
    x0(5) = 0.0;      // velocità iniziale (vz)

    Eigen::Matrix<double,6,6> P; P.setIdentity(); P *= 1e-10;
    Eigen::Matrix<double,6,6> Q; Q.setIdentity(); Q *= 1e-10;
    Eigen::Matrix<double,3,3> R; R.setIdentity(); R *= 5e-13;

    // Se vuoi replicare i tuoi parametri ROS, mettili qui (hard-coded o letti da yaml).
    state_estimator::KFSensorFusion kf(t0, x0, P, Q, R, false, false);

    try {
        std::ofstream out(out_csv);
        out.setf(std::ios::fixed);
        out << std::setprecision(9);
        out << "t_rel,t_abs,"
            "px,py,pz,"
            "vx,vy,vz,"
            "qw,qx,qy,qz\n";

        CsvRow rs, ra, rl;

        bool first = true;
        double t_abs0 = 0.0;

        while (true) {
        if (!sensor.next(rs)) break;
        if (!att.next(ra)) break;
        if (!leg.next(rl)) break;

        // time: usiamo t_abs (timestamp) dai csv
        const double t_abs = stod_safe(rs.v[sensor.idx("t")]);
        if (first) { t_abs0 = t_abs; first = false; }
        const double t_rel = t_abs - t_abs0;

        // accel IMU dal sensor_data.csv
        // NB: nomi come nel tuo file: imu_ax, imu_ay, imu_az
        Eigen::Vector3d acc;
        acc.x() = stod_safe(rs.v[sensor.idx("imu_ax")]);
        acc.y() = stod_safe(rs.v[sensor.idx("imu_ay")]);
        acc.z() = stod_safe(rs.v[sensor.idx("imu_az")]);

        // quaternion MUSE attitude_estimate_muse.csv (qw,qx,qy,qz)
        Eigen::Quaterniond q_wb;
        q_wb.w() = stod_safe(ra.v[att.idx("qw")]);
        q_wb.x() = stod_safe(ra.v[att.idx("qx")]);
        q_wb.y() = stod_safe(ra.v[att.idx("qy")]);
        q_wb.z() = stod_safe(ra.v[att.idx("qz")]);
        q_wb.normalize();

        // plugin faceva: w_R_b = quatToRotMat(quat_est).transpose()
        // qui: R(q_wb) in Eigen è w_R_b se q è "world<-body". Se nel tuo MUSE è l’opposto, inverti.
        // Dato che in attitude hai già sistemato le convenzioni, manteniamo:
        const Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(q_wb).transpose();

        // accel input u = w_R_b * (base_R_imu*acc) + gravity
        const Eigen::Vector3d f_b = base_R_imu * acc;
        const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
        const Eigen::Vector3d u_w = w_R_b * f_b + gravity;

        kf.predict(t_rel, u_w);

        // misura velocità
        Eigen::Vector3d z_v_w;

        const bool leg_has_world =
            leg.has("v_base_w_x") && leg.has("v_base_w_y") && leg.has("v_base_w_z");

        if (leg_has_world) {
            z_v_w.x() = stod_safe(rl.v[leg.idx("v_base_w_x")]);
            z_v_w.y() = stod_safe(rl.v[leg.idx("v_base_w_y")]);
            z_v_w.z() = stod_safe(rl.v[leg.idx("v_base_w_z")]);
        } else {
            // fallback: base-frame velocity in CSV -> rotate to world like plugin
            Eigen::Vector3d v_b;
            v_b.x() = stod_safe(rl.v[leg.idx("v_base_b_x")]);
            v_b.y() = stod_safe(rl.v[leg.idx("v_base_b_y")]);
            v_b.z() = stod_safe(rl.v[leg.idx("v_base_b_z")]);
            z_v_w = w_R_b * v_b;
        }

        kf.update(t_rel, z_v_w);

        const auto& x = kf.getX();
        out << t_rel << "," << t_abs << ","
            << x(0) << "," << x(1) << "," << x(2) << ","
            << x(3) << "," << x(4) << "," << x(5) << ","
            << q_wb.w() << "," << q_wb.x() << "," << q_wb.y() << "," << q_wb.z() << "\n";
    }

    std::cout << "Done. Wrote: " << out_csv << "\n";
    return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
