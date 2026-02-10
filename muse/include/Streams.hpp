#pragma once
#include "Csv.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <string>

#include <iostream>

namespace muse_offline {

struct SensorSample {
  double t_abs{};
  Eigen::Vector3d omega_imu{0,0,0};
  bool stance_lf{false}, stance_rf{false}, stance_lh{false}, stance_rh{false};
};

class SensorStream {
public:
  explicit SensorStream(const std::string& path) : r_(path) {
    const auto& idx = r_.idx();
    has_t_ = idx.count("t") > 0;
    has_ts_ = idx.count("timestamp") > 0;
    if (!has_t_ && !has_ts_) throw std::runtime_error("Sensor CSV needs 't' or 'timestamp'");
    for (auto c : {"imu_wx","imu_wy","imu_wz","contact_LF","contact_RF","contact_LH","contact_RH"}) {
      if (idx.count(c) == 0) throw std::runtime_error(std::string("Sensor CSV missing: ") + c);
    }
  }

  bool next(SensorSample& s) {
    if (!r_.next()) return false;
    const auto& row = r_.row();
    const auto& idx = r_.idx();
    s.t_abs = has_t_ ? getD(row, idx, "t") : getD(row, idx, "timestamp");
    s.omega_imu = Eigen::Vector3d(getD(row, idx, "imu_wx"),
                                  getD(row, idx, "imu_wy"),
                                  getD(row, idx, "imu_wz"));
    s.stance_lf = (getD(row, idx, "contact_LF") > 0.5);
    s.stance_rf = (getD(row, idx, "contact_RF") > 0.5);
    s.stance_lh = (getD(row, idx, "contact_LH") > 0.5);
    s.stance_rh = (getD(row, idx, "contact_RH") > 0.5);
    // std::cout << "stance_lf: " << s.stance_lf << ", stance_rf: " << s.stance_rf
    //           << ", stance_lh: " << s.stance_lh << ", stance_rh: " << s.stance_rh << std::endl;
    return true;
  }

private:
  CsvReader r_;
  bool has_t_{false}, has_ts_{false};
};

struct FeetKin {
  double t_abs{};
  Eigen::Vector3d p_lf{0,0,0}, p_rf{0,0,0}, p_lh{0,0,0}, p_rh{0,0,0};
  Eigen::Vector3d v_lf{0,0,0}, v_rf{0,0,0}, v_lh{0,0,0}, v_rh{0,0,0};
};

class FeetStream {
public:
  explicit FeetStream(const std::string& path) : r_(path) {
    const auto& idx = r_.idx();
    if (idx.count("t") == 0) throw std::runtime_error("Feet CSV missing 't'");
    for (auto c : {"p_LF_x","p_LF_y","p_LF_z","p_RF_x","p_RF_y","p_RF_z","p_LH_x","p_LH_y","p_LH_z","p_RH_x","p_RH_y","p_RH_z",
                   "v_LF_x","v_LF_y","v_LF_z","v_RF_x","v_RF_y","v_RF_z","v_LH_x","v_LH_y","v_LH_z","v_RH_x","v_RH_y","v_RH_z"}) {
      if (idx.count(c) == 0) throw std::runtime_error(std::string("Feet CSV missing: ") + c);
    }
    // prime first row
    if (!r_.next()) throw std::runtime_error("Feet CSV has no data");
    current_ = parseCurrent();
  }

  // Advance until current_.t_abs >= t_abs (simple sync)
  // Returns current feet sample (closest "at or after").
  const FeetKin& at(double t_abs) {
    while (true) {
      if (current_.t_abs >= t_abs) return current_;
      if (!r_.next()) return current_; // end of file: keep last
      current_ = parseCurrent();
    }
  }

private:
  FeetKin parseCurrent() const {
    FeetKin f;
    const auto& row = r_.row();
    const auto& idx = r_.idx();
    f.t_abs = getD(row, idx, "t");
    f.p_lf = {getD(row, idx, "p_LF_x"), getD(row, idx, "p_LF_y"), getD(row, idx, "p_LF_z")};
    f.p_rf = {getD(row, idx, "p_RF_x"), getD(row, idx, "p_RF_y"), getD(row, idx, "p_RF_z")};
    f.p_lh = {getD(row, idx, "p_LH_x"), getD(row, idx, "p_LH_y"), getD(row, idx, "p_LH_z")};
    f.p_rh = {getD(row, idx, "p_RH_x"), getD(row, idx, "p_RH_y"), getD(row, idx, "p_RH_z")};

    f.v_lf = {getD(row, idx, "v_LF_x"), getD(row, idx, "v_LF_y"), getD(row, idx, "v_LF_z")};
    f.v_rf = {getD(row, idx, "v_RF_x"), getD(row, idx, "v_RF_y"), getD(row, idx, "v_RF_z")};
    f.v_lh = {getD(row, idx, "v_LH_x"), getD(row, idx, "v_LH_y"), getD(row, idx, "v_LH_z")};
    f.v_rh = {getD(row, idx, "v_RH_x"), getD(row, idx, "v_RH_y"), getD(row, idx, "v_RH_z")};
    return f;
  }

  CsvReader r_;
  FeetKin current_;
};

struct AttRow {
  double t_abs{};
  Eigen::Quaterniond q{1,0,0,0}; // w,x,y,z
};

class AttitudeStream {
public:
    explicit AttitudeStream(const std::string& path) : r_(path) {
        const auto& idx = r_.idx();
        for (auto c : {"t_abs","qw","qx","qy","qz"}) {
        if (idx.count(c) == 0) throw std::runtime_error(std::string("Attitude CSV missing: ") + c);
        }
    }

    bool next(double& t_abs, Eigen::Quaterniond& q_wxyz) {
        if (!r_.next()) return false;
        const auto& row = r_.row();
        const auto& idx = r_.idx();

        t_abs = getD(row, idx, "t_abs");
        const double qw = getD(row, idx, "qw");
        const double qx = getD(row, idx, "qx");
        const double qy = getD(row, idx, "qy");
        const double qz = getD(row, idx, "qz");

        q_wxyz = Eigen::Quaterniond(qw, qx, qy, qz);
        q_wxyz.normalize();
        return true;
    }


// class AttitudeTable {
// public:
//     explicit AttitudeTable(const std::string& path) : r_(path) {
//         const auto& idx = r_.idx();
//         for (auto c : {"t_abs","qw","qx","qy","qz"}) {
//         if (idx.count(c) == 0) throw std::runtime_error(std::string("Attitude CSV missing: ") + c);
//         }
//         loadAll();
//         if (rows_.size() < 2) throw std::runtime_error("Not enough attitude samples");
//     }

//     Eigen::Quaterniond orientation(double t_abs) {
//         while (k_ + 1 < rows_.size() && rows_[k_ + 1].t_abs < t_abs) k_++;
//         if (k_ + 1 >= rows_.size()) return rows_.back().q;

//         const auto& a = rows_[k_];
//         const auto& b = rows_[k_ + 1];
//         const double dt = b.t_abs - a.t_abs;
//         if (dt <= 1e-12) return a.q;

//         double u = (t_abs - a.t_abs) / dt;
//         u = std::clamp(u, 0.0, 1.0);

//         Eigen::Vector4d qa(a.q.w(), a.q.x(), a.q.y(), a.q.z());
//         Eigen::Vector4d qb(b.q.w(), b.q.x(), b.q.y(), b.q.z());
//         if (qa.dot(qb) < 0.0) qb = -qb; // avoid sign flip

//         Eigen::Vector4d q = (1.0 - u) * qa + u * qb;
//         Eigen::Quaterniond out(qa(0), qa(1), qa(2), qa(3));
//         out.normalize();
//         return out;
//   }

private:
  void loadAll() {
    while (r_.next()) {
        const auto& row = r_.row();
        const auto& idx = r_.idx();
        AttRow a;
        a.t_abs = getD(row, idx, "t_abs");
        a.q = Eigen::Quaterniond(getD(row, idx, "qw"),
                                getD(row, idx, "qx"),
                                getD(row, idx, "qy"),
                                getD(row, idx, "qz"));
        a.q.normalize();
        rows_.push_back(a);
    }
  }

  CsvReader r_;
  std::vector<AttRow> rows_;
  size_t k_{0};
};

} // namespace muse_offline
