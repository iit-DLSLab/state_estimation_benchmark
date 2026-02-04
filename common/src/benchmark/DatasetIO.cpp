#include "benchmark/DatasetIO.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace benchmark {

static std::vector<double> parseCsvLineToDoubles(const std::string& line)
{
    std::vector<double> values;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) continue;
        values.push_back(std::stod(item));
    }
    return values;
}

MeasurementSequence loadMeasurements(const std::string& csv_path, int num_joints)
{
    const int expected_cols = 1 + 3 + 3 + num_joints + num_joints + 4;

    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open sensor CSV: " + csv_path);
    }

    MeasurementSequence seq;
    std::string line;
    // optionally skip header if first line has non-numeric tokens
    if (std::getline(ifs, line)) {
        bool header = false;
        for (char c : line) { if (std::isalpha((unsigned char)c)) { header = true; break; } }
        if (!header) {
            // first line is data: parse it
            auto vals = parseCsvLineToDoubles(line);
            if ((int)vals.size() != expected_cols) {
                throw std::runtime_error("sensor_data.csv: expected " + std::to_string(expected_cols) + " cols, got " + std::to_string(vals.size()));
            }
            MeasurementSample m;
            int idx = 0;
            m.t = vals[idx++];
            m.omega = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]); idx += 3;
            m.acc   = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]); idx += 3;
            m.q.resize(num_joints);
            for (int i=0;i<num_joints;++i) m.q[i]=vals[idx++];
            m.dq.resize(num_joints);
            for (int i=0;i<num_joints;++i) m.dq[i]=vals[idx++];
            for (int i=0;i<4;++i) m.contact[i] = vals[idx++] > 0.5 ? 1 : 0;
            seq.push_back(std::move(m));
        }
        // else header -> skip parsing header and continue with rest
    }
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        auto vals = parseCsvLineToDoubles(line);
        if ((int)vals.size() != expected_cols) {
            throw std::runtime_error("sensor_data.csv: expected " + std::to_string(expected_cols) + " cols, got " + std::to_string(vals.size()));
        }
        MeasurementSample m;
        int idx = 0;
        m.t = vals[idx++];
        m.omega = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]); idx += 3;
        m.acc   = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]); idx += 3;
        m.q.resize(num_joints);
        for (int i=0;i<num_joints;++i) m.q[i]=vals[idx++];
        m.dq.resize(num_joints);
        for (int i=0;i<num_joints;++i) m.dq[i]=vals[idx++];
        for (int i=0;i<4;++i) m.contact[i] = vals[idx++] > 0.5 ? 1 : 0;
        seq.push_back(std::move(m));
    }
    return seq;
}

GroundTruthSequence loadGroundTruth(const std::string& csv_path)
{
    const int expected_cols = 1 + 3 + 4 + 3; // t, pos(3), quat(4), vel(3)

    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open GT CSV: " + csv_path);
    }

    GroundTruthSequence seq;
    std::string line;
    // detect header like above
    if (std::getline(ifs, line)) {
        bool header = false;
        for (char c : line) { if (std::isalpha((unsigned char)c)) { header = true; break; } }
        if (!header) {
            auto vals = parseCsvLineToDoubles(line);
            if ((int)vals.size() != expected_cols) {
                throw std::runtime_error("groundtruth.csv: expected " + std::to_string(expected_cols) + " cols, got " + std::to_string(vals.size()));
            }
            GroundTruthSample g;
            int idx = 0;
            g.t = vals[idx++];
            g.p = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]); idx += 3;
            double qx = vals[idx++], qy = vals[idx++], qz = vals[idx++], qw = vals[idx++];
            g.q = Eigen::Quaterniond(qw,qx,qy,qz); g.q.normalize();
            g.v = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]);
            seq.push_back(std::move(g));
        }
    }
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        auto vals = parseCsvLineToDoubles(line);
        if ((int)vals.size() != expected_cols) {
            throw std::runtime_error("groundtruth.csv: expected " + std::to_string(expected_cols) + " cols, got " + std::to_string(vals.size()));
        }
        GroundTruthSample g;
        int idx = 0;
        g.t = vals[idx++];
        g.p = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]); idx += 3;
        double qx = vals[idx++], qy = vals[idx++], qz = vals[idx++], qw = vals[idx++];
        g.q = Eigen::Quaterniond(qw,qx,qy,qz); g.q.normalize();
        g.v = Eigen::Vector3d(vals[idx], vals[idx+1], vals[idx+2]);
        seq.push_back(std::move(g));
    }
    return seq;
}

} // namespace benchmark
