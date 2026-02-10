#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

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
    // fallback: prova a parsare numero
    return (std::stod(s) != 0.0) ? 1 : 0;
}

// -----------------------------
// Feet kinematics stream (semplice: lockstep per riga, oppure cerca per timestamp)
// Qui facciamo lockstep: una riga feet per ogni riga sensor.
// Feet CSV header: t, p_*, v_*, J_* ...
// -----------------------------
struct FeetRow {
    double t_abs = 0.0;
    // foot pos: LF, RF, LH, RH (3 ciascuno) => 12
    std::array<double, 12> p{};
    // jacobians: 4*(3x3)=36, row-major per leg => 36
    std::array<double, 36> J{};
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

        // positions order in your feet_kinematics.csv header:
        // p_LF_x,p_LF_y,p_LF_z, p_RF_x..., p_LH..., p_RH...
        const char* legs[4] = {"LF","RF","LH","RH"};
        int k = 0;
        for (int leg = 0; leg < 4; ++leg) {
            fr.p[k++] = getColAsDouble(row, idx_, std::string("p_") + legs[leg] + "_x");
            fr.p[k++] = getColAsDouble(row, idx_, std::string("p_") + legs[leg] + "_y");
            fr.p[k++] = getColAsDouble(row, idx_, std::string("p_") + legs[leg] + "_z");
        }

        // Jacobians: J_LF_00..J_LF_22 then RF then LH then RH (row-major 3x3)
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
// IS (Invariant Smoother) ordered input (82 dims) + timestamp
// -----------------------------
struct ISInput {
    double t_abs = 0.0;
    std::array<double, 82> z{}; // in the required order
};

// Build the 82-vector in the exact order shown in your screenshot.
static ISInput buildISInputFromRows(const std::vector<std::string>& row,
                                    const std::unordered_map<std::string,int>& idx,
                                    const FeetRow& feet)
{
    ISInput out;

    // time
    // (your sensor_data.csv uses "t")
    out.t_abs = getColAsDouble(row, idx, "t");

    int k = 0;

    // 1) IMU angular velocity (3)
    out.z[k++] = getColAsDouble(row, idx, "imu_wx");
    out.z[k++] = getColAsDouble(row, idx, "imu_wy");
    out.z[k++] = getColAsDouble(row, idx, "imu_wz");

    // 2) IMU acceleration (3)
    out.z[k++] = getColAsDouble(row, idx, "imu_ax");
    out.z[k++] = getColAsDouble(row, idx, "imu_ay");
    out.z[k++] = getColAsDouble(row, idx, "imu_az");

    // 3) Joint positions (12) in the leg order expected by your IS.
    // Your screenshot says legs order: RR, RL, FL, FR (rear right, rear left, front left, front right)
    // BUT your sensor_data.csv columns are: LF, RF, LH, RH (front-left, front-right, hind-left, hind-right).
    //
    // Mapping to (RR, RL, FL, FR) corresponds to (RH, LH, LF, RF).
    const char* JO_POS[12] = {
        "joint_pos_RH_HAA","joint_pos_RH_HFE","joint_pos_RH_KFE", // RR
        "joint_pos_LH_HAA","joint_pos_LH_HFE","joint_pos_LH_KFE", // RL
        "joint_pos_LF_HAA","joint_pos_LF_HFE","joint_pos_LF_KFE", // FL
        "joint_pos_RF_HAA","joint_pos_RF_HFE","joint_pos_RF_KFE"  // FR
    };
    for (int i = 0; i < 12; ++i) out.z[k++] = getColAsDouble(row, idx, JO_POS[i]);

    // 4) Joint velocities (12), same order as positions
    const char* JO_VEL[12] = {
        "joint_vel_RH_HAA","joint_vel_RH_HFE","joint_vel_RH_KFE", // RR
        "joint_vel_LH_HAA","joint_vel_LH_HFE","joint_vel_LH_KFE", // RL
        "joint_vel_LF_HAA","joint_vel_LF_HFE","joint_vel_LF_KFE", // FL
        "joint_vel_RF_HAA","joint_vel_RF_HFE","joint_vel_RF_KFE"  // FR
    };
    for (int i = 0; i < 12; ++i) out.z[k++] = getColAsDouble(row, idx, JO_VEL[i]);

    // 5) Contact state (4) in RR,RL,FL,FR => (RH, LH, LF, RF)
    out.z[k++] = (double)getColAsBool01(row, idx, "contact_RH");
    out.z[k++] = (double)getColAsBool01(row, idx, "contact_LH");
    out.z[k++] = (double)getColAsBool01(row, idx, "contact_LF");
    out.z[k++] = (double)getColAsBool01(row, idx, "contact_RF");

    // 6) Foot positions (12): again RR,RL,FL,FR
    // feet_kinematics.csv is stored as LF,RF,LH,RH. Remap -> RH,LH,LF,RF.
    auto pickP = [&](int legLF_RF_LH_RH, int axis) -> double {
        // leg index: 0=LF,1=RF,2=LH,3=RH; axis 0=x,1=y,2=z
        return feet.p[legLF_RF_LH_RH*3 + axis];
    };

    // RR (RH=3)
    out.z[k++] = pickP(3,0); out.z[k++] = pickP(3,1); out.z[k++] = pickP(3,2);
    // RL (LH=2)
    out.z[k++] = pickP(2,0); out.z[k++] = pickP(2,1); out.z[k++] = pickP(2,2);
    // FL (LF=0)
    out.z[k++] = pickP(0,0); out.z[k++] = pickP(0,1); out.z[k++] = pickP(0,2);
    // FR (RF=1)
    out.z[k++] = pickP(1,0); out.z[k++] = pickP(1,1); out.z[k++] = pickP(1,2);

    // 7) Foot Jacobians (36): feet_kinematics.csv stores J in legs LF,RF,LH,RH.
    // Remap order to RR,RL,FL,FR => RH,LH,LF,RF.
    auto copyJleg = [&](int legLF_RF_LH_RH) {
        // each leg has 9 values row-major
        const int base = legLF_RF_LH_RH * 9;
        for (int i = 0; i < 9; ++i) out.z[k++] = feet.J[base + i];
    };

    copyJleg(3); // RH -> RR
    copyJleg(2); // LH -> RL
    copyJleg(0); // LF -> FL
    copyJleg(1); // RF -> FR

    if (k != 82) {
        throw std::runtime_error("Internal error: IS z has size " + std::to_string(k) + " (expected 82)");
    }

    return out;
}

// -----------------------------
// MAIN
// -----------------------------
static const std::string DEFAULT_DATASET_ROOT = "../../data/anymalD_grandtour";

int main(int argc, char** argv)
{
    std::string dataset_root = DEFAULT_DATASET_ROOT;
    if (argc > 1) dataset_root = argv[1];

    const std::string sensor_csv = dataset_root + "/sensor_data.csv";
    const std::string feet_csv   = dataset_root + "/feet_kinematics.csv";

    std::cout << "IS input reorder (in-memory)\n"
              << "  Sensor: " << sensor_csv << "\n"
              << "  Feet  : " << feet_csv << "\n";

    std::ifstream in(sensor_csv);
    if (!in.is_open()) {
        std::cerr << "Error: cannot open " << sensor_csv << "\n";
        return 1;
    }

    FeetStream feet(feet_csv);
    FeetRow fr;

    // Read sensor header
    std::string line;
    if (!std::getline(in, line)) {
        std::cerr << "Error: empty sensor_data.csv\n";
        return 1;
    }
    auto header = splitCSVLine(line);
    auto idx = headerIndex(header);

    // sanity check required columns (sensor)
    const std::vector<std::string> req = {
        "t","imu_wx","imu_wy","imu_wz","imu_ax","imu_ay","imu_az",
        "joint_pos_LF_HAA","joint_pos_LF_HFE","joint_pos_LF_KFE",
        "joint_pos_RF_HAA","joint_pos_RF_HFE","joint_pos_RF_KFE",
        "joint_pos_LH_HAA","joint_pos_LH_HFE","joint_pos_LH_KFE",
        "joint_pos_RH_HAA","joint_pos_RH_HFE","joint_pos_RH_KFE",
        "joint_vel_LF_HAA","joint_vel_LF_HFE","joint_vel_LF_KFE",
        "joint_vel_RF_HAA","joint_vel_RF_HFE","joint_vel_RF_KFE",
        "joint_vel_LH_HAA","joint_vel_LH_HFE","joint_vel_LH_KFE",
        "joint_vel_RH_HAA","joint_vel_RH_HFE","joint_vel_RH_KFE",
        "contact_LF","contact_RF","contact_LH","contact_RH"
    };
    for (const auto& c : req) {
        if (idx.count(c) == 0) {
            std::cerr << "Error: missing required sensor column: " << c << "\n";
            return 1;
        }
    }

    // Example loop: build ISInput and (for now) just print first few
    std::size_t n = 0;
    while (std::getline(in, line)) {
        if (line.empty()) continue;

        if (!feet.next(fr)) {
            std::cerr << "Feet stream ended early.\n";
            break;
        }

        auto row = splitCSVLine(line);

        try {
            ISInput u = buildISInputFromRows(row, idx, fr);

            if (n < 3) {
                std::cout << "Sample " << n << " t_abs=" << u.t_abs << " z[0..5]="
                          << u.z[0] << " " << u.z[1] << " " << u.z[2] << " "
                          << u.z[3] << " " << u.z[4] << " " << u.z[5] << "\n";
            }

            // TODO: qui chiami il tuo IS:
            // is.step(u.t_abs, u.z);
            n++;
        } catch (const std::exception& e) {
            std::cerr << "Skipping row " << n << " due to: " << e.what() << "\n";
        }
    }

    std::cout << "Done. Processed " << n << " samples.\n";
    return 0;
}
