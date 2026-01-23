// Run MUSE’s four stages (Attitude - Contact - Leg Odometry - Sensor Fusion)
//
// Build (example):
//    ./build_release.sh
// Run:
//    ./build/release/bin/Estimator ./samples/FILE.csv [OPTIONAL: model.urdf]


#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <filesystem>

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp> 

// Repo headers
#include "state_estimator/Models/attitude_bias_XKF.hpp"   // AttitudeBiasXKF
#include "state_estimator/Models/sensor_fusion.hpp"       // KFSensorFusion
#include "iit/commons/geometry/rotations.h"               // quatToRotMat, quatToRPY, quatToOmega



// ===== Messages (mirror of state_estimator_msgs) =====
struct AttitudeMsg {
  double quaternion[4]{};    // [w,x,y,z]
  double roll_deg{}, pitch_deg{}, yaw_deg{};
  double angular_velocity[3]{}; // filtered ω (rad/s)
};

struct ContactDetectionMsg {
  bool stance_lf{}, stance_rf{}, stance_lh{}, stance_rh{};
};

struct LegOdometryMsg {
  double lin_vel_lf[3]{};double lin_vel_rf[3]{};
  double lin_vel_lh[3]{}; double lin_vel_rh[3]{};
  double base_velocity[3]{}; // world frame
  double base_z_viapc=0;
};

struct SensorFusionMsg {
  double position[3]{};          // x,y,z (world)
  double linear_velocity[3]{};   // vx,vy,vz (world)
  double orientation_quat[4]{};  // w,x,y,z (from attitude)
  double angular_velocity[3]{};  // from attitude
};

struct GroundTruth {
  Eigen::Vector3d pos{0,0,0};
  Eigen::Vector3d vel{0,0,0};
  Eigen::Vector3d rpy{0,0,0};          // roll, pitch, yaw (deg or rad as logged)
  Eigen::Vector4d quat{0,0,0,0};
  Eigen::Vector3d ang_vel{0,0,0};
};

// ===== Helpers =====
static inline double rad2deg(double r){ return r * 180.0/M_PI; }

static void print_attitude(const AttitudeMsg& a, const GroundTruth& gt){
  std::cout << std::fixed << std::setprecision(5); 
  std::cout << "Attitude\n"
            << "  rpy (deg):  " << std::setw(10) << a.roll_deg << " " << std::setw(10) << a.pitch_deg << " " << std::setw(10) << a.yaw_deg
            << "      quat [w x y z]: " << std::setw(8) << a.quaternion[0] << " " << std::setw(8) << a.quaternion[1] << " " << std::setw(8) << a.quaternion[2] << " " << std::setw(8) << a.quaternion[3]
            << "      omega_filt:     " << std::setw(8) << a.angular_velocity[0] << " " << std::setw(8) << a.angular_velocity[1] << " " << std::setw(8) << a.angular_velocity[2] << "\n"
            << "  ROOT_RPY_W: " << std::setw(10) << gt.rpy[0] << " " << std::setw(10) << gt.rpy[1] << " " << std::setw(10) << gt.rpy[2]   
            << "      ROOT_QUAT_W:    " << std::setw(8) << gt.quat[0] << " " << std::setw(8) << gt.quat[1] << " " << std::setw(8) << gt.quat[2] << " " << std::setw(8) << gt.quat[3]
            << "      ROOT_ANG_VEL_W: " << std::setw(8) << gt.ang_vel[0] << " " << std::setw(8) << gt.ang_vel[1] << " " << std::setw(8) << gt.ang_vel[2] << "\n";
}

static void print_attitude_mujoco(const AttitudeMsg& a){
  std::cout << std::fixed << std::setprecision(5); 
  std::cout << "Attitude\n"
            << "  rpy (deg):  " << std::setw(10) << a.roll_deg << " " << std::setw(10) << a.pitch_deg << " " << std::setw(10) << a.yaw_deg
            << "      quat [w x y z]: " << std::setw(8) << a.quaternion[0] << " " << std::setw(8) << a.quaternion[1] << " " << std::setw(8) << a.quaternion[2] << " " << std::setw(8) << a.quaternion[3]
            << "      omega_filt:     " << std::setw(8) << a.angular_velocity[0] << " " << std::setw(8) << a.angular_velocity[1] << " " << std::setw(8) << a.angular_velocity[2] << "\n";
}

static void print_contacts(const ContactDetectionMsg& c){
  std::cout << "Contacts  LF=" << c.stance_lf << " RF=" << c.stance_rf
            << " LH=" << c.stance_lh << " RH=" << c.stance_rh << "\n";
}

static void print_legodom(const LegOdometryMsg& lo){
  auto leg=[&](const char* n,const double v[3]){
    std::cout << "  " << n << " [vx vy vz]=" << v[0] << " " << v[1] << " " << v[2] << "\n";
  };
  std::cout << "Leg Odometry\n";
  leg("LF",lo.lin_vel_lf); leg("RF",lo.lin_vel_rf);
  leg("LH",lo.lin_vel_lh); leg("RH",lo.lin_vel_rh);
  std::cout << "  base_velocity=" << lo.base_velocity[0] << " "
            << lo.base_velocity[1] << " " << lo.base_velocity[2] << "\n";
  std::cout << "Base height via planned contacts:" << lo.base_z_viapc << "\n";
}


// ===== CONFIG (mirror of YAML defaults; tweak for RBQ) =====
struct Config {
  // Attitude (VERIFIED! Extracted from: ./muse/muse_ws/src/state_estimator/config/attitude_plugin.yaml)
  double ki = 0.02, kp = 10.0;
  Eigen::Matrix3d b_R_imu;   // body←imu
  Eigen::Vector3d north_vec{1.0/sqrt(3),1.0/sqrt(3),1.0/sqrt(3)};
  Eigen::Vector3d gravity_vec{0,0,9.81}; // magnitude +9.81 along +z in body for model’s convention

  // Matrices
  Eigen::Matrix<double,6,6> P0, Q, R, SF_P, SF_Q;
  Eigen::Matrix<double,3,3> SF_R; // Sensor fusion (VERIFIED! Extracted from: ./muse/muse_ws/src/state_estimator/config/sensor_fusion.yaml)

  // planned contacts vars
  double rho = 0.05; // gain to regulate intensity of drift correction
  double x0_IMU = 0.02557; // initial IMU x (0.02557)
  double z0_IMU = 0.55768; // initial IMU height (0.6-0.04232)

  // Contact detection (VERIFIED! Extracted from: ./muse/muse_ws/src/state_estimator/config/contact_plugin.yaml)
  double grf_threshold = -50.0; // N (changed by us)

  // Leg odometry
  std::string urdf_path = "./muse_ws/src/state_estimator/urdfs/RBQ.urdf"; // set to RBQ/anymal
  Eigen::Matrix3d base_R_imu = Eigen::Matrix3d::Identity();
  std::vector<std::string> foot_frames = {"FR_foot","FL_foot","HR_foot","HL_foot"}; //RBQ Names

  Eigen::Matrix<double,6,1> xhat6_init; 

  template<typename Mat>
  static void readMatrix(std::ifstream& f, Mat& M) {
    std::string dummy; // consume exactly two comment lines before each block
    std::getline(f, dummy);
    std::getline(f, dummy);
    for (int r = 0; r < M.rows(); ++r){
      for (int c = 0; c < M.cols(); ++c){
        f >> M(r,c);
      }
    }
    // flush remaining newline after numeric block
    f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  Config() {
    // get executable path, independent of CD
    char exePath[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", exePath, PATH_MAX);
    std::filesystem::path exeDir = std::filesystem::path(exePath).remove_filename();
    std::filesystem::path confPath = exeDir / "muse_conf.txt";

    std::ifstream f(confPath);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open config: " + confPath.string());
    }

    xhat6_init << x0_IMU, 0.0, z0_IMU, // x, y, z (world) of IMU
                0.0, 0.0, 0.0;        // vx, vy, vz
    readMatrix(f,b_R_imu);
    readMatrix(f,P0);
    readMatrix(f,Q);
    readMatrix(f,R);
    readMatrix(f,SF_P);
    readMatrix(f,SF_Q);
    readMatrix(f,SF_R);
  }
};

static void print_sensorfusion(const SensorFusionMsg& s, const Config& cfg, const GroundTruth& gt){
  std::cout << "Sensor Fusion\n"
            << "  pos=   " << std::setw(8) << s.position[0] << " " << std::setw(8) << s.position[1] << " " << std::setw(8) << s.position[2]
            << "                    root_pos_w:     " << std::setw(8) << gt.pos[0] << " " << std::setw(8) << gt.pos[1] << " " << std::setw(8) << gt.pos[2] << "\n"
            << "  vel=   " << std::setw(8) << s.linear_velocity[0] << " " << std::setw(8) << s.linear_velocity[1] << " " << std::setw(8) << s.linear_velocity[2]
            << "                    root_lin_vel_w: " << std::setw(8) << gt.vel[0] << " " << std::setw(8) << gt.vel[1] << " " << std::setw(8) << gt.vel[2] << "\n"
            << "  quat=  " << std::setw(8) << s.orientation_quat[0] << " " << std::setw(8) << s.orientation_quat[1] << " " << std::setw(8) << s.orientation_quat[2] << " " << std::setw(8) << s.orientation_quat[3]
            << "           root_quat_w: " << std::setw(8) << gt.quat[0] << " " << std::setw(8) << gt.quat[1] << " " << std::setw(8) << gt.quat[2] << " " << std::setw(8) << gt.quat[3] << "\n"
            << "  omega= " << s.angular_velocity[0] << " " << s.angular_velocity[1] << " " << s.angular_velocity[2] << "\n";
}

// ===== Inputs per-sample =====
struct DemoInputs {
  Eigen::Vector3d acc{0,0,9.81};
  Eigen::Vector3d gyro{0,0,0};
  Eigen::Vector3d F_lf{0,0,0}, F_rf{0,0,0}, F_lh{0,0,0}, F_rh{0,0,0};
  Eigen::Vector3d pos_lf_world{0,0,0}, pos_rf_world{0,0,0}, pos_lh_world{0,0,0}, pos_rh_world{0,0,0};
  std::vector<std::pair<std::string,double>> q_named; // joint position (optional)
  std::vector<std::pair<std::string,double>> v_named; // joint velocity (optional)
};


// ===== Stage 1: Attitude (persistent AttitudeBiasXKF) =====
AttitudeMsg run_attitude(state_estimator::AttitudeBiasXKF& att, const Config& cfg, const DemoInputs& in, double t)
{
  Eigen::Quaterniond quat_est, quat_dot;

  // Use current filter state to compute m_b
  Eigen::Matrix<double,7,1> xhat_estimated = att.getX();                        // CHECKED 1ST VALUE, needs extra check next
  quat_est.w() = xhat_estimated(0);
  quat_est.vec() << xhat_estimated(1),xhat_estimated(2),xhat_estimated(3);

  // Measurement vector z = [f_b; m_b]
  Eigen::Vector3d f_b = cfg.b_R_imu * in.acc;                                   // CHECKED: f_b matches original code
  Eigen::Vector3d m_b = iit::commons::quatToRotMat(quat_est) * cfg.north_vec;   // CHECKED

  Eigen::Matrix<double,6,1> z; z << f_b, m_b;                                   // CHECKED: z matches the original code

  // Update with current sample
  att.update(t, cfg.b_R_imu*in.gyro, z);                                        // CHECKED: update takes the same input and follows the same internal logic

  // Build output from new state
  xhat_estimated = att.getX();
  quat_est.w() = xhat_estimated(0);
  quat_est.vec() << xhat_estimated(1),xhat_estimated(2),xhat_estimated(3);

  Eigen::Matrix<double,7,1> xdot = att.calc_f(t, xhat_estimated, cfg.b_R_imu*in.gyro);    // CHECKED: xdot is obtained in the same way
  quat_dot.w() = xdot(0);
	quat_dot.vec() << xdot(1),xdot(2),xdot(3); 
  Eigen::Vector3d omega_filt = iit::commons::quatToOmega(quat_est, quat_dot);             // NOTE: original implementation: outdated estimated value

  //----------------------------------------------------------- OUTPUTS                          

  // Filtered omega from state derivative
  Eigen::Vector3d rpy = iit::commons::quatToRPY(quat_est);

  AttitudeMsg out;
  out.quaternion[0]=quat_est.w(); out.quaternion[1]=quat_est.x(); out.quaternion[2]=quat_est.y(); out.quaternion[3]=quat_est.z();
  out.roll_deg=rad2deg(rpy(0)); out.pitch_deg=rad2deg(rpy(1)); out.yaw_deg=rad2deg(rpy(2));
  out.angular_velocity[0]=omega_filt(0); out.angular_velocity[1]=omega_filt(1); out.angular_velocity[2]=omega_filt(2);
  return out;
}


// ===== Stage 2: Contact Detection =====
ContactDetectionMsg run_contacts(const Config& cfg, const DemoInputs& in)
{
  ContactDetectionMsg out;
  out.stance_lf = in.F_lf[2] < cfg.grf_threshold;
  out.stance_rf = in.F_rf[2] < cfg.grf_threshold;
  out.stance_lh = in.F_lh[2] < cfg.grf_threshold;
  out.stance_rh = in.F_rh[2] < cfg.grf_threshold;
  return out;
}


// ===== Stage 3: Leg Odometry (persistent Pinocchio model/data) =====
struct LegOdomContext {
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<std::size_t> foot_fids;

  explicit LegOdomContext(const Config& cfg)
  : model(), data(model), foot_fids()
  {
    pinocchio::urdf::buildModel(cfg.urdf_path, model);
    data = pinocchio::Data(model);
    foot_fids.reserve(cfg.foot_frames.size());
    for (const auto& name : cfg.foot_frames){
      //std::cout << "Foot frame: " << name << " frame ID: " << model.getFrameId(name) << "\n";
      foot_fids.push_back(model.getFrameId(name));
    }
  }
};

LegOdometryMsg run_leg_odom(LegOdomContext& ctx,
                            const Config& cfg,
                            const DemoInputs& in,
                            const AttitudeMsg& att,
                            const ContactDetectionMsg& c)
{
  // Build q,v from named inputs; fallback zeros
  Eigen::VectorXd q = Eigen::VectorXd::Zero(ctx.model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(ctx.model.nv);
  {
    std::unordered_map<std::string,double> qmap, vmap;
    for (auto& kv: in.q_named) qmap[kv.first]=kv.second;
    for (auto& kv: in.v_named) vmap[kv.first]=kv.second;
    //std::cout << "Leg Odom joint states---------------------------------------------\n";
    for (pinocchio::JointIndex i=1;i<ctx.model.njoints;++i){
      const std::string& jname = ctx.model.names[i];
      if (qmap.count(jname)) q[i-1]=qmap[jname];
      if (vmap.count(jname)) v[i-1]=vmap[jname];
      //std::cout << "Joint " << i << " " << jname << " q=" << q[i-1] << " v=" << v[i-1] << "\n";
    }
  }

  // Base angular velocity (use estimated omega_filt)
  Eigen::Vector3d omega(in.gyro[0], in.gyro[1], in.gyro[2]);  // The original code does not use the filtered version, and it actually seems less accurate with the mix2 example (in pos, in vel almost no diff)
  //Eigen::Vector3d omega(att.angular_velocity[0],att.angular_velocity[1],att.angular_velocity[2]);
  Eigen::Vector3d omega_rot = cfg.base_R_imu * omega;

  pinocchio::forwardKinematics(ctx.model, ctx.data, q, v);
  pinocchio::updateFramePlacements(ctx.model, ctx.data);

  
  // For each foot: LOCAL_WORLD_ALIGNED velocity minus ω×r
  std::vector<Eigen::Vector3d> foot_vels;
  foot_vels.reserve(ctx.foot_fids.size());
  
  // -----------------------------------------
  // base estimation using planned contact
  std::vector<Eigen::Vector3d> base_est_pc;
  base_est_pc.reserve(ctx.foot_fids.size());
  // -----------------------------------------

  // Convert attitude quaternion to rotation (pineapple pizza)
  Eigen::Quaterniond quat_est;
  quat_est.w() = att.quaternion[0];
  quat_est.vec() << att.quaternion[1], att.quaternion[2], att.quaternion[3];
  Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();
  
  for (std::size_t idx=0; idx<ctx.foot_fids.size(); ++idx){
    std::size_t frame_id = ctx.foot_fids[idx];
    pinocchio::Motion foot_vel_global = pinocchio::getFrameVelocity(ctx.model, ctx.data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector3d foot_pos_base = ctx.data.oMf[frame_id].translation();
    Eigen::Vector3d omega_cross_r = omega_rot.cross(foot_pos_base); // Compute velocity contribution from base angular motion: ω x r
    Eigen::Vector3d rel_vel = -(foot_vel_global.linear() + omega_cross_r); // compensate base rot
    foot_vels.push_back(rel_vel);
    
    // simulate use planned contacts info when the foot is in contact (only z will be relevant)
    // Eigen::Vector3d x_0_b_est
    if (idx==0) base_est_pc.push_back(in.pos_rf_world - w_R_b*foot_pos_base);
    if (idx==1) base_est_pc.push_back(in.pos_lf_world - w_R_b*foot_pos_base);
    if (idx==2) base_est_pc.push_back(in.pos_rh_world - w_R_b*foot_pos_base);
    if (idx==3) base_est_pc.push_back(in.pos_lh_world - w_R_b*foot_pos_base);
  }

  // NOTE: cfg.foot_frames set to {"FR_FOOT","FL_FOOT","HR_FOOT","HL_FOOT"}; adjust mapping if order differs.
  if (ctx.foot_fids.size()!=4){exit(0);}
  Eigen::Vector3d lin_leg_rf = foot_vels[0];
  Eigen::Vector3d lin_leg_lf = foot_vels[1];
  Eigen::Vector3d lin_leg_rh = foot_vels[2];
  Eigen::Vector3d lin_leg_lh = foot_vels[3];

  // base height estimation from contacts, world
  double base_z_est_pc_rf = base_est_pc[0].z();
  double base_z_est_pc_lf = base_est_pc[1].z();
  double base_z_est_pc_rh = base_est_pc[2].z();
  double base_z_est_pc_lh = base_est_pc[3].z();
  
  // Weighted average of stance feet
  double w_lf = c.stance_lf?1.0:0.0, w_rf=c.stance_rf?1.0:0.0, w_lh=c.stance_lh?1.0:0.0, w_rh=c.stance_rh?1.0:0.0;
  double sum = w_lf+w_rf+w_lh+w_rh;
  if (sum < 1) { sum = 0.01; } // avoid div-by-zero if all swing
  static Eigen::Vector3d v_base = Eigen::Vector3d::Zero();
  static double base_z_est_world = cfg.z0_IMU; // used before any contact is detected


  if(sum>=1){
    // use planned contact to estimate base height
    base_z_est_world = (w_lf*base_z_est_pc_lf + w_rf*base_z_est_pc_rf + w_lh*base_z_est_pc_lh + w_rh*base_z_est_pc_rh) / sum;
    // velocity msr from kinematics
    v_base = (w_lf*lin_leg_lf + w_rf*lin_leg_rf + w_lh*lin_leg_lh + w_rh*lin_leg_rh) / sum;
    v_base = w_R_b * v_base; // to world
  }else{
    //std::cout << "bypassing leg odometry while flying, prev v_base:" << v_base.transpose() << "\n";
  }
  
  
  
  LegOdometryMsg out;
  auto copy=[&](double* dst,const Eigen::Vector3d& v){ dst[0]=v(0); dst[1]=v(1); dst[2]=v(2); };
  copy(out.lin_vel_lf, lin_leg_lf);
  copy(out.lin_vel_rf, lin_leg_rf);
  copy(out.lin_vel_lh, lin_leg_lh);
  copy(out.lin_vel_rh, lin_leg_rh);
  copy(out.base_velocity, v_base);
  out.base_z_viapc = base_z_est_world;
  return out;
}


// ===== Stage 4: Sensor Fusion (persistent KFSensorFusion) =====
SensorFusionMsg run_sensor_fusion(state_estimator::KFSensorFusion& kf,
                                  const Config& cfg,
                                  const DemoInputs& in,
                                  const AttitudeMsg& att,
                                  const LegOdometryMsg& lo,
                                  double t)
{
  // Inputs
  Eigen::Vector3d acc = in.acc; // imu frame

  // Reading attitude estimation	
  Eigen::Quaterniond quat_est;
  quat_est.w() = att.quaternion[0];
	quat_est.vec() << att.quaternion[1], att.quaternion[2], att.quaternion[3];
  
  Eigen::Vector3d rpy = iit::commons::quatToRPY(quat_est);
  Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose();

  Eigen::Vector3d v_b (lo.base_velocity[0], lo.base_velocity[1], lo.base_velocity[2]); // world
  
  // simulate planned contacts drift correction here
  // the base velocity is updated to slowly converge to the planned height
  Eigen::Matrix<double,6,1> xhat_estimated_current = kf.getX();
  // check that this is active only if at least one foot is in contact
  v_b[2] = v_b[2] + cfg.rho*(lo.base_z_viapc - xhat_estimated_current(2));
  
  // reading acceleration from imu
  Eigen::Vector3d f_b = cfg.base_R_imu * acc;

  // input u = w_R_b*f_b - gravity
  Eigen::Vector3d gravity(0,0,-9.81);
  Eigen::Vector3d u = w_R_b * f_b + gravity;

  // prediction
  kf.predict(t, u);
  
  // reading leg odometry
  Eigen::Vector3d w_v_b = v_b; //w_R_b*v_b; bug

  Eigen::Vector3d z_proprio;
	z_proprio << w_v_b;
  
  kf.update(t, z_proprio);

  Eigen::Matrix<double,6,1> xhat_estimated = kf.getX();

  SensorFusionMsg out;
  out.position[0]=xhat_estimated(0); out.position[1]=xhat_estimated(1); out.position[2]=xhat_estimated(2);
  out.linear_velocity[0]=xhat_estimated(3); out.linear_velocity[1]=xhat_estimated(4); out.linear_velocity[2]=xhat_estimated(5);
  out.orientation_quat[0]=att.quaternion[0]; out.orientation_quat[1]=att.quaternion[1];
  out.orientation_quat[2]=att.quaternion[2]; out.orientation_quat[3]=att.quaternion[3];
  out.angular_velocity[0]=att.angular_velocity[0]; out.angular_velocity[1]=att.angular_velocity[1]; out.angular_velocity[2]=att.angular_velocity[2];
  return out;
}

double computeRho(double Ts, double Tconv, double alpha = 0.02) {
  // Ts     = sampling period (s)
  // Tconv  = desired convergence time (s)
  // alpha  = remaining error fraction (default 2%)
    return (1.0 - std::pow(alpha, Ts / Tconv)) / Ts;
}




struct MuJoCoExtras {
    Eigen::Vector4d out_quat{0,0,0,1};      // (w,x,y,z) as logged
    double out_roll_deg  = 0.0;
    double out_pitch_deg = 0.0;
    double out_yaw_deg   = 0.0;

    Eigen::Vector3d out_p_trunk{0,0,0};
    Eigen::Vector3d out_v_trunk_sf{0,0,0};
};

double read_from_mujoco(std::vector<double> &data, DemoInputs &in, MuJoCoExtras& mj, GroundTruth& gt) {
    // --- Sanity check ---
    if (data.size() < 63) {
        std::cerr << "[read_from_mujoco] Error: expected at least 63 doubles, got "
                  << data.size() << std::endl;
        return 0.0;
    }

    int idx = 0;

    double t = data[idx++];  // time [s]
    idx++; // skip clock

    // --- RBQ pose and velocity ---
    gt.pos = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    gt.vel = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    gt.rpy = Eigen::Vector3d(data[idx]*180.0/M_PI, data[idx+1]*180.0/M_PI, data[idx+2]*180.0/M_PI); idx += 3;

    // --- IMU data ---
    in.gyro = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    in.acc  = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;

    // --- Foot forces ---
    in.F_rh = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    in.F_lh = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    in.F_rf = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    in.F_lf = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;

    // --- Joint positions ---
    in.q_named = {
      {"joint0_HRR", data[idx++]},
      {"joint1_HRP", data[idx++]},
      {"joint2_HRK", data[idx++]},
      {"joint3_HLR", data[idx++]},
      {"joint4_HLP", data[idx++]},
      {"joint5_HLK", data[idx++]},
      {"joint6_FRR", data[idx++]},
      {"joint7_FRP", data[idx++]},
      {"joint8_FRK", data[idx++]},
      {"joint9_FLR", data[idx++]},
      {"joint10_FLP",data[idx++]},
      {"joint11_FLK",data[idx++]}};
    in.v_named = {
      {"joint0_HRR", data[idx++]},
      {"joint1_HRP", data[idx++]},
      {"joint2_HRK", data[idx++]},
      {"joint3_HLR", data[idx++]},
      {"joint4_HLP", data[idx++]},
      {"joint5_HLK", data[idx++]},
      {"joint6_FRR", data[idx++]},
      {"joint7_FRP", data[idx++]},
      {"joint8_FRK", data[idx++]},
      {"joint9_FLR", data[idx++]},
      {"joint10_FLP",data[idx++]},
      {"joint11_FLK",data[idx++]}};

    // --- known foot world positions ---
    in.pos_rh_world = Eigen::Vector3d(0,0,0);
    in.pos_lh_world = Eigen::Vector3d(0,0,0);
    in.pos_rf_world = Eigen::Vector3d(0,0,0);
    in.pos_lf_world = Eigen::Vector3d(0,0,0);

     // Outputs (extras)
    mj.out_quat = Eigen::Vector4d(data[idx], data[idx+1], data[idx+2], data[idx+3]); idx += 4;
    mj.out_roll_deg  = data[idx++];
    mj.out_pitch_deg = data[idx++];
    mj.out_yaw_deg   = data[idx++];

    // --- Trunk position and velocity ---
    mj.out_p_trunk    = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;
    mj.out_v_trunk_sf = Eigen::Vector3d(data[idx], data[idx+1], data[idx+2]); idx += 3;

    return t;
}


int main(int argc, char** argv) {

  //std::ios::sync_with_stdio(false);

  Config cfg;

  // Allow overriding URDF / CSV path
  const std::string csv_path = (argc>1? std::string(argv[1]) : std::string("samples/just-throw-error.csv"));
  if (argc>2) cfg.urdf_path = argv[2];

  // override planned contact gain
  cfg.rho = computeRho(0.005, 1.0, 0.02);
  
  // Persistent estimators
  // Attitude initial state: [qw qx qy qz bx by bz]
  Eigen::Matrix<double,7,1> xhat;
  xhat << 1,0,0,0, 0,0,0; xhat.head<4>().normalize();

  state_estimator::AttitudeBiasXKF att(0.0, xhat, cfg.P0, cfg.Q, cfg.R, cfg.gravity_vec, cfg.north_vec, cfg.ki, cfg.kp);

  std::cout << "$$$$$$$$$$$$$$$$$$$$$$$ Attitude config $$$$$$$$$$$$$$$$$$$$$$$\n";
  std::cout << "t: 0.0\n";
  std::cout << "xhat: " << xhat.transpose() << "\n";
  std::cout << "P0: " << cfg.P0 << "\n";
  std::cout << "Q: "  << cfg.Q << "\n";
  std::cout << "R: "  << cfg.R << "\n";
  std::cout << "gravity: "  << cfg.gravity_vec.transpose() << "\n";
  std::cout << "north vec: "  << cfg.north_vec.transpose() << "\n";
  std::cout << "ki: "  << cfg.ki << "\n";
  std::cout << "kp: "  << cfg.kp << "\n";
  std::cout << "b_R_imu:\n" << cfg.b_R_imu << "\n";
  std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n";
 
  
  state_estimator::KFSensorFusion kf(0.0, cfg.xhat6_init, cfg.SF_P, cfg.SF_Q, cfg.SF_R, false, false);

  std::cout << "$$$$$$$$$$$$$$$$$$$$$$$ Sensor Fusion config $$$$$$$$$$$$$$$$$$$$$$$\n";
  std::cout << "t: 0.0\n";
  std::cout << "xhat: " << cfg.xhat6_init.transpose() << " (from sensor_fusion_plugin.cpp, modified for initial conditions)\n";
  std::cout << "P: " << cfg.SF_P << "\n";
  std::cout << "Q: " << cfg.SF_Q << "\n";
  std::cout << "R: " << cfg.SF_R << "\n";
  std::cout << "lidar: false (from sensor_fusion_plugin.cpp)\n";
  std::cout << "slippage: false (from sensor_fusion_plugin.cpp)\n";
  std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n";  

  LegOdomContext legctx(cfg);


  // I/O
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << csv_path << "\n";
    return 1;
  }

  // Optional: write outputs
  std::ofstream outcsv(csv_path+".output.csv");
  outcsv << std::fixed << std::setprecision(6);
  outcsv << "t,,muse_roll,muse_pitch,muse_yaw,rbq_roll,rbq_pitch,rbq_yaw,,"
       << "muse_qw,muse_qx,muse_qy,muse_qz,,"
       << "muse_px,muse_py,muse_pz,rbq_px,rbq_py,rbq_pz,,"
       << "muse_vx,muse_vy,muse_vz,rbq_vx,rbq_vy,rbq_vz,,"
       << "muse_vx_odom,muse_vy_odom,muse_vz_odom,,"
       << "muse_z_via_planned_cont,muse_IMU_z_via_planned_cont,,"
       << "muse_IMU_x,muse_IMU_y,muse_IMU_z,muse_IMU_vx,muse_IMU_vy,muse_IMU_vz,muse_IMU_vx_odom,muse_IMU_vy_odom,muse_IMU_vz_odom,,"
       << "old_muse_roll,old_muse_pitch,old_muse_yaw,old_muse_qw,old_muse_qx,old_muse_qy,old_muse_qz,,"
       << "old_muse_px,old_muse_py,old_muse_pz,old_muse_vx,old_muse_vy,old_muse_vz\n";
       

  std::string line;
  std::size_t line_no = 0;
  DemoInputs in;
  MuJoCoExtras mj;
  GroundTruth gt;

  // Record start time
  auto start = std::chrono::high_resolution_clock::now();


  while (std::getline(file, line)) {
    ++line_no;
    if (line_no == 1) continue; // skip headers
    if (line.empty()) continue;

    std::vector<double> data;
    data.reserve(128);
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
      if (token.empty()) { data.push_back(0.0); continue; }
      try {
        data.push_back(std::stod(token));
      } catch (...) {
        std::cerr << "Warning: non-numeric token on line " << line_no << ": '" << token << "' -> 0\n";
        data.push_back(0.0);
      }
    }

    double t;
    t = read_from_mujoco(data, in, mj, gt);
 
    
    
    // ---- Pipeline (persistent) ----
    AttitudeMsg att_out = run_attitude(att, cfg, in, t);
    //print_attitude(att_out,gt);

    ContactDetectionMsg contacts = run_contacts(cfg, in);
    //print_contacts(contacts);

    LegOdometryMsg lo = run_leg_odom(legctx, cfg, in, att_out, contacts);
    //print_legodom(lo);

    SensorFusionMsg sf = run_sensor_fusion(kf, cfg, in, att_out, lo, t);
    //print_sensorfusion(sf,cfg,gt);



    // ===== Convert IMU to trunk link ===============================================================================
    Eigen::Vector3d t_i2t(-0.02557, 0.0, 0.04232);          // trunk expressed in IMU frame

    // Rotation IMU->world (your code uses transpose of quatToRotMat)
    Eigen::Quaterniond quat_est;
    quat_est.w() = att_out.quaternion[0];
    quat_est.vec() << att_out.quaternion[1], att_out.quaternion[2], att_out.quaternion[3];
    Eigen::Matrix3d w_R_b = iit::commons::quatToRotMat(quat_est).transpose(); // rotation from IMU fr to world fr

    // Vector IMU->trunk in world
    Eigen::Vector3d v_IMU_trunk_w = w_R_b * t_i2t;

    // Positions (world)
    Eigen::Vector3d p_IMU(sf.position[0], sf.position[1], sf.position[2]);
    Eigen::Vector3d p_trunk = p_IMU + v_IMU_trunk_w;

    // Angular velocity: use gyro (body) rotated to world
    Eigen::Vector3d omega_b = in.gyro;          // rad/s in IMU/body frame
    Eigen::Vector3d omega_w = w_R_b * omega_b;   // world

    // Velocities (world)
    Eigen::Vector3d v_IMU_odom(lo.base_velocity[0], lo.base_velocity[1], lo.base_velocity[2]);
    Eigen::Vector3d v_IMU_sf  (sf.linear_velocity[0], sf.linear_velocity[1], sf.linear_velocity[2]);

    Eigen::Vector3d v_trunk_odom = v_IMU_odom + omega_w.cross(v_IMU_trunk_w);
    Eigen::Vector3d v_trunk_sf   = v_IMU_sf   + omega_w.cross(v_IMU_trunk_w);

    // Planned-contact z (world)
    double z_trunk_viapc = lo.base_z_viapc + v_IMU_trunk_w.z();

    // ================================================================================================================================

    /*std::cout << "-------------------------------------\n";
    std::cout << "gyro:  " << std::setw(8) << in.gyro(0) << " " << std::setw(8) << in.gyro(1) << " " << std::setw(8) << in.gyro(2) << "\n";
    std::cout << "acc:   " << std::setw(8) << in.acc(0) << " " << std::setw(8) << in.acc(1) << " " << std::setw(8) << in.acc(2) << "\n";
    std::cout << "time:  " << std::setw(8) << data[0] << "\n";
    std::cout << "=====================================\n";
    std::cout << "=====================================\n";*/


    outcsv << t << ",,"
       << att_out.roll_deg << "," << att_out.pitch_deg << "," << att_out.yaw_deg << ","
       << gt.rpy[0] << "," << gt.rpy[1] << "," << gt.rpy[2] << ",," 

       // MUSE quaternion
       << att_out.quaternion[0] << "," << att_out.quaternion[1] << "," << att_out.quaternion[2] <<  "," << att_out.quaternion[3] << ",," 

       // Position MUSE/RBQ
       << p_trunk.x() << "," << p_trunk.y() << "," << p_trunk.z() << ","
       << gt.pos[0] << "," << gt.pos[1] << "," << gt.pos[2] << ",,"
       
       // Velocity MUSE/RBQ
       << v_trunk_sf.x()   << "," << v_trunk_sf.y()   << "," << v_trunk_sf.z()   << ","
       << gt.vel[0] << "," << gt.vel[1] << "," << gt.vel[2] << ",,"
       
       // Velocity odometry MUSE
       << v_trunk_odom.x() << "," << v_trunk_odom.y() << "," << v_trunk_odom.z() << ",,"
       
       // z via planned contact MUSE/MUSE_IMU
       << z_trunk_viapc << "," << lo.base_z_viapc << ",,"
       
       // IMU pos, vel, vel_odom
       << sf.position[0] << "," << sf.position[1] << "," << sf.position[2] << ","
       << sf.linear_velocity[0] << "," << sf.linear_velocity[1] << "," << sf.linear_velocity[2] << ","
       << lo.base_velocity[0] << "," << lo.base_velocity[1] << "," << lo.base_velocity[2] << ",,"

       // OLD MUSE roll, pitch, yaw, quaternion
       << mj.out_roll_deg << "," << mj.out_pitch_deg << "," << mj.out_yaw_deg << ","
       << mj.out_quat[0] << "," << mj.out_quat[1] << "," << mj.out_quat[2] << "," << mj.out_quat[3] << ",,"

       // OLD MUSE pos, vel
       << mj.out_p_trunk[0] << "," << mj.out_p_trunk[1] << "," << mj.out_p_trunk[2] << ","
       << mj.out_v_trunk_sf[0] << "," << mj.out_v_trunk_sf[1] << "," << mj.out_v_trunk_sf[2] << "\n";
  }

  // Record end time
  auto end = std::chrono::high_resolution_clock::now();

  // Calculate duration
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // Output the duration
  std::cout << "Time taken: " << duration.count()*0.001 << " ms\n" << duration.count()*0.001/line_no << "ms per step\n" << line_no/(duration.count()*0.000001) << "Hz" << std::endl;

  std::cout << "Done. Wrote estimator outputs to estimator_outputs.csv\n";
  return 0;
}
