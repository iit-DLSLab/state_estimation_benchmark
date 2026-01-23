// Copyright (c) 2023-2024. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2024.

#pragma once
#include <string>
#include <eigen3/Eigen/Core>

//#define LAIKAGO
// #define ANYMAL
#define HOUND

const int NumberOfJointsPerLeg = 3;
const int NumberOfLegs = 4;
const int NumberOfControlInputs = NumberOfJointsPerLeg*NumberOfLegs;    /// # of control inputs: force xyz for each leg
const int NumberOfDeltaStates = 12;    /// # of delta states: [phi pCoM w vCoM]'
const int NumberOfStates = 18; /// #of states [R, p, w, v]
const int NumberOfDeltaStatesAndControlInputs = NumberOfDeltaStates+NumberOfControlInputs;
const int MpcHorizonNumber = 8;
const int NumberOfFootPosition = 3*NumberOfLegs;
const int SensorMeasurementsSize = 6+NumberOfControlInputs+NumberOfControlInputs+NumberOfControlInputs; /// IMU [Gyro 3x1, Acc 3x1] , Encoder [NumberOfControlInputs x 1], EncoderDot [NumberOfControlInputs x 1]

enum HoundState : int {
// enum AnymalState : int {
  UNDEFINED = 0,
  SHUTTING_DOWN,
  BOOTING,
  BOOTED,
  HALT,
  COMM_DISABLED,
  COMM_ENABLED,
  MOTOR_DISABLED,
  MOTOR_ENABLED,
  ZERO_CONTROL,
  PDTEST_CONTROL,
  NULL_CONTROL,
  MOVING_TO_HOME_POSE,
  IN_TEST_MODE,
  IN_CONTROL,
  NMPC_CONTROL_TEMP,
  NMPC_CONTROL
};
enum FsmState : int {
  SAFETY_MOTION_START = -4,
  SAFETY_MOTION,
  PASSIVE_CONTROL,
  PD_SIT,
  PD_STANDING,
  NMPC_STANDING, // 4다리가 모두 땅에 붙어있는 case
  NMPC_STANCE, // NMPC swing 할 때 땅에 붙어있는 다리의 state
  NMPC_SWING // NMPC swing 할 때 땅에 떨어져있는 다리의 state
};

enum PdMode : int {
  JOINT_PD = 0,
  CARTESIAN_PD,
  CUSTOMPD
};



enum PdFsmState : int {
  PASSIVE_TO_PD_STAND = 0,
  NMPC_STAND_TO_PD_STAND,
  PD_STAND_TO_PD_SIT
};

typedef struct _LiDAR_STRUCT_
{
  Eigen::Matrix<double,3,1> X_LiDAR_Raw_i;
  Eigen::Matrix<double, 3, 1> X_LiDAR_i, sqrt_LiDAR_i;
  bool LiDAR_In_i = false;
  bool LiDAR_logging_In_i = false;
} LiDAR_STRUCT;

typedef struct _GPS_STRUCT_
{
  Eigen::Matrix<double,3,1> X_GPS_Raw_i;
  Eigen::Matrix<double, 3, 1> X_GPS_i, sqrt_GPS_i;
  bool GPS_In_i = false;
  bool GPS_logging_In_i = false;
} GPS_STRUCT;

typedef struct _LVM_STRUCT_
{
  Eigen::Matrix<double, 3, 1> X_LVM_i, sqrt_LVM_i;
  bool LVM_In_i = false;
  bool LVM_logging_In_i = false;
} LVM_STRUCT;

typedef struct _ESTIMATED_STATES_ {
  Eigen::Matrix<double, NumberOfStates, 1> state_estimated; // orientation, position, angular velocity, linear velocity
  Eigen::Matrix<double, NumberOfFootPosition, 1> fpos;
  Eigen::Matrix<double, NumberOfFootPosition, 1> fvel;
  Eigen::Matrix<double, NumberOfFootPosition, 1> pos_hip_to_foot_local;
  Eigen::Matrix<double, NumberOfFootPosition, 1> vel_hip_to_foot_local;
  Eigen::Matrix<double, NumberOfFootPosition, 1> pos_hip_to_foot_world;
  Eigen::Matrix<double, NumberOfFootPosition, 1> vel_hip_to_foot_world;
  Eigen::Matrix<double, NumberOfFootPosition, NumberOfFootPosition> jacobian_local;
  Eigen::Matrix<double, NumberOfFootPosition, NumberOfFootPosition> jacobian_world;
  Eigen::Matrix<double, NumberOfFootPosition, NumberOfFootPosition> djacobian_local;
  Eigen::Matrix<double, NumberOfFootPosition, NumberOfFootPosition> djacobian_world;
  Eigen::Matrix<double,3,1> est_plane;
  Eigen::Matrix<double,3,1> ws_accb;
  Eigen::Matrix<double,3,1> ws_wb;
  Eigen::Matrix<double,3,3> ws_r_pel;
  Eigen::Matrix<double,3,1> eul_zyx;
  Eigen::Matrix<double, NumberOfControlInputs, 1> residual_all;// : residual
  Eigen::Matrix<bool, NumberOfControlInputs, 1> contact_all;// : contact
  Eigen::Matrix<double,3,3> rotation_plane;
  Eigen::Matrix<bool, NumberOfLegs, 1> Contact;
  Eigen::Matrix<bool, NumberOfLegs, 1> ContactForce;
  Eigen::Matrix<bool, NumberOfLegs, NumberOfJointsPerLeg> ContactRaw;
  Eigen::Matrix<bool, NumberOfLegs, 1> Slip;
  Eigen::Matrix<bool, NumberOfLegs, 1> Hard_Contact;
  Eigen::Matrix<bool, NumberOfLegs, 1> Contact_acc;
  Eigen::Matrix<bool, NumberOfLegs, 1> Contact_Trust;
  Eigen::Matrix<double,NumberOfJointsPerLeg,NumberOfLegs> Contact_Residual;
  Eigen::Matrix<double,NumberOfJointsPerLeg,NumberOfLegs> Contact_Acc_Residual;
  Eigen::Matrix<double,3,1> bias_gyro;
  Eigen::Matrix<double,3,1> bias_acc;
  Eigen::Matrix<double,NumberOfControlInputs,1> dv;
  Eigen::Matrix<double,9,9> covariance;

  int iteration_number=0;
  int total_backppgn_number=0;
  int backprop_number=0;
} EstimatedStates;

typedef struct _DESIRED_STATES_ {
  Eigen::Matrix<double, NumberOfStates, 1> state_desired;
} DesiredStates;

typedef struct _TRUE_STATES_ {
  Eigen::Matrix<double, NumberOfStates, 1> state_true;
  Eigen::Matrix<bool, NumberOfLegs, 1> contact_true;
  Eigen::Matrix<bool,NumberOfLegs,1> trueSlip;
  Eigen::Matrix<double,NumberOfControlInputs,1> dv;
} TrueStates;


struct Joystick_Input {
  Eigen::Matrix<double,3,1> joystick_command; // [vx vy omega]
  Eigen::Matrix<double,3,1> navigation_command; // [vx vy omega]
  Eigen::Matrix<bool, 6, 1> flag_fsm_joy; // [stand start sit end sw_trigger slope]
  Eigen::Matrix<bool, 6, 1> flag_fsm_joy_temp; // [stand start sit end sw_trigger slope]
  double pitch_d;
  double zd;
  double lpf_freq;
};


struct LegParameter {
  // 0: hip roll, 1: hip pitch, 2: knee pitch

  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> encoder_resolution;
  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> peak_current;
  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> rated_current;
  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> rated_current_rms;
  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> rated_torque;
  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> torque_constant;
  Eigen::Matrix<double, NumberOfJointsPerLeg, 1> gear_ratio;

  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> conv_joint_torque_to_motor_torque;  // tau_m = convJointTorque2MotorTorque*tau_j
  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_joint_torque_to_motordrive_current;

  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_joint_angle_to_motor_angle;
  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_joint_angle_to_motordrive_enc_cnt;

  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_motor_torque_to_joint_torque;
  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_motordrive_current_to_joint_torque;

  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_motor_angle_to_joint_angle;
  Eigen::Matrix<double, NumberOfJointsPerLeg, NumberOfJointsPerLeg> convert_motordrive_enc_cnt_to_joint_angle;

  LegParameter() {
	encoder_resolution(0) = pow(2, 19);
	encoder_resolution(1) = pow(2, 19);
	encoder_resolution(2) = pow(2, 19);

	peak_current(0) = 22.624;
	peak_current(1) = 13.1502;
	peak_current(2) = 13.1502;

	rated_current_rms(0) = 13.890;
	rated_current_rms(1) = 13.890;
	rated_current_rms(2) = 13.890;

	torque_constant(0) = 0.72;
	torque_constant(1) = 0.72;
	torque_constant(2) = 0.72;

	gear_ratio(0) = 4.0 / 1.0;
	gear_ratio(1) = 4.0 / 1.0;
	gear_ratio(2) = 119.0 / 15.0;

	conv_joint_torque_to_motor_torque << 1.0 / gear_ratio(0), 0, 0,
		0, 1.0 / gear_ratio(1), -1.0 / (gear_ratio(1) * gear_ratio(2)),
		0, 0, 1.0 / gear_ratio(2);

//            convJointTorque2MotorTorque << 1.0/gearRatio(0), 0, 0,
//                    0, 1.0/gearRatio(1), 0,
//                    0, -1.0/(gearRatio(2)*gearRatio(1)), 1.0/gearRatio(2);

	convert_joint_torque_to_motordrive_current = 1000.0 * rated_current_rms.asDiagonal().inverse().toDenseMatrix()
		* torque_constant.asDiagonal().inverse().toDenseMatrix() * conv_joint_torque_to_motor_torque;
	convert_joint_angle_to_motor_angle = conv_joint_torque_to_motor_torque.transpose().inverse();
	convert_joint_angle_to_motordrive_enc_cnt =
		encoder_resolution.asDiagonal().toDenseMatrix() * convert_joint_angle_to_motor_angle / (2 * M_PI);

	convert_motor_torque_to_joint_torque = conv_joint_torque_to_motor_torque.inverse();
	convert_motordrive_current_to_joint_torque = convert_joint_torque_to_motordrive_current.inverse();
	convert_motor_angle_to_joint_angle = convert_joint_angle_to_motor_angle.inverse();
	convert_motordrive_enc_cnt_to_joint_angle = convert_joint_angle_to_motordrive_enc_cnt.inverse();

  }
};
struct PDGain {
  double p;
  double d;

  operator std::string() const {
	std::ostringstream out;
	out << "p: " << p << ", d: " << d;
	return out.str();
  }
};


namespace Robot
{


#ifdef ALIENGO

const double gravity = 9.80665;
const double mass = 9.041 + 4*(1.993+0.639+0.207);
const double height = 0.371;

const double Ixx = 0.033260231;
const double Iyy = 0.16117211;
const double Izz = 0.17460442;

const double mu = 0.6;
const double min_friction = 10;
const double max_friction = 400;
const double max_linear_acc = 6;

const int num_states = 18;
const int num_mpc_state = 12;
const int num_inputs = 12;
const int NumProblemUnit = num_mpc_state + num_inputs;
const int num_legs = 4;

const double pelvis_to_roll[] = {0.2399,0.051,0};
const double roll_to_pitch = 0.083;
const double foot_radius = 0.0265;
const double thigh_length = 0.25;
const double calf_length = 0.25;
const double shoulder_width = 0.15;
const double shoulder_depth = 0.6477;


enum JointSequentialNumber
{
	RRH = 0, RRT, RRC, RLH, RLT, RLC,
	FRH, FRT, FRC, FLH, FLT, FLC,
	NO_OF_JOINTS
};


const std::string JointNameList[NO_OF_JOINTS] = {
	"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
	"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
	"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
	"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"
};




#endif

#ifdef LAIKAGO


const double gravity = 9.80665;
const double mass = 13.733 + 4*(1.096+1.528+0.241);
const double height = 0.401;

const double Ixx = 0.073348887;
const double Iyy = 0.250684593;
const double Izz = 0.254469458;

const double mu = 0.6;
const double min_friction = 10;
const double max_friction = 400;
const double max_linear_acc = 6;

const int num_states = 18;
const int num_mpc_state = 12;
const int num_inputs = 12;
const int NumProblemUnit = num_mpc_state + num_inputs;
const int num_legs = 4;

const double pelvis_to_roll[] = {0.21935,0.0875,0};

const double roll_to_pitch = 0.037;
//const double foot_radius = 0.0265 + 0.011;
const double foot_radius = 0.0265;
const double thigh_length = 0.25;
const double calf_length = 0.25;
const double shoulder_width = 0.249;
const double shoulder_depth = 0.4387;



const double leg_sign[] = {-1.0,-1.0,-1.0,1.0,1.0,-1.0,1.0,1.0};

enum JointSequentialNumber
{
	RRH = 0, RRT, RRC, RLH, RLT, RLC,
	FRH, FRT, FRC, FLH, FLT, FLC,
	NO_OF_JOINTS
};


const std::string JointNameList[NO_OF_JOINTS] = {
	"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
	"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
	"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
	"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"
};
#endif

#ifdef ANYMAL


const double Gravity = 9.80665;
const double Mass = 16.7935 + 4*(1.4246+1.63498+0.207+0.1401);
const double Height = 0.401;

const double Ixx = 0.217391101503;
const double Iyy = 0.639432546734;
const double Izz = 0.62414077654;

const double mu = 0.6;
const double min_friction = 10;
const double max_friction = 400;
const double max_linear_acc = 6;

const int num_states = 18;
const int num_mpc_state = 12;
const int num_inputs = 12;
const int NumProblemUnit = num_mpc_state + num_inputs;
const int num_legs = 4;

const double PelvisToRoll[] = {0.277,0.116,0};

const double RollToPitch = 0.041;
//const double foot_radius = 0.0265 + 0.011;
const double FootRadius = 0.031;
const double FootRadiusForEstimator = 0.031;
const double ThighLength = 0.25;
const double CalfLength = 0.32125;
const double ShoulderWidth = 0.249;
const double ShoulderDepth = 0.4387;



const double LegSign[] = {-1.0,-1.0,-1.0,1.0,1.0,-1.0,1.0,1.0};

enum JointSequentialNumber
{
	RRH = 0, RRT, RRC, RLH, RLT, RLC,
	FRH, FRT, FRC, FLH, FLT, FLC,
	NO_OF_JOINTS
};


const std::string JointNameList[NO_OF_JOINTS] = {
	"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
	"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
	"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
	"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"
};
// }
#endif


#ifdef HOUND

const double Gravity = 9.80665;
//    const double mass = 17.66173773 + 4*(3.96168582+1.52408165+0.68173107);
const double Mass = 50;         // Hound:45;  Hound2: 50;
const double Height = 0.48;     // Hound:0.48; Hound2: 0.51;

const double Ixx = 0.20856455;
const double Iyy = 0.80371419;
const double Izz = 0.95938169;

const int NumStates = 18;
const int NumMpcState = 12;
const int NumInputs = 12;
const int NumProblemUnit = NumMpcState + NumInputs;
const int NumLegs = 4;

const double PelvisToRoll[] = {0.349, 0.1, 0};
const double PelvisToNominal[] = {0.349, 0.17, 0};

// const double RollToPitch = 0.1135;
// //const double FootRadius = 0.039;
// const double FootRadius = 0.0;
// //const double FootRadiusForEstimator = 0.039;
// const double FootRadiusForEstimator = 0.0;
// const double ThighLength = 0.3279;
// const double CalfLength = 0.35;
// const double ShoulderWidth = 0.335;
// const double ShoulderDepth = 0.562;

// HOUND2
const double RollToPitch = 0.1286;
//const double FootRadius = 0.039;
const double FootRadius = 0.0;
const double ThighLength = 0.3485;
const double CalfLength = 0.3505;
const double ShoulderWidth = 0.335;
const double ShoulderDepth = 0.558;
const double FootRadiusForEstimator = 0.0;

//for NMPC
const double LegSign[] = {-1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0};

enum JointSequentialNumber {
  RRH = 0,
  RRT,
  RRC,
  RLH,
  RLT,
  RLC,
  FRH,
  FRT,
  FRC,
  FLH,
  FLT,
  FLC,
  NO_OF_JOINTS
};

const std::string JointNameList[NO_OF_JOINTS] = {
	"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
	"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
	"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
	"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"
};
}

#endif