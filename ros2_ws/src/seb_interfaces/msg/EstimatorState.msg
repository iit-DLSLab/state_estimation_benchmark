# Base state published by the online state-estimation benchmark (seb_online).
# One message per processed LowState sample. Monitoring-only: this message is an
# estimator output, it never commands the robot.
#
# Frames
#   header.frame_id : world frame (gravity-aligned, z up; origin and yaw are the
#                     estimator's initial base pose).
#   child_frame_id  : robot base frame (the URDF "base" link).
#
# Time
#   header.stamp is the time of the STATE, i.e. the timestamp of the LowState
#   sample the state was computed from (see stamp_source), NOT the publication
#   time. Publication-related timing is reported in the *_s fields below.

std_msgs/Header header
string child_frame_id

# Estimator that produced this state: "muse", "iekf" or "invariant_smoother".
string estimator

# Internal identifier of the LowState sample (monotonic, consecutive among the
# samples dispatched to the workers).
uint64 sample_id
# LowState.tick of the source sample (robot MCU counter, ~1 ms per count).
uint32 lowstate_tick

# How header.stamp was obtained.
uint8 STAMP_RECEIVE_TIME=0    # ROS clock when the LowState message was received by this node
uint8 STAMP_LOWSTATE_TICK=1   # LowState.tick * tick_period, offset to the ROS clock at the first sample
uint8 stamp_source

# Status of the estimate.
uint8 STATUS_INITIALIZING=0   # first samples after (re)initialisation, see estimator.warmup_samples
uint8 STATUS_VALID=1
uint8 STATUS_DEGRADED=2       # processed, but after a measurement gap or with invalid contact data
uint8 STATUS_INVALID=3        # non-finite or diverged estimate; the estimator is re-initialised
uint8 status

# Base orientation: rotation from base frame to world frame.
geometry_msgs/Quaternion orientation

# Base position in the world frame [m]. Integrated (dead-reckoned) by every
# estimator; position_valid=false if the estimator does not provide it.
geometry_msgs/Point position
bool position_valid

# Base linear velocity [m/s], expressed in the world frame and in the base frame.
geometry_msgs/Vector3 linear_velocity_world
geometry_msgs/Vector3 linear_velocity_base

# Base angular velocity [rad/s] in the base frame. None of the three estimators
# estimates angular velocity as a state: it is the gyroscope measurement of the
# same sample, rotated into base axes, with the estimated gyroscope bias removed
# when the estimator provides one (see angular_velocity_source).
geometry_msgs/Vector3 angular_velocity_base
uint8 ANGULAR_VELOCITY_IMU_RAW=0
uint8 ANGULAR_VELOCITY_IMU_BIAS_CORRECTED=1
uint8 angular_velocity_source

# Estimated IMU biases (in base axes) if the estimator provides them.
geometry_msgs/Vector3 gyro_bias
geometry_msgs/Vector3 accel_bias
bool gyro_bias_valid
bool accel_bias_valid

# Contact state used for this update (order LF, RF, LH, RH).
bool[4] contact

# Timing (all durations measured with a monotonic clock inside this process).
# Time from LowState reception to the end of the estimator update [s].
float64 latency_to_estimate_s
# Estimator update execution time [s] (computation only).
float64 estimator_compute_time_s
# Age of the state at publication: publication ROS time - header.stamp [s].
float64 state_age_at_publish_s
# Number of samples waiting for the estimator when this one was dequeued.
uint32 estimator_queue_depth
# Cumulative number of samples that did not reach the estimator.
uint64 dropped_samples_total
