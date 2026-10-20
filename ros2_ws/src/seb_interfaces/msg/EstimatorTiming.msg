# Timing of ONE estimator update of seb_online (profiling.publish_timing), published
# right after the state. All durations come from the monotonic clock of the
# estimator process (std::chrono::steady_clock); NaN = not measured for this sample.
#
#   header.stamp            ROS time at which the state was published (use it for
#                           the output rate; it is not the time of the state)
#   sample_id               id of the source LowState sample, = EstimatorState.sample_id
#                           and BaseState.sequence_id (mod 2^32)
#   lowstate_tick           LowState.tick of the source sample: a robot-side counter,
#                           an identifier only, never a time
#
#   estimator_ms            duration of the estimator update (adapter step(), which
#                           calls the unmodified MUSE / IEKF / smoother update)
#   kinematics_ms           preprocessing: forward kinematics of the sample (worker thread)
#   contact_ms              preprocessing: contact detection of the sample (worker thread);
#                           kinematics and contact run in parallel
#   receive_to_publish_ms   entry of the LowState subscription callback -> start of
#                           the state publication (includes queueing and synchronisation)

std_msgs/Header header
string estimator
uint64 sample_id
uint32 lowstate_tick
uint8 status               # EstimatorState.STATUS_*

float64 estimator_ms
float64 kinematics_ms
float64 contact_ms
float64 receive_to_publish_ms
