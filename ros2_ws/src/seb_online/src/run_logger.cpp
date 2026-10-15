#include "seb_online/run_logger.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace seb {

namespace {

constexpr const char* kSampleHeader =
    "sample_id,acq_seq,msg_seq,tick,estimator,status,stamp_source,"
    "stamp_ns,receive_ros_ns,publish_ros_ns,dds_source_ns,"
    "receive_steady_ns,dispatch_steady_ns,kin_start_steady_ns,kin_end_steady_ns,"
    "contact_start_steady_ns,contact_end_steady_ns,complete_steady_ns,release_steady_ns,"
    "dequeue_steady_ns,est_start_steady_ns,est_end_steady_ns,publish_steady_ns,"
    // derived durations [s] (steady clock unless stated)
    "kinematics_exec_s,contact_exec_s,sync_wait_s,estimator_queue_wait_s,estimator_exec_s,"
    "latency_to_estimate_s,e2e_latency_s,state_age_at_publish_s,"
    // occupancy and losses
    "kin_queue_at_dispatch,contact_queue_at_dispatch,sync_pending_at_dispatch,estimator_queue_depth,"
    "missing_before,dropped_total,"
    // estimator diagnostics
    "dt_s,iterations,window_frames,window_duration_s,"
    // state (world W, base B)
    "px,py,pz,qw,qx,qy,qz,vx_w,vy_w,vz_w,vx_b,vy_b,vz_b,wx_b,wy_b,wz_b,omega_bias_corrected,"
    "bgx,bgy,bgz,bax,bay,baz,position_valid,contact_LF,contact_RF,contact_LH,contact_RH\n";

constexpr const char* kEventHeader = "steady_ns,event,msg_seq,sample_id,stamp_ns,tick,value\n";

double sec(std::int64_t ns) { return static_cast<double>(ns) * 1e-9; }

}  // namespace

RunLogger::RunLogger(const std::string& directory, std::size_t queue_capacity)
    : directory_(directory), queue_(queue_capacity) {
  std::filesystem::create_directories(directory_);
  samples_.open(directory_ + "/samples.csv");
  events_.open(directory_ + "/events.csv");
  if (!samples_.is_open() || !events_.is_open()) {
    throw std::runtime_error("RunLogger: cannot open log files in " + directory_);
  }
  samples_ << kSampleHeader;
  events_ << kEventHeader;
  samples_ << std::setprecision(10);
  events_ << std::setprecision(10);
}

RunLogger::~RunLogger() { stop(); }

void RunLogger::start() {
  if (thread_.joinable()) return;
  queue_.reopen();
  thread_ = std::thread([this] { run(); });
}

void RunLogger::stop() {
  queue_.close();
  if (thread_.joinable()) thread_.join();
  samples_.flush();
  events_.flush();
}

void RunLogger::logSample(const StateEstimate& s) {
  if (!queue_.tryPush(Record(s))) dropped_records_.fetch_add(1);
}

void RunLogger::logEvent(const PipelineEvent& e) {
  if (!queue_.tryPush(Record(e))) dropped_records_.fetch_add(1);
}

void RunLogger::writeTextFile(const std::string& name, const std::string& content) {
  std::ofstream f(directory_ + "/" + name);
  f << content;
}

void RunLogger::run() {
  while (auto r = queue_.pop()) {
    if (std::holds_alternative<StateEstimate>(*r)) {
      writeSample(std::get<StateEstimate>(*r));
    } else {
      writeEvent(std::get<PipelineEvent>(*r));
    }
  }
}

void RunLogger::writeSample(const StateEstimate& s) {
  const std::int64_t first_result = std::min(s.kin_end_steady_ns, s.contact_end_steady_ns);
  auto& o = samples_;
  o << s.sample_id << ',' << s.acq_seq << ',' << s.msg_seq << ',' << s.tick << ',' << s.estimator << ','
    << toString(s.status) << ',' << (s.stamp_source == StampSource::kReceiveTime ? "receive_time" : "lowstate_tick")
    << ',' << s.stamp_ns << ',' << s.receive_ros_ns << ',' << s.publish_ros_ns << ',' << s.dds_source_ns << ','
    << s.receive_steady_ns << ','
    << s.dispatch_steady_ns << ',' << s.kin_start_steady_ns << ',' << s.kin_end_steady_ns << ','
    << s.contact_start_steady_ns << ',' << s.contact_end_steady_ns << ',' << s.complete_steady_ns << ','
    << s.release_steady_ns << ',' << s.dequeue_steady_ns << ',' << s.est_start_steady_ns << ','
    << s.est_end_steady_ns << ',' << s.publish_steady_ns << ',';
  o << sec(s.kin_end_steady_ns - s.kin_start_steady_ns) << ','
    << sec(s.contact_end_steady_ns - s.contact_start_steady_ns) << ',' << sec(s.release_steady_ns - first_result)
    << ',' << sec(s.dequeue_steady_ns - s.release_steady_ns) << ','
    << sec(s.est_end_steady_ns - s.est_start_steady_ns) << ',' << sec(s.est_end_steady_ns - s.receive_steady_ns)
    << ',' << (s.publish_steady_ns > 0 ? sec(s.publish_steady_ns - s.receive_steady_ns) : 0.0) << ','
    // ROS-clock difference: both values are in the ROS time domain (see docs).
    << (s.publish_ros_ns > 0 ? sec(s.publish_ros_ns - s.stamp_ns) : 0.0) << ',';
  o << s.kin_queue_at_dispatch << ',' << s.contact_queue_at_dispatch << ',' << s.sync_pending_at_dispatch << ','
    << s.estimator_queue_depth << ',' << s.missing_before << ',' << s.dropped_total << ',';
  o << s.dt_s << ',' << s.iterations << ',' << s.window_frames << ',' << s.window_duration_s << ',';
  o << s.p_wb.x() << ',' << s.p_wb.y() << ',' << s.p_wb.z() << ',' << s.q_wb.w() << ',' << s.q_wb.x() << ','
    << s.q_wb.y() << ',' << s.q_wb.z() << ',' << s.v_wb.x() << ',' << s.v_wb.y() << ',' << s.v_wb.z() << ','
    << s.v_b.x() << ',' << s.v_b.y() << ',' << s.v_b.z() << ',' << s.omega_b.x() << ',' << s.omega_b.y() << ','
    << s.omega_b.z() << ',' << s.omega_bias_corrected << ',' << s.gyro_bias.x() << ',' << s.gyro_bias.y() << ','
    << s.gyro_bias.z() << ',' << s.acc_bias.x() << ',' << s.acc_bias.y() << ',' << s.acc_bias.z() << ','
    << s.position_valid << ',' << s.contact[0] << ',' << s.contact[1] << ',' << s.contact[2] << ','
    << s.contact[3] << '\n';
}

void RunLogger::writeEvent(const PipelineEvent& e) {
  events_ << e.steady_ns << ',' << toString(e.type) << ',' << e.msg_seq << ','
          << (e.has_sample_id ? std::to_string(e.sample_id) : std::string()) << ',' << e.stamp_ns << ',' << e.tick
          << ',' << e.value << '\n';
}

}  // namespace seb
