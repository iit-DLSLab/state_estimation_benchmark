#pragma once

// Online processing pipeline (ROS independent).
//
//   onLowState() [caller thread, single producer]
//        |  acquisition: validate, de-duplicate, time base, canonical SensorSample
//        |  capacity check (worker queues + synchronizer) -> else kPipelineFull
//        |  sample_id = next id; register in the synchronizer
//        +------------------------------+
//        v                              v
//   kinematics worker thread      contact worker thread
//   (Pinocchio, frame E)          (Go2 foot force, hysteresis)
//        |                              |
//        +------> ExactSampleSynchronizer <------+   (exact sample_id match,
//                        |                           in-order release, timeouts)
//                        v  bounded queue (overflow policy)
//                 estimator worker thread (one estimator)
//                        |
//                 output sink (publication)  -> record sink (logging)

#include "seb_online/acquisition.hpp"
#include "seb_online/contact_detector.hpp"
#include "seb_online/estimator_worker.hpp"
#include "seb_online/robot_mapping.hpp"
#include "seb_online/sample_worker.hpp"
#include "seb_online/synchronizer.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

namespace seb {

struct PipelineConfig {
  RobotMapping mapping;
  AcquisitionConfig acquisition;
  ContactDetectionConfig contact;
  SynchronizerConfig sync;
  std::size_t worker_queue_capacity = 64;
  ThreadSettings kinematics_thread;
  ThreadSettings contact_thread;
  EstimatorWorkerConfig estimator_worker;
  EstimatorSettings estimator;
};

struct PipelineCounters {
  std::uint64_t received = 0;
  std::uint64_t dispatched = 0;
  std::uint64_t processed = 0;
  std::array<std::uint64_t, kNumEventTypes> events{};  // indexed by EventType
  std::uint64_t droppedBeforeEstimator() const;
};

class Pipeline {
 public:
  using KinematicsFn = SampleWorker<KinematicsResult>::Process;
  using ContactFn = SampleWorker<ContactResult>::Process;

  struct Sinks {
    EstimatorWorker::OutputSink output;   // publication
    EstimatorWorker::RecordSink record;   // logging of processed samples
    std::function<void(const PipelineEvent&)> event;   // logging of events
    std::function<void(const std::string&)> status;    // human-readable warnings
  };

  // Test hooks: replace the worker computations and/or filter their results.
  struct Hooks {
    KinematicsFn kinematics;  // default: Pinocchio EstimationFrameKinematics
    ContactFn contact;        // default: Go2FootForceContactDetector
    std::function<bool(const KinematicsResult&)> deliver_kinematics;  // false: result is lost
    std::function<bool(const ContactResult&)> deliver_contact;
  };

  Pipeline(const PipelineConfig& config, std::unique_ptr<EstimatorAdapter> estimator, Sinks sinks,
           Hooks hooks = Hooks());
  ~Pipeline();

  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;

  void start();
  void stop();

  // Entry point of the acquisition (one producer thread).
  void onLowState(const RawLowState& raw);

  // Re-initialises the estimator on the next sample.
  void requestEstimatorReset();

  // Blocks until fewer than `max_in_flight` dispatched samples are not yet
  // consumed by the estimator (lossless replay). Returns false on timeout.
  bool waitForInFlightBelow(std::uint64_t max_in_flight, std::chrono::milliseconds timeout);
  // Blocks until every dispatched sample was consumed or expired.
  bool drain(std::chrono::milliseconds timeout);

  PipelineCounters counters() const;
  SynchronizerStats syncStats() const { return sync_->stats(); }
  std::size_t kinematicsQueued() const { return kin_worker_->queued(); }
  std::size_t contactQueued() const { return contact_worker_->queued(); }
  const PipelineConfig& config() const { return config_; }
  const EstimatorAdapter& estimator() const { return estimator_worker_->adapter(); }

 private:
  void recordEvent(const PipelineEvent& e);
  void resetDownstream();

  PipelineConfig config_;
  Sinks sinks_;
  Hooks hooks_;

  LowStateAcquisition acquisition_;
  std::unique_ptr<ExactSampleSynchronizer> sync_;
  std::unique_ptr<SampleWorker<KinematicsResult>> kin_worker_;
  std::unique_ptr<SampleWorker<ContactResult>> contact_worker_;
  std::unique_ptr<EstimatorWorker> estimator_worker_;

  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> dispatched_{0};
  std::array<std::atomic<std::uint64_t>, kNumEventTypes> event_counts_{};
  std::atomic<std::uint64_t> expired_or_late_{0};
  bool started_ = false;
};

}  // namespace seb
