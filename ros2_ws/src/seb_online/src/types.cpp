#include "seb_online/types.hpp"

namespace seb {

const char* toString(EventType type) {
  switch (type) {
    case EventType::kInvalidMessage: return "invalid_message";
    case EventType::kDuplicate: return "duplicate";
    case EventType::kOutOfOrder: return "out_of_order";
    case EventType::kDecimated: return "decimated";
    case EventType::kPipelineFull: return "pipeline_full";
    case EventType::kMissingKinematics: return "missing_kinematics";
    case EventType::kMissingContact: return "missing_contact";
    case EventType::kMissingBoth: return "missing_kinematics_and_contact";
    case EventType::kLateResult: return "late_worker_result";
    case EventType::kEstimatorBacklog: return "estimator_backlog_drop";
    case EventType::kStaleInput: return "stale_input";
    case EventType::kGap: return "gap";
    case EventType::kResetGap: return "reset_gap";
    case EventType::kResetDiverged: return "reset_diverged";
    case EventType::kResetRequested: return "reset_requested";
    case EventType::kResetTimeJump: return "reset_time_jump";
    case EventType::kDtTooSmall: return "dt_too_small";
    case EventType::kEstimatorException: return "estimator_exception";
  }
  return "unknown";
}

}  // namespace seb
