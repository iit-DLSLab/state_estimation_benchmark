#include "seb_online/thread_utils.hpp"

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>

namespace seb {

std::string applyThreadSettings(const std::string& name, const ThreadSettings& settings) {
  std::string errors;
  pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());

  if (settings.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(settings.cpu, &set);
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc != 0) errors += "affinity cpu=" + std::to_string(settings.cpu) + ": " + std::strerror(rc) + "; ";
  }
  if (settings.rt_priority > 0) {
    sched_param param{};
    param.sched_priority = settings.rt_priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0) errors += "SCHED_FIFO priority=" + std::to_string(settings.rt_priority) + ": " + std::strerror(rc) + "; ";
  }
  return errors;
}

}  // namespace seb
