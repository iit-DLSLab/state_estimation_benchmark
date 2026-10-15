#pragma once

// The offline IEKF and invariant smoother are global objects in their mains,
// i.e. zero-initialised static storage, and some members are read before they
// are first written (e.g. IEKF IMU_Gyro[1] in send_states at the first step).
// To reproduce that initial state the online adapters allocate them in zeroed
// anonymous memory (mmap: zero pages are provided lazily, so the large unused
// offline buffers of these classes, ~190 MB each, are not made resident).

#include <sys/mman.h>
#include <unistd.h>

#include <cstddef>
#include <memory>
#include <new>

namespace seb {

template <typename T>
struct ZeroedDeleter {
  void operator()(T* p) const {
    if (p == nullptr) return;
    p->~T();
    ::munmap(static_cast<void*>(p), bytes());
  }
  static std::size_t bytes() {
    const std::size_t page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
    return ((sizeof(T) + page - 1) / page) * page;
  }
};

template <typename T>
using ZeroedPtr = std::unique_ptr<T, ZeroedDeleter<T>>;

template <typename T>
ZeroedPtr<T> makeZeroed() {
  void* mem = ::mmap(nullptr, ZeroedDeleter<T>::bytes(), PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (mem == MAP_FAILED) throw std::bad_alloc();
  try {
    return ZeroedPtr<T>(new (mem) T());
  } catch (...) {
    ::munmap(mem, ZeroedDeleter<T>::bytes());
    throw;
  }
}

}  // namespace seb
