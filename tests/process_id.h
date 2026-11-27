#pragma once

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace obf2test {

inline int processId() {
#if defined(_WIN32)
  return ::_getpid();
#else
  return ::getpid();
#endif
}

}  // namespace obf2test
