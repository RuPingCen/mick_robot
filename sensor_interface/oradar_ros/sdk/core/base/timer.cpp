#include "timer.h"
#if defined(_WIN32)
#include <mmsystem.h>
#pragma comment(lib, "Winmm.lib")

namespace impl {

static LARGE_INTEGER _current_freq;

void HPtimer_reset() {
  BOOL ans = QueryPerformanceFrequency(&_current_freq);
  _current_freq.QuadPart /= 1000;
}

uint32_t getHDTimer() {
  LARGE_INTEGER current;
  QueryPerformanceCounter(&current);

  return (uint32_t)(current.QuadPart / (_current_freq.QuadPart));
}

uint64_t getCurrentTime() {
  FILETIME		t;
  GetSystemTimeAsFileTime(&t);
  return ((((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime)) *
         100;
}


BEGIN_STATIC_CODE(timer_cailb) {
  HPtimer_reset();
} END_STATIC_CODE(timer_cailb)

}
#else

namespace impl {
uint32_t getHDTimer() {
  struct timespec t;
  t.tv_sec = t.tv_nsec = 0;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000L + t.tv_nsec / 1000000L;
}
uint64_t getCurrentTime() {
#if HAS_CLOCK_GETTIME
  struct timespec  tim;
  clock_gettime(CLOCK_REALTIME, &tim);
  return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  return static_cast<uint64_t>(timeofday.tv_sec) * 1000000000LL +
         static_cast<uint64_t>(timeofday.tv_usec) * 1000LL;
#endif
}
}
#endif
