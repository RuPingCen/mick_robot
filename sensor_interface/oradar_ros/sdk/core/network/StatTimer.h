/*----------------------------------------------------------------------------*/
/*                                                                            */
/* StatTimer.h: interface for the CStatTimer class.                           */
/*                                                                            */
/* Author: Mark Carrier (mark@carrierlabs.com)                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2006 CarrierLabs, LLC.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. The name "CarrierLabs" must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission. For written permission, please contact
 *    mark@carrierlabs.com.
 *
 * THIS SOFTWARE IS PROVIDED BY MARK CARRIER ``AS IS'' AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL MARK CARRIER OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------*/
#ifndef __CSTATTIMER_H__
#define __CSTATTIMER_H__

#include <string.h>

#if defined(_WIN32)
#include <Winsock2.h>
#include <time.h>
#else
#include <stdio.h>
#include <sys/time.h>
#endif


#if defined(_WIN32)
#if !defined (_WINSOCK2API_) && !defined(_WINSOCKAPI_)
struct timeval {
  long    tv_sec;         /* seconds */
  long    tv_usec;        /* and microseconds */
};
#endif

inline static int gettimeofday(struct timeval *tv, void *tz) {
  union {
    long long ns100;
    FILETIME    t;
  } now;
  GetSystemTimeAsFileTime(&now.t);
  tv->tv_usec = long((now.ns100 / 10LL) % 1000000LL);
  tv->tv_sec = long((now.ns100 - 116444736000000000LL) / 10000000LL);
  return 0;
}
#undef HAS_CLOCK_GETTIME

#endif


#define GET_CLOCK_COUNT(x)  gettimeofday(x, NULL)

#include <core/base/v8stdint.h>


/// Class to abstract socket communications in a cross platform manner.
/// This class is designed
class CStatTimer {
 public:
  CStatTimer() {
    memset(&m_startTime, 0, sizeof(struct timeval));
    memset(&m_endTime, 0, sizeof(struct timeval));
  };

  ~CStatTimer() {
  };

  void Initialize() {
    memset(&m_startTime, 0, sizeof(struct timeval));
    memset(&m_endTime, 0, sizeof(struct timeval));
  };

  struct timeval GetStartTime() {
    return m_startTime;
  };
  void SetStartTime() {
    GET_CLOCK_COUNT(&m_startTime);
  };

  struct timeval GetEndTime() {
    return m_endTime;
  };
  void SetEndTime() {
    GET_CLOCK_COUNT(&m_endTime);
  };

  uint32_t GetMilliSeconds() {
    return (CalcTotalUSec() / MILLISECONDS_CONVERSION);
  };
  uint64_t GetMicroSeconds() {
    return (CalcTotalUSec());
  };
  uint32_t GetSeconds() {
    return (CalcTotalUSec() / MICROSECONDS_CONVERSION);
  };

  static uint64_t GetCurrentTime() {
#if HAS_CLOCK_GETTIME
    struct timespec  tim;
    clock_gettime(CLOCK_REALTIME, &tim);
    return (uint64_t)(tim.tv_sec * 1000000000LL + tim.tv_nsec);
#else
    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    return (uint64_t)(timeofday.tv_sec * 1000000000LL + timeofday.tv_usec * 1000);
#endif
  };

 private:
  uint32_t CalcTotalUSec() {
    return (((m_endTime.tv_sec - m_startTime.tv_sec) * MICROSECONDS_CONVERSION) +
            (m_endTime.tv_usec - m_startTime.tv_usec));
  };


 private:
  struct timeval  m_startTime;
  struct timeval  m_endTime;
};

#endif // __CSTATTIMER_H__
