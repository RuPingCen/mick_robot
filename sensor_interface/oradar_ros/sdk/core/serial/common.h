#ifndef COMMON_H_
#define COMMON_H_

#if defined(_WIN32)
#include "impl\windows\win.h"
#include "impl\windows\win_serial.h"
#elif defined(__GNUC__)
#include "impl/unix/unix.h"
#include "impl/unix/unix_serial.h"
#else
#error "unsupported target"
#endif
#include <core/base/thread.h>
#include <core/base/locker.h>
#include <core/base/timer.h>

#endif

