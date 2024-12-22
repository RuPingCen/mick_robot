#ifndef DATATYPE_H_
#define DATATYPE_H_

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#include <sys/stat.h>
#if defined(_MSC_VER)
#include <io.h>
#endif
#include "typedef.h"

#if !defined(_MSC_VER)
#include <unistd.h>
#define _itoa(value, str, radix) {sprintf(str, "%d", value);}
#endif

#define UNUSED(x) (void)x

#if !defined(_MSC_VER)
#	define _access access
#endif

#define valName(val) (#val)
#define valLastName(val) \
{ \
    char* strToken; \
    char str[64]; \
    strncpy(str, (const char*)val, sizeof(str)); \
    strToken = strtok(str, "."); \
    while (strToken != NULL) { \
        strcpy(val, (const char*)strToken); \
        strToken = strtok(NULL, "."); \
    } \
}


/**
 * @class dataFrame
 * @brief data frame Structure.
 *
 * @author jzhang
 */
#define FRAME_PREAMBLE 0xFFEE
#define LIDAR_2D 0x2
#define DATA_FRAME 0x1
#define DEFAULT_INTENSITY 10
#define DSL(c, i) ((c << i) & (0xFF << i))


/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Type Definition Macros                                                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#ifndef __WORDSIZE
/* Assume 32 */
#define __WORDSIZE 32
#endif

#if defined(__linux__) || defined(_DARWIN)
#include <stdint.h>
typedef int            SOCKET;
#endif


#if defined(_WIN32)
struct iovec {
  void  *iov_base;
  size_t iov_len;
};

typedef int socklen_t;

#endif

#if defined(_WIN32)

#ifndef UINT8_MAX
#define UINT8_MAX  (UCHAR_MAX)
#endif
#ifndef UINT16_MAX
#define UINT16_MAX (USHRT_MAX)
#endif
#ifndef UINT32_MAX
#define UINT32_MAX (ULONG_MAX)
#endif

#if __WORDSIZE == 64
#define SIZE_MAX (18446744073709551615UL)
#else
#ifndef SIZE_MAX
#define SIZE_MAX (4294967295U)
#endif
#endif
#endif

#if defined(_WIN32)
#define ssize_t size_t
#endif

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


#ifdef _AVR_
typedef uint8_t        _size_t;
#define THREAD_PROC
#elif defined (WIN64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef uint32_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (_M_X64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _size_t;
#define THREAD_PROC
#elif defined (__ICCARM__)
typedef uint32_t       _size_t;
#define THREAD_PROC
#endif

typedef int32_t result_t;

#define RESULT_OK      0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL    -2

#define INVALID_TIMESTAMP (0)


#define IS_OK(x)    ( (x) == RESULT_OK )
#define IS_TIMEOUT(x)  ( (x) == RESULT_TIMEOUT )
#define IS_FAIL(x)  ( (x) == RESULT_FAIL )

#endif  // DATATYPE_H_
