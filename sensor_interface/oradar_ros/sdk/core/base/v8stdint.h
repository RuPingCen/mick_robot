#ifndef V8STDINT_H_
#define V8STDINT_H_
#include "datatype.h"
typedef _size_t (THREAD_PROC *thread_proc_t)(void *);

//Socket

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef htonll
#ifdef _BIG_ENDIAN
#define htonll(x)   (x)
#define ntohll(x)   (x)
#else
#define htonll(x)   ((((uint64)htonl(x)) << 32) + htonl(x >> 32))
#define ntohll(x)   ((((uint64)ntohl(x)) << 32) + ntohl(x >> 32))
#endif
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Socket Macros                                                             */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#if defined(_WIN32)
#define SHUT_RD                0
#define SHUT_WR                1
#define SHUT_RDWR              2
#define ACCEPT(a,b,c)          accept(a,b,c)
#define CONNECT(a,b,c)         connect(a,b,c)
#define CLOSE(a)               closesocket(a)
#define READ(a,b,c)            read(a,b,c)
#define RECV(a,b,c,d)          recv(a, (char *)b, c, d)
#define RECVFROM(a,b,c,d,e,f)  recvfrom(a, (char *)b, c, d, (sockaddr *)e, (int *)f)
#define RECV_FLAGS             MSG_WAITALL
#define SELECT(a,b,c,d,e)      select((int32_t)a,b,c,d,e)
#define SEND(a,b,c,d)          send(a, (const char *)b, (int)c, d)
#define SENDTO(a,b,c,d,e,f)    sendto(a, (const char *)b, (int)c, d, e, f)
#define SEND_FLAGS             0
#define SENDFILE(a,b,c,d)      sendfile(a, b, c, d)
#define SET_SOCKET_ERROR(x,y)  errno=y
#define SOCKET_ERROR_INTERUPT  EINTR
#define SOCKET_ERROR_TIMEDOUT  EAGAIN
#define WRITE(a,b,c)           write(a,b,c)
#define WRITEV(a,b,c)          Writev(b, c)
#define GETSOCKOPT(a,b,c,d,e)  getsockopt(a,b,c,(char *)d, (int *)e)
#define SETSOCKOPT(a,b,c,d,e)  setsockopt(a,b,c,(char *)d, (int)e)
#define GETHOSTBYNAME(a)       gethostbyname(a)
#define IOCTLSOCKET(a, b, c)   ioctlsocket(a,b,(u_long*)c)
#endif

#if defined(__linux__) || defined(_DARWIN)
#define ACCEPT(a,b,c)          accept(a,b,c)
#define CONNECT(a,b,c)         connect(a,b,c)
#define CLOSE(a)               close(a)
#define READ(a,b,c)            read(a,b,c)
#define RECV(a,b,c,d)          recv(a, (void *)b, c, d)
#define RECVFROM(a,b,c,d,e,f)  recvfrom(a, (char *)b, c, d, (sockaddr *)e, f)
#define RECV_FLAGS             MSG_WAITALL
#define SELECT(a,b,c,d,e)      select(a,b,c,d,e)
#define SEND(a,b,c,d)          send(a, (const int8_t *)b, c, d)
#define SENDTO(a,b,c,d,e,f)    sendto(a, (const int8_t *)b, c, d, e, f)
#define SEND_FLAGS             0
#define SENDFILE(a,b,c,d)      sendfile(a, b, c, d)
#define SET_SOCKET_ERROR(x,y)  errno=y
#define SOCKET_ERROR_INTERUPT  EINTR
#define SOCKET_ERROR_TIMEDOUT  EAGAIN
#define WRITE(a,b,c)           write(a,b,c)
#define WRITEV(a,b,c)          writev(a, b, c)
#define GETSOCKOPT(a,b,c,d,e)  getsockopt((int)a,(int)b,(int)c,(void *)d,(socklen_t *)e)
#define SETSOCKOPT(a,b,c,d,e)  setsockopt((int)a,(int)b,(int)c,(const void *)d,(int)e)
#define GETHOSTBYNAME(a)       gethostbyname(a)
#define IOCTLSOCKET(a, b, c)   ioctl(a,b,c)
#endif


/*---------------------------------------------------------------------------*/
/*                                                                           */
/* File Macros                                                               */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#define STRUCT_STAT         struct stat
#define LSTAT(x,y)          lstat(x,y)
#define FILE_HANDLE         FILE *
#define CLEARERR(x)         clearerr(x)
#define FCLOSE(x)           fclose(x)
#define FEOF(x)             feof(x)
#define FERROR(x)           ferror(x)
#define FFLUSH(x)           fflush(x)
#define FILENO(s)           fileno(s)
#define FOPEN(x,y)          fopen(x, y)
//#define FREAD(a,b,c,d)      fread(a, b, c, d)
#define FSTAT(s, st)        fstat(FILENO(s), st)
//#define FWRITE(a,b,c,d)     fwrite(a, b, c, d)
#define STAT_BLK_SIZE(x)    ((x).st_blksize)

#define DEFAULT_CONNECTION_TIMEOUT_SEC 2
#define DEFAULT_CONNECTION_TIMEOUT_USEC 800000

#define DEFAULT_REV_TIMEOUT_SEC 2
#define DEFAULT_REV_TIMEOUT_USEC 800000




/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Misc Macros                                                               */
/*                                                                           */
/*---------------------------------------------------------------------------*/

#if defined(_WIN32)
#define STRTOULL(x) _atoi64(x)
#else
#define STRTOULL(x) strtoull(x, NULL, 10)
#endif

#if defined(_WIN32)
#define SNPRINTF _snprintf
#define PRINTF   printf
#define VPRINTF  vprintf
#define FPRINTF  fprintf
#else
#define SNPRINTF snprintf
#define PRINTF   printf
#define VPRINTF  vprintf
#define FPRINTF  fprintf
#endif



#define MILLISECONDS_CONVERSION 1000
#define MICROSECONDS_CONVERSION 1000000
#define NANOECONDS_CONVERSION 1000000000

//#include "ordlidar.h"

#endif  // V8STDINT_H_
