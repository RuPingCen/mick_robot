// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.


#ifndef ORADAR_PROTOCOL_H_
#define ORADAR_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

#define TMPBUFF_SIZE (1024)
#define MAX_BLOCK_POINT_NUM (100)
#define MAX_BLOCK_SIZE (MAX_BLOCK_POINT_NUM * 4)
#define POINT_CIRCLE_MAX_SIZE (4096)
#define POINT_PKG_MAX_SIZE (200)
#define POINT_PER_PACK (12)

#define SET_TIME_OUT    (10) //unit:s
#define HEAD_FLAG   (0xF5A5)        
#define TAIL_FLAG   (0x31F2)
#define HEAD_LEN    (5)

typedef enum
{
    ORADAR_MS200 = 1,
    ORADAR_MS300 = 2,
}oradar_lidar_type_id;

typedef enum {
    ORADAR_TYPE_SERIAL = 0x0,/**< serial type.*/
    ORADAR_TYPC_UDP = 0x1,/**< socket udp type.*/
    ORADAR_TYPE_TCP = 0x1,/**< socket tcp type.*/
} device_type_id;

typedef enum
{
    SET_ROTATION_SPEED = 0xA1,
    SET_RUN_MODE = 0xA2,
}CMD;

typedef enum
{
    WRITE_PARAM = 0xC1,
    WRITE_PARAM_RESPONSE = 0xC2,
    READ_PARAM = 0xC3,
    READ_PARAM_RESPONSE = 0xC4,
}CMD_TYPE;

typedef struct uart_comm_st
{
    uint16_t head_flag;
    uint8_t cmd;
    uint8_t cmd_type;
    uint8_t payload_len;
    uint8_t data[10];
}uart_comm_t;

typedef struct point_data_st
{
    unsigned short distance;
    unsigned short intensity;
    float angle;
} point_data_t;

typedef enum frame_head_flag_et
{
    HEAD_FLAG_NONE,
    HEAD_FLAG_OK,
} frame_head_flag_t;

typedef enum protocol_version_et
{
    VERSION_NONE = 0,
    VERSION_MS200,
} protocol_version_t;

typedef struct __attribute__((packed))
{
    uint16_t distance;
    uint8_t confidence;
} OradarLidarPoint;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    OradarLidarPoint point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} OradarLidarFrame;

typedef struct
{
    frame_head_flag_t head_flag;
    protocol_version_t version;
    double speed;
    unsigned char cmd_mode;
    unsigned char point_num;
    float start_angle;
    float end_angle;
    uint16_t timestamp;
    unsigned short crc_value;
} parsed_data_st;

typedef struct
{
    point_data_t data[POINT_CIRCLE_MAX_SIZE + 1];
    unsigned short vailtidy_point_num;
    double speed;
} full_scan_data_st;

typedef struct
{
    point_data_t data[POINT_PKG_MAX_SIZE + 1];
    unsigned short vailtidy_point_num;
    double speed;
} one_scan_data_st;

#ifdef __cplusplus
}
#endif


#endif  // ORADAR_PROTOCOL_H_
