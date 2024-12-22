// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef ORD_LIDAR_C_API_DRIVER_H
#define ORD_LIDAR_C_API_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "ordlidar_protocol.h"

  /// lidar instance
  typedef struct
  {
    void *lidar;
  } ORDLidar;


  ORDLidar *oradar_lidar_create(uint8_t type, int model);
  void oradar_lidar_destroy(ORDLidar **lidar);
  bool oradar_set_serial_port(ORDLidar *lidar, char *port, int baudrate);
  bool oradar_connect(ORDLidar *lidar);
  bool oradar_disconnect(ORDLidar *lidar);
  bool oradar_get_timestamp(ORDLidar *lidar, uint16_t *timestamp);
  bool oradar_get_rotation_speed(ORDLidar *lidarm, double *rotation_speed);
  bool oradar_get_firmware_version(ORDLidar *lidar, char *top_fw_version, char *bot_fw_version);
  bool oradar_get_device_sn(ORDLidar *lidar, char *device_sn);
  bool oradar_set_rotation_speed(ORDLidar *lidar, uint16_t speed);
  bool oradar_activate(ORDLidar *lidar);
  bool oradar_deactive(ORDLidar *lidar);

  bool oradar_get_grabonescan_blocking(ORDLidar *lidar, one_scan_data_st *data, int timeout_ms);
  bool oradar_get_grabfullscan_blocking(ORDLidar *lidar, full_scan_data_st *data, int timeout_ms);
  bool oradar_get_grabonescan(ORDLidar *lidar, one_scan_data_st *data);
  bool oradar_get_grabfullscan(ORDLidar *lidar, full_scan_data_st *data);

#ifdef __cplusplus
}
#endif

#endif
