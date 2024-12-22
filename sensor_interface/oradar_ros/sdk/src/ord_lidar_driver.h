// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef ORD_LIDAR_DRIVER_H
#define ORD_LIDAR_DRIVER_H

#include <thread>

#include <stdlib.h>
#include <atomic>
#include <map>
#include <vector>
#include <core/common/ChannelDevice.h>
#include <core/base/locker.h>
#include <core/base/thread.h>
#include <core/base/timer.h>
#include "ordlidar_protocol.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The Oradar LIDAR SDK requires a C++ compiler to be built"
#endif
#endif

using namespace std;
//using namespace ordlidar;
namespace ordlidar
{
	using namespace core;
	using namespace base;
	using namespace core::common;
	class OrdlidarDriver
	{
	public:
		OrdlidarDriver(uint8_t type, int model);
		~OrdlidarDriver();

		// 设置串口属性
		bool SetSerialPort(const std::string &port_name, const uint32_t &baudrate);
		// 检查激光雷达串口并打开，创建激光雷达串口读写线程
		bool Connect();
		// 关闭激光雷达串口读写线程， 关闭串口
		void Disconnect();
		// 判断激光雷达串口是否打开
		bool isConnected() const;
		// 激光雷达从待机状态进入测距状态
		bool Activate();
		// 激光雷达从测距状态进入待机状态
		bool Deactive();
		// 获取最新一包点云数据，非阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabOneScan(one_scan_data_st &scan_data);
		// 获取最新一包点云数据，阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms);
		// 获取最新一圈点云数据，非阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabFullScan(full_scan_data_st &scan_data);
 		// 获取最新一圈点云数据，阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms);
		// 获取最新包的时间戳, 单位为毫秒
		uint16_t GetTimestamp() const;
		// 获取最新包的电机转速, 单位为Hz
		double GetRotationSpeed() const;
		// 设置电机转速
		bool SetRotationSpeed(int speed);
		// 获取上下部组固件版本号
		bool GetFirmwareVersion(std::string &top_fw_version, std::string &bot_fw_version);
		// 获取雷达设备SN
		bool GetDeviceSN(std::string &device_sn);

	private:
		static void mRxThreadProc(void *arg);
		int read(unsigned char *data, int length);
		int write(unsigned char *data, int length);
		bool uart_data_handle(unsigned char *data, int len);
		bool uart_data_find_init_info(unsigned char *data, int len);
		bool IsFullScanReady(void) { return full_scan_ready_; }
		void ResetFullScanReady(void) { full_scan_ready_ = false; }
		bool IsOneScanReady(void) { return one_scan_ready_; }
		void ResetOneScanReady(void) { one_scan_ready_ = false; }
		//int point_data_parse_frame_ms200(point_data_t *data, unsigned char *buf, unsigned short buf_len, float start_angle, float end_angle);
		int point_data_parse_frame_ms200(point_data_t *data, OradarLidarFrame *pkg);

	private:
		// serial port
		ChannelDevice *serial_;
		std::string port_name_;
		std::string top_fw_version_;
		std::string bot_fw_version_;
		std::string device_sn_;
		uint32_t baudrate_;
		//tranformer type
  		uint8_t tranformer_type_;
  		int model_;
		bool is_connected_;
		bool full_scan_ready_;
		bool one_scan_ready_;
		int valid_data_;
		uint8_t init_info_flag_;
		std::vector<uint8_t> bin_buf_;
		std::vector<uint8_t> cmd_buf_;
		parsed_data_st parsed_data_;
		full_scan_data_st full_scan_data_;
		full_scan_data_st temp_data;
		one_scan_data_st one_scan_data_;
		std::thread *rx_thread_;
		std::atomic<bool> rx_thread_exit_flag_;

		Event full_data_event_;
		Event one_data_event_;
		//Locker lock_;
	};

}

#endif
