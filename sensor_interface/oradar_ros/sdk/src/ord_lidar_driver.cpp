#include <math.h>
#include "ord_lidar_driver.h"
#include <signal.h>
#include <core/base/utils.h>
#include <core/serial/serial.h>
#include <core/network/ActiveSocket.h>
#include <core/serial/common.h>

namespace ordlidar
{
#define DEFAULT_TIMEOUT (2000)

	static const uint8_t CrcTable[256] =
		{
			0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
			0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
			0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
			0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
			0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
			0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
			0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
			0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
			0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
			0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
			0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
			0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
			0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
			0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
			0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
			0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
			0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
			0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
			0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
			0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
			0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
			0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};
	OrdlidarDriver::OrdlidarDriver(uint8_t type, int model)
	{
		serial_ = NULL;
		rx_thread_ = NULL;
		baudrate_ = 0;
		is_connected_ = false;
		tranformer_type_ = type;
		model_ = model;
		valid_data_ = 0;
		init_info_flag_ = false;
		memset(&parsed_data_, 0, sizeof(parsed_data_st));
		memset(&temp_data, 0, sizeof(full_scan_data_st));
		memset(&full_scan_data_, 0, sizeof(full_scan_data_st));
		memset(&one_scan_data_, 0, sizeof(one_scan_data_st));

	}

	OrdlidarDriver::~OrdlidarDriver()
	{
		Disconnect();
	}

	bool OrdlidarDriver::SetSerialPort(const std::string &port_name, const uint32_t &baudrate)
	{
		port_name_ = port_name;
		baudrate_ = baudrate;
		return true;
	}

	bool OrdlidarDriver::Connect()
	{
		if (isConnected())
		{
			Disconnect();
		}

		if (!serial_)
		{
			if(tranformer_type_ == ORADAR_TYPE_SERIAL)
			{
				serial_ = new serial::Serial(port_name_, baudrate_,
											 serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
			}
			// serial_->bindport(port_name_, baudrate_);
		}

		{
			// ScopedLocker l(lock_);

			if (!serial_->open())
			{
				return is_connected_;
			}

			is_connected_ = true;
			rx_thread_exit_flag_ = false;
			rx_thread_ = new std::thread(mRxThreadProc, this);
			delay(200);	//等待收到数据，才可提前知道一些状态
		}

		return is_connected_;
	}

	bool OrdlidarDriver::isConnected() const
	{
		return is_connected_;
	}

	bool OrdlidarDriver::uart_data_find_init_info(unsigned char *data, int len)
	{
		cmd_buf_.insert(cmd_buf_.end(), data, data + len);
		int cmd_buf_len = cmd_buf_.size();
		int tmp_buf_len = 0;
		uint8_t cur_cmd = 0;
		//printf("cmd_buf_len:%d\n", cmd_buf_len);
		int pos = 0, data_len = 0;
		static uint8_t head_flag = 0;
		while (pos < (cmd_buf_len - 3))
		{
			if (head_flag == 0)
			{
				if ((cmd_buf_[pos] == 0xAA) && (cmd_buf_[pos + 1] == 0x55))
				{
					head_flag = 1;
					break;
				}
			}
			else
			{
				break;
			}
			pos++;
		}

		if (pos != 0)
		{
			cmd_buf_.erase(cmd_buf_.begin(), cmd_buf_.begin() + pos);
		}

		tmp_buf_len = cmd_buf_.size();
		if(head_flag && (tmp_buf_len > 3))	
		{
			cur_cmd = cmd_buf_[2];
			data_len = cmd_buf_[3];
			if(cur_cmd == 0x01)
			{
				if(tmp_buf_len >=(data_len +7))	//7= 2byte head + 1byte flag + 1byte len + 1byte crc + 2byte tail
				{
					if(cmd_buf_[5+data_len] == 0x31 && cmd_buf_[6+data_len] == 0xF2)
					{
						device_sn_.assign((char*)&cmd_buf_[4], data_len);
						//printf("sn:%s\n", device_sn_.c_str());
					}
					else
					{
						printf("find device sn fail, tmp_buf_len:%d\n", tmp_buf_len);
					}
					cmd_buf_.erase(cmd_buf_.begin(), cmd_buf_.begin() + data_len+7);
					head_flag = 0;
				}
			}
			else if(cur_cmd == 0x02)
			{
				if(tmp_buf_len >=(data_len +7))	//7= 2byte head + 1byte flag + 1byte len + 1byte crc + 2byte tail
				{
					if(cmd_buf_[5+data_len] == 0x31 && cmd_buf_[6+data_len] == 0xF2)
					{
						int tmp_len = 5 + cmd_buf_[4];
						int fw_len = cmd_buf_[tmp_len];
						bot_fw_version_.assign((char*)&cmd_buf_[tmp_len+1], fw_len);
						//printf("bot_fw_version_:%s\n", bot_fw_version_.c_str());
					}
					else
					{
						printf("find tail fail, tmp_buf_len:%d\n", tmp_buf_len);
					}
					cmd_buf_.erase(cmd_buf_.begin(), cmd_buf_.begin() + data_len+7);
					head_flag = 0;
				}
			}
			else if(cur_cmd == 0x03)
			{
				if(tmp_buf_len >=(data_len +7))	//7= 2byte head + 1byte flag + 1byte len + 1byte crc + 2byte tail
				{
					if(cmd_buf_[5+data_len] == 0x31 && cmd_buf_[6+data_len] == 0xF2)
					{
						int tmp_len = 5 + cmd_buf_[4];
						int fw_len = cmd_buf_[tmp_len];
						top_fw_version_.assign((char*)&cmd_buf_[tmp_len+1], fw_len);
						//printf("top_fw_version_:%s\n", top_fw_version_.c_str());
						init_info_flag_ = false;
					}
					else
					{
						printf("find tail fail, tmp_buf_len:%d\n", tmp_buf_len);
					}
					cmd_buf_.clear();
				}
			}
			else
			{
				cmd_buf_.clear();
				head_flag = 0;
			}

		}
	}

	bool OrdlidarDriver::uart_data_handle(unsigned char *data, int len)
	{
		// printf("uart_data_handle, len= %d\n", len);
		for (int i = 0; i < len; i++)
		{
			bin_buf_.push_back(*(data + i));
		}

		if (bin_buf_.size() < sizeof(OradarLidarFrame))
		{
			return false;
		}

		if (bin_buf_.size() > sizeof(OradarLidarFrame) * 100)
		{
			bin_buf_.clear();
			return false;
		}

		uint32_t pos = 0;
		int flag = 0;
		uint32_t frame_size = 0, temp_size = 0;
		unsigned char per_block_buff[MAX_BLOCK_SIZE] = {0};
		int n = 0;
		unsigned char crc_check = 0;
		static float last_angle = 0;

		while (pos < (bin_buf_.size() - 2))
		{
			pos = 0;
			while (pos < (bin_buf_.size() - 2))
			{
				if (parsed_data_.head_flag == HEAD_FLAG_NONE)
				{
					if ((bin_buf_[pos] == 0x54) && (bin_buf_[pos + 1] == 0x2C))
					{
						parsed_data_.head_flag = HEAD_FLAG_OK;
						parsed_data_.version = VERSION_MS200;
						break;
					}
				}
				else
				{
					break;
				}
				pos++;
			}

			if (pos != 0)
			{
				for (int i = 0; i < pos; i++)
				{
					bin_buf_.erase(bin_buf_.begin());
				}
			}

			if (bin_buf_.size() < sizeof(OradarLidarFrame))
			{
				return false;
			}
			if (VERSION_MS200 == parsed_data_.version)
			{
				OradarLidarFrame *pkg = (OradarLidarFrame *)bin_buf_.data();
				crc_check = 0;
				for (uint32_t i = 0; i < sizeof(OradarLidarFrame) - 1; i++)
				{
					crc_check = CrcTable[(crc_check ^ bin_buf_[i]) & 0xff];
				}

				if (crc_check == pkg->crc8)
				{
					parsed_data_.point_num = POINT_PER_PACK;
					parsed_data_.speed = pkg->speed / 360.0; // Hz
					parsed_data_.start_angle = pkg->start_angle / 100.0;
					parsed_data_.end_angle = pkg->end_angle / 100.0;
					parsed_data_.timestamp = pkg->timestamp;

					if (temp_data.vailtidy_point_num + parsed_data_.point_num < POINT_CIRCLE_MAX_SIZE)
					{
						//point_data_parse_frame_ms200(&temp_data.data[temp_data.vailtidy_point_num], (unsigned char *)pkg->point, parsed_data_.point_num * 3, parsed_data_.start_angle, parsed_data_.end_angle);
						point_data_parse_frame_ms200(&temp_data.data[temp_data.vailtidy_point_num], pkg);
						if (parsed_data_.point_num < POINT_PKG_MAX_SIZE)
						{
							// one package data
							memcpy(&one_scan_data_.data[0], &temp_data.data[temp_data.vailtidy_point_num], sizeof(point_data_t) * parsed_data_.point_num);
							one_scan_data_.vailtidy_point_num = parsed_data_.point_num;
							one_scan_data_.speed = parsed_data_.speed;
							one_scan_ready_ = true;
							one_data_event_.set();
						}

						for (int i = 0; i < POINT_PER_PACK; i++)
						{
							if ((last_angle - temp_data.data[temp_data.vailtidy_point_num + i].angle) > 1)
							{
								// printf("\n#################### MS200: %d #################\n", full_scan_data_.vailtidy_point_num);
								full_scan_ready_ = false;
								memcpy(&full_scan_data_.data, &temp_data.data, sizeof(point_data_t) * (temp_data.vailtidy_point_num + i));
								full_scan_data_.speed = parsed_data_.speed;
								full_scan_data_.vailtidy_point_num = temp_data.vailtidy_point_num + i;
								if (full_scan_data_.data[0].angle > 10)
								{
									// printf("not a full scan, start angle:%f, end angle:%f\n", full_scan_data_.data[0].angle,
									//	   full_scan_data_.data[full_scan_data_.vailtidy_point_num - 1].angle);
								}
								else
								{
									full_scan_ready_ = true;
									full_data_event_.set();
								}

								last_angle = temp_data.data[temp_data.vailtidy_point_num + POINT_PER_PACK - 1].angle;
								memcpy(&temp_data.data[0], &temp_data.data[temp_data.vailtidy_point_num + i], sizeof(point_data_t) * (POINT_PER_PACK - i));
								temp_data.vailtidy_point_num = POINT_PER_PACK - i;
								flag = 1;
								break;
							}
							last_angle = temp_data.data[temp_data.vailtidy_point_num + i].angle;
						}

						if (flag)
						{
							flag = 0;
						}
						else
						{
							temp_data.vailtidy_point_num += parsed_data_.point_num;
						}
					}
					else
					{
						temp_data.vailtidy_point_num = 0;
					}

					parsed_data_.head_flag = HEAD_FLAG_NONE;
					parsed_data_.version = VERSION_NONE;
					for (uint32_t i = 0; i < sizeof(OradarLidarFrame); i++)
					{
						bin_buf_.erase(bin_buf_.begin());
					}

					if (bin_buf_.size() < sizeof(OradarLidarFrame))
					{
						break;
					}
				}
				else
				{
					parsed_data_.head_flag = HEAD_FLAG_NONE;
					parsed_data_.version = VERSION_NONE;
					/*only remove header,not all frame,because lidar data may contain head*/
					for (int i = 0; i < 2; i++)
					{
						bin_buf_.erase(bin_buf_.begin());
					}
				}
			}
		}
		return true;
	}

	void OrdlidarDriver::mRxThreadProc(void *arg)
	{
		OrdlidarDriver *lidar_ptr = (OrdlidarDriver *)arg;
		unsigned char readbuf[TMPBUFF_SIZE] = {0};
		size_t wait_size = 1; // sizeof(OradarLidarFrame);
		size_t retval, recv_size, actual_read;
		while (!lidar_ptr->rx_thread_exit_flag_.load())
		{
			// retval = lidar_ptr->serial_->available();
			retval = lidar_ptr->serial_->waitfordata(wait_size, 500, &recv_size);
			if (retval == 0)
			{
				lidar_ptr->valid_data_ = 1;
				actual_read = lidar_ptr->serial_->readData(readbuf, recv_size);
				if (actual_read > 0)
				{
					switch (lidar_ptr->model_)
					{
					case ORADAR_MS200:
					{
						lidar_ptr->uart_data_handle(readbuf, actual_read);
						if (lidar_ptr->init_info_flag_)
						{
							lidar_ptr->uart_data_find_init_info(readbuf, actual_read);
						}
						break;
					}
					}
					
				}
			}
			else
			{
				lidar_ptr->valid_data_ = 0;
			}
		}
	}

	int OrdlidarDriver::point_data_parse_frame_ms200(point_data_t *data, OradarLidarFrame *pkg)
	{

		if (data == NULL || pkg == NULL)
		{
			return -1;
		}

		uint32_t diff = ((uint32_t)pkg->end_angle + 36000 - (uint32_t)pkg->start_angle) % 36000;
		float step = diff / (POINT_PER_PACK - 1) / 100.0;
		float start = (double)pkg->start_angle / 100.0;
        float end = (double)(pkg->end_angle % 36000) / 100.0;
		
		for (int i = 0; i < POINT_PER_PACK; i++)
		{
			data[i].distance = pkg->point[i].distance;

			data[i].intensity = pkg->point[i].confidence;

			float tmp_angle = start + i * step;
			if (tmp_angle >= 360)
			{
				data[i].angle = tmp_angle - 360;
			}
			else
			{
				data[i].angle = tmp_angle;
			}
		}
		return 0;
	}
	#if 0
	int OrdlidarDriver::point_data_parse_frame_ms200(point_data_t *data, unsigned char *buf, unsigned short buf_len, float start_angle, float end_angle)
	{

		if (data == NULL || buf == NULL || buf_len == 0)
		{
			return -1;
		}
		unsigned short point_num = buf_len / 3;

		float angle_ratio;
		if (point_num > 1)
		{
			start_angle < end_angle ? (angle_ratio = (end_angle - start_angle) / (point_num - 1)) : (angle_ratio = (end_angle + 360 - start_angle) / (point_num - 1));
		}
		else
		{
			angle_ratio = 0;
		}
		for (int i = 0; i < point_num; i++)
		{

			data[i].distance = buf[i * 3 + 1] * 256 + buf[i * 3 + 0];
			// printf("i %d \n",i);

			data[i].intensity = buf[i * 3 + 2];

			float tmp_angle = start_angle + angle_ratio * i;
			if (tmp_angle > 360)
			{
				data[i].angle = tmp_angle - 360;
			}
			else
			{
				data[i].angle = tmp_angle;
			}
		}
		return 0;
	}
	#endif
	int OrdlidarDriver::read(unsigned char *data, int length)
	{
		// return serial_port_->uart_read(serial_fd_, data, length);
		return 0;
	}

	int OrdlidarDriver::write(unsigned char *data, int length)
	{
		int ret = 0;

		// ret = serial_port_->uart_write(serial_fd_, data, length);
		if (ret < 0)
			printf("lidar write error \n");

		return ret;
	}

	void OrdlidarDriver::Disconnect()
	{
		rx_thread_exit_flag_ = true;
		if (serial_)
		{
			if (serial_->isOpen())
			{
				serial_->closePort();
			}
		}

		if ((rx_thread_ != nullptr) && rx_thread_->joinable())
		{
			rx_thread_->join();
			delete rx_thread_;
			rx_thread_ = NULL;
		}

		ResetFullScanReady();
		ResetOneScanReady();

		is_connected_ = false;
	}

	uint16_t OrdlidarDriver::GetTimestamp() const
	{
		return parsed_data_.timestamp;
	}

	double OrdlidarDriver::GetRotationSpeed() const
	{
		return parsed_data_.speed;
	}

	bool OrdlidarDriver::SetRotationSpeed(int speed)
	{
		uart_comm_t request;
		uint16_t timeout = 0;
		double cur_speed = 0.0, min_thr = 0.0, max_thr = 0.0;
		bool return_value = false;
		int set_flag = 0;
		if(speed < 5)
			speed = 5;
		if(speed > 15)
			speed = 15;

		if(valid_data_)
		{
			set_flag = 1;
			return_value = Deactive();
			if(return_value != true)
			{
				return false;
			}
		}

        delay(400);
		request.head_flag = HEAD_FLAG;
		request.cmd = SET_ROTATION_SPEED;
		request.cmd_type = WRITE_PARAM;
		request.payload_len = 2;
		request.data[0] = speed & 0x00FF;
		request.data[1] = speed & 0xFF00;
		request.data[2] = Utils::xor_check((uint8_t *)&request, HEAD_LEN + request.payload_len);
		request.data[3] = 0x31;
		request.data[4] = 0xF2;
		int ret = serial_->writeData((uint8_t*)&request, HEAD_LEN + request.payload_len + 3);
		if(ret == -1)
		{
			return false;
		}

		if(set_flag)
		{
			return_value = Activate();
			if(return_value != true)
			{
				return false;
			}
		}
		else
		{
			return true;
		}

		min_thr = (double)speed - ((double)speed  * 0.1);
		max_thr = (double)speed + ((double)speed  * 0.1);
		return_value = false;
		while(timeout < SET_TIME_OUT)
		{
			delay(1000);
			cur_speed = GetRotationSpeed();
			if(valid_data_ == 0)
			{
				break;
			}
			if((cur_speed > min_thr) && (cur_speed < max_thr))
			{
				return_value = true;
				break;
			}
			timeout++;
		}

		return return_value;
	}

	bool OrdlidarDriver::GetFirmwareVersion(std::string &top_fw_version, std::string &bot_fw_version)
	{
		if((!top_fw_version_.empty()) && (!bot_fw_version_.empty()))
		{
			top_fw_version = top_fw_version_;
			bot_fw_version = bot_fw_version_;
			return true;
		}

		printf("please soft restart the lidar device !\n");
		init_info_flag_ = true;
		return false;
	}

	bool OrdlidarDriver::GetDeviceSN(std::string &device_sn)
	{
		if(!device_sn_.empty())
		{
			device_sn = device_sn_;
			return true;
		}

		printf("please soft restart the lidar device !\n");
		init_info_flag_ = true;
		return false;
	}

	bool OrdlidarDriver::Activate(void)
	{
		uart_comm_t request;
		int ret = false;
		uint16_t timeout = 0;

		if(valid_data_)
		{
			return true;
		}

		delay(400);						//防止连续调用，导致雷达无法处理
		request.head_flag = HEAD_FLAG;
		request.cmd = SET_RUN_MODE;
		request.cmd_type = WRITE_PARAM;
		request.payload_len = 1;
		request.data[0] = 0X81;
		request.data[1] = Utils::xor_check((uint8_t *)&request, HEAD_LEN + request.payload_len);
		request.data[2] = 0x31;
		request.data[3] = 0xF2;
		int ret_value = serial_->writeData((uint8_t*)&request, HEAD_LEN + request.payload_len + 3);
		if(ret_value == -1)
		{
			return false;
		}

		ret = false;
		while(timeout < SET_TIME_OUT)
		{
			delay(1000);
			if(valid_data_)
			{
				ret = true;
				break;
			}
			timeout++;
		}
		return ret;
	}

	bool OrdlidarDriver::Deactive(void)
	{
		uart_comm_t request;
		bool ret = false;
		uint16_t timeout = 0;
		if(valid_data_ == 0)
		{
			full_scan_data_.vailtidy_point_num = 0;
			one_scan_data_.vailtidy_point_num = 0;
			parsed_data_.timestamp = 0;
			parsed_data_.speed = 0;
			return true;
		}

		delay(400);						//防止连续调用，导致雷达无法处理
		request.head_flag = HEAD_FLAG;
		request.cmd = SET_RUN_MODE;
		request.cmd_type = WRITE_PARAM;
		request.payload_len = 1;
		request.data[0] = 0X80;
		request.data[1] = Utils::xor_check((uint8_t *)&request, HEAD_LEN + request.payload_len);
		request.data[2] = 0x31;
		request.data[3] = 0xF2;
		int ret_value = serial_->writeData((uint8_t*)&request, HEAD_LEN + request.payload_len + 3);
		if(ret_value == -1)
		{
			return false;
		}

		ret = false;
		while(timeout < SET_TIME_OUT)
		{
			delay(1000);
			if(valid_data_ == 0)
			{
				full_scan_data_.vailtidy_point_num = 0;
				one_scan_data_.vailtidy_point_num = 0;
				parsed_data_.timestamp = 0;
				parsed_data_.speed = 0;
				ret = true;
				break;
			}
			timeout++;
		}

		return ret;
	}

	bool OrdlidarDriver::GrabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms)
	{
		switch (full_data_event_.wait(timeout_ms))
		{
		case Event::EVENT_TIMEOUT:
		{
			return false;
		}
		case Event::EVENT_OK:
		{
			// ScopedLocker l(lock_);
			if (IsFullScanReady())
			{
				scan_data = full_scan_data_;
				ResetFullScanReady();
				return true;
			}
		}
		}
		return false;
	}

	bool OrdlidarDriver::GrabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms)
	{

		switch (one_data_event_.wait(timeout_ms))
		{
		case Event::EVENT_TIMEOUT:
			return false;

		case Event::EVENT_OK:
		{
			// ScopedLocker l(lock_);
			if (IsOneScanReady())
			{
				scan_data = one_scan_data_;
				ResetOneScanReady();
				return true;
			}
		}
		}

		return false;
	}

	bool OrdlidarDriver::GrabFullScan(full_scan_data_st &scan_data)
	{
		if (full_scan_data_.vailtidy_point_num == 0)
		{
			return false;
		}

		if (!IsFullScanReady())
			return false;

		scan_data = full_scan_data_;
		ResetFullScanReady();
		return true;
	}

	bool OrdlidarDriver::GrabOneScan(one_scan_data_st &scan_data)
	{
		if (one_scan_data_.vailtidy_point_num == 0)
		{
			return false;
		}

		if (!IsOneScanReady())
			return false;

		scan_data = one_scan_data_;
		ResetOneScanReady();
		return true;
	}

}
