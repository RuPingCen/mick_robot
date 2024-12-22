// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#include "ord_lidar_c_api_driver.h"
#include "ord_lidar_driver.h"

using namespace ordlidar;

ORDLidar *oradar_lidar_create(uint8_t type, int model)
{
    OrdlidarDriver *driver = new OrdlidarDriver(type, model);
    ORDLidar *instance = new ORDLidar;
    instance->lidar = NULL;
    instance->lidar = (void *)driver;
    return instance;
}

void oradar_lidar_destroy(ORDLidar **lidar)
{
    if (lidar == NULL || *lidar == NULL)
    {
        return;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>((*lidar)->lidar);

    if (drv)
    {
        delete drv;
        drv = NULL;
    }

    (*lidar)->lidar = NULL;
    delete *lidar;
    *lidar = NULL;
    return;
}

bool oradar_set_serial_port(ORDLidar *lidar, char *port, int baudrate)
{
    if (lidar == NULL || lidar->lidar == NULL || port == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        return drv->SetSerialPort(port, baudrate);
    }

    return false;
}

bool oradar_connect(ORDLidar *lidar)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        return drv->Connect();
    }

    return false;
}
bool oradar_disconnect(ORDLidar *lidar)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        drv->Disconnect();
    }

    return true;
}

bool oradar_get_timestamp(ORDLidar *lidar, uint16_t *timestamp)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        *timestamp = drv->GetTimestamp();
        return true;
    }

    return false;
}
  
bool oradar_get_rotation_speed(ORDLidar *lidar, double *rotation_speed)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        *rotation_speed = drv->GetRotationSpeed();
        return true;
    }

    return false;
}

bool oradar_get_firmware_version(ORDLidar *lidar, char *top_fw_version, char *bot_fw_version)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }
    
    std::string top_fw, bot_fw;
    //str.copy()
    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        if(drv->GetFirmwareVersion(top_fw, bot_fw))
        {
            strcpy(top_fw_version, top_fw.c_str());
            strcpy(bot_fw_version, bot_fw.c_str());
            return true;
        }
    }

    return false;
}

bool oradar_get_device_sn(ORDLidar *lidar, char *device_sn)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }
    
    std::string sn;
    //str.copy()
    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        if(drv->GetDeviceSN(sn))
        {
            strcpy(device_sn, sn.c_str());
            return true;
        }
    }

    return false;
}

bool oradar_set_rotation_speed(ORDLidar *lidar, uint16_t speed)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        return drv->SetRotationSpeed(speed);
    }

    return false;
}

bool oradar_activate(ORDLidar *lidar)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        return drv->Activate();
    }

    return false;
}

bool oradar_deactive(ORDLidar *lidar)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        return drv->Deactive();
    }

    return false;
}

bool oradar_get_grabonescan_blocking(ORDLidar *lidar, one_scan_data_st *data, int timeout_ms)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    one_scan_data_st scan_data;
    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        if(drv->GrabOneScanBlocking(scan_data, timeout_ms))
        {
            memcpy(data, &scan_data, sizeof(one_scan_data_st));
            return true;
        }
         
    }

    return false;
}

bool oradar_get_grabfullscan_blocking(ORDLidar *lidar, full_scan_data_st *data, int timeout_ms)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    full_scan_data_st scan_data;
    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        if(drv->GrabFullScanBlocking(scan_data, timeout_ms))
        {
            memcpy(data, &scan_data, sizeof(full_scan_data_st));
            return true;
        }
    }

    return false;
}

bool oradar_get_grabonescan(ORDLidar *lidar, one_scan_data_st *data)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    one_scan_data_st scan_data;
    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        if(drv->GrabOneScan(scan_data))
        {
            memcpy(data, &scan_data, sizeof(one_scan_data_st));
            return true;
        }
    }

    return false;
}

bool oradar_get_grabfullscan(ORDLidar *lidar, full_scan_data_st *data)
{
    if (lidar == NULL || lidar->lidar == NULL)
    {
        return false;
    }

    full_scan_data_st scan_data;
    OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
    if(drv)
    {
        if(drv->GrabFullScan(scan_data))
        {
            memcpy(data, &scan_data, sizeof(full_scan_data_st));
            return true;
        }
    }

    return false;
}


