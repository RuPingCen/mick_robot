#include <stdio.h>
#include <signal.h>
#include "ord_lidar_c_api_driver.h"

int running = 1;

static void sig_handle(int signo)
{
    printf("program exit, [%s,%s] Receive SIGNAL %d ====== \r\n", __FILE__, __func__, signo);
    running = 0;
    exit(1);
}

int main(int argc, char *argv[])
{
    (void)argc;(void)argv;
    
    signal(SIGINT, sig_handle);

    full_scan_data_st scan_data;
    #if defined(_WIN32)
    char port_name[50] = "com18";
    #else
    char port_name[50] = "/dev/ttyACM0";
    #endif
    int baudrate = 230400;
    bool ret = false;
    bool is_logging = false; 
    uint8_t type = ORADAR_TYPE_SERIAL;
    int model = ORADAR_MS200;
    ORDLidar *laser = oradar_lidar_create(ORADAR_TYPE_SERIAL, ORADAR_MS200);
    ret = oradar_set_serial_port(laser, port_name, baudrate);
    if (!ret)
    {
        printf("serial port_name set fail\n");
        goto exit;
    }
    ret = oradar_connect(laser);
    if(!ret)
    {
        printf("serial connect fail\n");
        goto exit;
    }

    while (running)
    {
        if (oradar_get_grabfullscan_blocking(laser, &scan_data, 1000))
        {
            printf("vailtidy_point_num: %d\n", scan_data.vailtidy_point_num);
            if(is_logging)
            {
                for (int i = 0; i < scan_data.vailtidy_point_num; i++)
                {
                    printf("[%d: %f, %f] \n", i, (scan_data.data[i].distance * 0.001), scan_data.data[i].angle);
                }
            }
        }
        else
        {
            printf("fail to get lidar data\n");
        }
    }

exit:
    oradar_lidar_destroy(&laser);
    return 0;
}
