#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <libobsensor/ObSensor.h>

#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif

#define ESC 27

void check_error(ob_error *error) {
    if(error) {
        printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(error), ob_error_args(error));
        printf("\tmessage: %s\n", ob_error_message(error));
        printf("\terror type: %d\n", ob_error_exception_type(error));
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv) {

    //打印SDK版本号
    printf("SDK version: %d.%d.%d\n", ob_get_major_version(), ob_get_minor_version(), ob_get_patch_version());

    //创建一个Context，与Pipeline不同，Context是底层API的入口，在开关流等常用操作上
    //使用低级会稍微复杂一些，但是底层API可以提供更多灵活的操作，如获取多个设备，读写
    //设备及相机的属性等
    ob_error *  error = NULL;
    ob_context *ctx   = ob_create_context(&error);
    check_error(error);

    //查询已经接入设备的列表
    ob_device_list *dev_list = ob_query_device_list(ctx, &error);
    check_error(error);

    //获取接入设备的数量
    int dev_count = ob_device_list_device_count(dev_list, &error);
    check_error(error);
    if(dev_count == 0) {
        printf("Device not found!\n");
        return -1;
    }

    //创建设备，0表示第一个设备的索引
    ob_device *dev = ob_device_list_get_device(dev_list, 0, &error);
    check_error(error);

    //获取设备信息
    ob_device_info *dev_info = ob_device_get_device_info(dev, &error);
    check_error(error);

    //获取设备的名称
    const char *name = ob_device_info_name(dev_info, &error);
    check_error(error);
    printf("Device name: %s\n", name);

    //获取设备的pid, vid, uid
    int pid = ob_device_info_pid(dev_info, &error);
    check_error(error);
    int vid = ob_device_info_vid(dev_info, &error);
    check_error(error);
    const char *uid = ob_device_info_uid(dev_info, &error);
    check_error(error);
    printf("Device pid: %d vid: %d uid: %s\n", pid, vid, uid);

    //通过获取设备的固件版本号
    const char *fw_ver = ob_device_info_firmware_version(dev_info, &error);
    check_error(error);
    printf("Firmware version: %s\n", fw_ver);

    //通过获取设备的序列号
    const char *sn = ob_device_info_serial_number(dev_info, &error);
    check_error(error);
    printf("Serial number: %s\n", sn);

    //通过获取设备的连接类型
    const char *connectType = ob_device_info_connection_type(dev_info, &error);
    check_error(error);
    printf("ConnectionType: %s\n", connectType);

    printf("Sensor types: \n");
    //获取支持的传感器列表
    ob_sensor_list *sensor_list = ob_device_get_sensor_list(dev, &error);
    check_error(error);

    //获取传感器数量
    int sensor_count = ob_sensor_list_get_sensor_count(sensor_list, &error);
    check_error(error);
    for(int i = 0; i < sensor_count; i++) {
        //获取传感器类型
        ob_sensor_type sensor_type = ob_sensor_list_get_sensor_type(sensor_list, i, &error);
        check_error(error);
        switch(sensor_type) {
        case OB_SENSOR_COLOR:
            printf("\tColor sensor\n");
            break;
        case OB_SENSOR_DEPTH:
            printf("\tDepth sensor\n");
            break;
        case OB_SENSOR_IR:
            printf("\tIR sensor\n");
            break;
        case OB_SENSOR_ACCEL:
            printf("\tAccel sensor\n");
            break;
        case OB_SENSOR_GYRO:
            printf("\tGyro sensor\n");
            break;
        default:
            break;
        }
    }

    printf("Press ESC to exit! \n");

    while(true) {
        //获取按下的按键的值，如果是ESC键，退出程序
        int key = getch();
        if(key == ESC)
            break;
    }

    //销毁sensor list
    ob_delete_sensor_list(sensor_list, &error);
    check_error(error);

    //销毁device info
    ob_delete_device_info(dev_info, &error);
    check_error(error);

    //销毁device
    ob_delete_device(dev, &error);
    check_error(error);

    //销毁device list
    ob_delete_device_list(dev_list, &error);
    check_error(error);

    //销毁context
    ob_delete_context(ctx, &error);
    check_error(error);

    return 0;
}