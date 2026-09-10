#include <iostream>
#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

#define ESC 27

int main(int argc, char **argv) try {
    //打印SDK的版本号，SDK版本号分为主版本号，副版本号和修订版本号
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;

    //创建一个Context，与Pipeline不同，Context是底层API的入口，在开关流等常用操作上
    //使用低级会稍微复杂一些，但是底层API可以提供更多灵活的操作，如获取多个设备，读写
    //设备及相机的属性等
    ob::Context ctx;

    //查询已经接入设备的列表
    auto devList = ctx.queryDeviceList();

    //获取接入设备的数量
    if(devList->deviceCount() == 0) {
        std::cerr << "Device not found!" << std::endl;
        return -1;
    }

    //创建设备，0表示第一个设备的索引
    auto dev = devList->getDevice(0);

    auto colorSensor = dev->getSensor(OB_SENSOR_COLOR);
    auto profileList = colorSensor->getStreamProfileList();

    //获取设备信息
    auto devInfo = dev->getDeviceInfo();

    //获取设备的名称
    std::cout << "Device name: " << devInfo->name() << std::endl;

    //获取设备的pid, vid, uid
    std::cout << "Device pid: " << devInfo->pid() << " vid: " << devInfo->vid() << " uid: " << devInfo->uid() << std::endl;

    //通过获取设备的固件版本号
    auto fwVer = devInfo->firmwareVersion();
    std::cout << "Firmware version: " << fwVer << std::endl;

    //通过获取设备的序列号
    auto sn = devInfo->serialNumber();
    std::cout << "Serial number: " << sn << std::endl;

    //通过获取设备的连接类型
    auto connectType = devInfo->connectionType();
    std::cout << "ConnectionType: " << connectType << std::endl;

    //获取支持的传感器列表
    std::cout << "Sensor types: " << std::endl;
    auto sensorList = dev->getSensorList();
    for(uint32_t i = 0; i < sensorList->count(); i++) {
        auto sensor = sensorList->getSensor(i);
        switch(sensor->type()) {
        case OB_SENSOR_COLOR:
            std::cout << "\tColor sensor" << std::endl;
            break;
        case OB_SENSOR_DEPTH:
            std::cout << "\tDepth sensor" << std::endl;
            break;
        case OB_SENSOR_IR:
            std::cout << "\tIR sensor" << std::endl;
            break;
        case OB_SENSOR_GYRO:
            std::cout << "\tGyro sensor" << std::endl;
            break;
        case OB_SENSOR_ACCEL:
            std::cout << "\tAccel sensor" << std::endl;
            break;
        default:
            break;
        }
    }

    std::cout << "Press ESC to exit! " << std::endl;

    while(true) {
        //获取按下的按键的值，如果是ESC键，退出程序
        int key = getch();
        if(key == ESC)
            break;
    }
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}