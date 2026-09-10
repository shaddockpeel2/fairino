#include <iostream>
#include <mutex>
#include <libobsensor/ObSensor.hpp>
#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif

#define ESC 27
std::mutex printerMutex;
int        main(int argc, char **argv) try {
    // 打印SDK的版本号，SDK版本号分为主版本号，副版本号和修订版本号
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;

    // 创建一个Context，与Pipeline不同，Context是底层API的入口，在开关流等常用操作上
    // 使用低级会稍微复杂一些，但是底层API可以提供更多灵活的操作，如获取多个设备，读写
    // 设备及相机的属性等
    ob::Context ctx;

    // 查询已经接入设备的列表
    auto devList = ctx.queryDeviceList();

    // 获取接入设备的数量
    if(devList->deviceCount() == 0) {
        std::cerr << "Device not found!" << std::endl;
        return -1;
    }

    // 创建设备，0表示第一个设备的索引
    auto                        dev         = devList->getDevice(0);
    std::shared_ptr<ob::Sensor> gyroSensor  = nullptr;
    std::shared_ptr<ob::Sensor> accelSensor = nullptr;
    try {
        // 获取陀螺仪Sensor
        gyroSensor = dev->getSensorList()->getSensor(OB_SENSOR_GYRO);
        if(gyroSensor) {
            // 获取配置列表
            auto profiles = gyroSensor->getStreamProfileList();
            // 选择第一个配置开流
            auto profile = profiles->getProfile(0);
            gyroSensor->start(profile, [](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(printerMutex);
                auto                         timeStamp = frame->timeStamp();
                auto                         gyroFrame = frame->as<ob::GyroFrame>();
                if(gyroFrame != nullptr && (timeStamp % 500) < 2) {  //  ( timeStamp % 500 ) < 2: 目的时减少打印频率
                    auto value = gyroFrame->value();
                    std::cout << "Gyro Frame: \n\r{\n\r"
                              << "  tsp = " << timeStamp << "\n\r"
                              << "  temperature = " << gyroFrame->temperature() << "\n\r"
                              << "  gyro.x = " << value.x << " dps"
                              << "\n\r"
                              << "  gyro.y = " << value.y << " dps"
                              << "\n\r"
                              << "  gyro.z = " << value.z << " dps"
                              << "\n\r"
                              << "}\n\r" << std::endl;
                }
            });
        }
        else {
            std::cout << "get gyro Sensor failed ! " << std::endl;
        }
    }
    catch(ob::Error &e) {
        std::cerr << "current device is not support imu!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 获取加速度Sensor
    accelSensor = dev->getSensorList()->getSensor(OB_SENSOR_ACCEL);
    if(accelSensor) {
        // 获取配置列表
        auto profiles = accelSensor->getStreamProfileList();
        // 选择第一个配置开流
        auto profile = profiles->getProfile(0);
        accelSensor->start(profile, [](std::shared_ptr<ob::Frame> frame) {
            std::unique_lock<std::mutex> lk(printerMutex);
            auto                         timeStamp  = frame->timeStamp();
            auto                         accelFrame = frame->as<ob::AccelFrame>();
            if(accelFrame != nullptr && (timeStamp % 500) < 2) {
                auto value = accelFrame->value();
                std::cout << "Accel Frame: \n\r{\n\r"
                          << "  tsp = " << timeStamp << "\n\r"
                          << "  temperature = " << accelFrame->temperature() << "\n\r"
                          << "  accel.x = " << value.x << " g"
                          << "\n\r"
                          << "  accel.y = " << value.y << " g"
                          << "\n\r"
                          << "  accel.z = " << value.z << " g"
                          << "\n\r"
                          << "}\n\r" << std::endl;
            }
        });
    }
    else {
        std::cout << "get Accel Sensor failed ! " << std::endl;
    }

    std::cout << "Press ESC to exit! " << std::endl;

    while(true) {
        // 获取按下的按键的值，如果是ESC键，退出程序
        int key = getch();
        if(key == ESC)
            break;
    }

    // 关流
    if(gyroSensor) {
        gyroSensor->stop();
    }
    if(accelSensor) {
        accelSensor->stop();
    }
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}