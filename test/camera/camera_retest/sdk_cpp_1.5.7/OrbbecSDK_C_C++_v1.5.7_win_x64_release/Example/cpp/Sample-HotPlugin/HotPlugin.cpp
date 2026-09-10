#include "libobsensor/ObSensor.hpp"

#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif
#include <iostream>
#include <thread>

#define ESC 27
std::shared_ptr<ob::Pipeline> pipeline = NULL;

void CreateAndStartWithConfig() {
    //按配置文件的流配置启动流，如果没有配置文件，将使用第0个流配置启动流
    try {
        pipeline->start(nullptr);
    }
    catch(...) {
        std::cout << "Pipeline start failed!" << std::endl;
    }
}

//设备连接回调
void DeviceConnectCallback(std::shared_ptr<ob::DeviceList> connectList) {
    std::cout << "Device connect: " << connectList->deviceCount() << std::endl;
    if(connectList->deviceCount() > 0) {
        if(pipeline == NULL) {
            pipeline = std::make_shared<ob::Pipeline>();
            CreateAndStartWithConfig();
        }
    }
}

//设备断开回调
void DeviceDisconnectCallback(std::shared_ptr<ob::DeviceList> disconnectList) {
    std::cout << "Device disconnect: " << disconnectList->deviceCount() << std::endl;
    if(disconnectList->deviceCount() > 0) {

        try {
            pipeline->stop();
        }
        catch(ob::Error &e) {
            std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType()
                      << std::endl;
        }
        pipeline.reset();
    }
}

int main(int argc, char **argv) try {

    //创建上下文
    ob::Context ctx;

    //注册设备回调
    ctx.setDeviceChangedCallback([](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
        DeviceDisconnectCallback(removedList);
        DeviceConnectCallback(addedList);
    });

    //获取当前设备数量，并开流
    if(ctx.queryDeviceList()->deviceCount() > 0) {
        if(pipeline == NULL) {
            pipeline = std::make_shared<ob::Pipeline>();
            CreateAndStartWithConfig();
        }
    }

    while(true) {
        if(kbhit()) {
            int key = getch();

            //按ESC键退出
            if(key == ESC) {
                break;
            }

            //按r键重启设备触发设备掉线/上线，也可以手动拔插设备触发
            if(key == 'r' || key == 'R') {
                if(pipeline) {
                    pipeline->getDevice()->reboot();
                }
            }
        }
        if(pipeline) {
            //等待一帧数据，超时时间为100ms
            auto frameSet = pipeline->waitForFrames(100);
            if(frameSet == nullptr) {
                continue;
            }

            //获取深度数据帧
            auto depthFrame = frameSet->depthFrame();
            if(depthFrame) {
                uint16_t *data       = (uint16_t *)depthFrame->data();
                auto      width      = depthFrame->width();
                auto      height     = depthFrame->height();
                auto      scale      = depthFrame->getValueScale();
                uint16_t  pixelValue = *(data + width * height / 2 + width / 2);
                std::cout << "=====Depth Frame Info======"
                          << "FrameType: " << depthFrame->type() << ", index: " << depthFrame->index() << ", width: " << width << ", height: " << height
                          << ", format: " << depthFrame->format() << ", timeStampUs: " << depthFrame->timeStampUs()
                          << "us, centerDepth: " << (int)(pixelValue * scale) << "mm" << std::endl;
            }

            //获取Color数据帧
            auto colorFrame = frameSet->colorFrame();
            if(colorFrame) {
                std::cout << "=====Color Frame Info======"
                          << "FrameType: " << colorFrame->type() << ", index: " << colorFrame->index() << ", width: " << colorFrame->width()
                          << ", height: " << colorFrame->height() << ", format: " << colorFrame->format() << ", timeStampUs: " << colorFrame->timeStampUs()
                          << "us" << std::endl;
            }
        }
    }
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}