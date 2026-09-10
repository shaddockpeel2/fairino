#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <libobsensor/ObSensor.hpp>
#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif

#define ESC 27

static void pressKeyExit(std::string msg) {
    if (msg.size() > 0) {
        std::cout << msg << std::endl;
    }

    while(true) {
        //获取按下的按键的值，如果是ESC键，退出程序
        int key = getch();
        if(key == ESC)
            break;
    }
}

static std::string   toHexText(const uint8_t *data, const uint32_t dataLen);
static std::ostream &operator<<(std::ostream &os, const OBDepthWorkMode &dataType);

int main(int argc, char *argv[]) try {
    //打印SDK的版本号，SDK版本号分为主版本号、副版本号和修订版本号
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;

    //创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭
    //多种类型的流并获取一组帧数据
    ob::Pipeline pipe;

    // 获取设备列表中的第一个设备
    auto device = pipe.getDevice();

    // 检查是否支持相机深度工作模式，目前（2022年12月5日）仅Gemini2双目摄像头支持深度工作模式
    if (!device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
        pressKeyExit("Current device not support depth work mode!");
        return -1;
    }

    // 查询当前的相机深度模式
    auto curDepthMode = device->getCurrentDepthWorkMode();

    // 获取相机深度模式列表
    auto depthModeList = device->getDepthWorkModeList();
    std::cout << "depthModeList size: " << depthModeList->count() << std::endl;
    for(uint32_t i = 0; i < depthModeList->count(); i++) {
        std::cout << "depthModeList[" << i << "]: " << (*depthModeList)[i];
        if (strcmp(curDepthMode.name, (*depthModeList)[i].name) == 0) {
            std::cout << "  (Current WorkMode)";
        }

        std::cout << std::endl;
    }

    // 让用户选择一个模式，然后进行切换
    if(depthModeList->count() > 0) {
        uint32_t index = 0;
        std::cout << "Please input the index from above depthModeList, newIndex = ";
        std::cin >> index;
        if(index >= 0 && index < depthModeList->count()) {  // 合法性检查
            device->switchDepthWorkMode((*depthModeList)[index].name);

            // 显示切换后再查询模式是否变化
            curDepthMode = device->getCurrentDepthWorkMode();
            if (strcmp(curDepthMode.name, (*depthModeList)[index].name) == 0) {
                std::cout << "Switch depth work mode success! currentDepthMode: " << curDepthMode << std::endl;
            } else {
                std::cout << "Switch depth work mode failed!" << std::endl;
            }
        }
        else {
            std::cout << "switchDepthMode faild. invalid index: " << index << std::endl;
        }
    }

    // 到此切换相机深度模式结束，可以用pipeline进行打开相机取流
    // 注意： 
    // 1. 如果需要切换相机深度模式，那么打开数据流必须在切换深度工作模式之后；每个相机深度模式下支持的有效分辨率不同
    // 2. 如果已经用pipeline打开数据流，那么切花相机深度工作模式前必须把原来申请的pipeline释放； 
    //    切换相机深度工作模式后重新创建pipeline，否则会造成野指针或者内存泄露；

    pressKeyExit("Press ESC to exit!");
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

static std::string toHexText(const uint8_t *data, const uint32_t dataLen) {
    static char alpha[] = "0123456789abcdef";
    char *      hexStr  = new char[dataLen * 2 + 1]{ 0 };
    for(uint32_t i = 0; i < dataLen; i++) {
        uint8_t val       = *(data + i);
        *(hexStr + i)     = alpha[(val >> 4) & 0xf];
        *(hexStr + i + 1) = alpha[val & 0xf];
    }

    std::string ret(hexStr);
    delete[] hexStr;
    return ret;
}

static std::ostream &operator<<(std::ostream &os, const OBDepthWorkMode &workMode) {
    os << "{name: " << workMode.name << ", checksum: " << toHexText(workMode.checksum, sizeof(workMode.checksum)) << "}";
    return os;
}