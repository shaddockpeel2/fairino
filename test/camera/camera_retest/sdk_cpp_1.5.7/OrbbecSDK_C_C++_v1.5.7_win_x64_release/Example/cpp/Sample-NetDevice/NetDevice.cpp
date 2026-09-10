#include "../window.hpp"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>
#include <string>

std::shared_ptr<ob::FrameSet> currentFrameSet;
std::mutex                    frameSetMutex;

int main(int argc, char **argv) try {
    // 创建一个 Context，用于获取设备列表
    ob::Context ctx;

    // 输入设备 ip 地址（当前仅 FemtoMega 设备支持网络连接，其默认 ip 地址为 192.168.1.10）
    std::string ip;
    std::cout << "Input your device ip(default: 192.168.1.10):";
    std::getline(std::cin, ip);
    if(ip.empty()) {
        ip = "192.168.1.10";
    }

    // 通过 ip 创建网络设备（端口号默认为：8090，当前支持网络模式的设备暂不支持修改端口号）
    auto device = ctx.createNetDevice(ip.c_str(), 8090);

    // 传入 device 创建 pipeline
    auto pipe = std::make_shared<ob::Pipeline>(device);

    // 创建 Config 用于配置 Pipeline 工作
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // 获取深度相机配置列表
    auto depthProfileList = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    // 使用默认配置
    auto depthProfile = depthProfileList->getProfile(0);
    // 使能深度流
    config->enableStream(depthProfile);

    // 获取彩色相机配置列表
    auto colorProfileList = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    // 使用默认配置
    auto colorProfile = colorProfileList->getProfile(0);
    // 使能深度流
    config->enableStream(colorProfile);

    // 传入配置，启动 pipeline
    pipe->start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(frameSetMutex);
        currentFrameSet = frameSet;
    });

    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("MultiDeviceViewer", 1280, 720);

    while(app) {
        std::shared_ptr<ob::FrameSet> frameSet;
        {  // 通过大括号定义作用域，lock 退出该作用域后自动解锁。及时解锁可避免 pipeline 的 frameset 输出线程阻塞太久而导致内部缓存增加及数据帧延时增加
            std::lock_guard<std::mutex> lock(frameSetMutex);
            frameSet = currentFrameSet;
        }
        if(frameSet) {
            auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
            auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
            if(colorFrame && depthFrame) {
                if(colorFrame->format() == OB_FORMAT_H264 || colorFrame->format() == OB_FORMAT_H265) {
                    // 对于 H264 和 H265 格式图像帧，用户可参考 FFmpeg 等解码库及其例程完成解码和渲染显示，本例为保持工程及代码简洁，不做展示。
                    if(colorFrame->index() % 30 == 0) {
                        // 每 30 帧打印一次 Color 数据帧信息
                        std::cout << "Color Frame: index=" << colorFrame->index() << ", timestamp=" << colorFrame->timeStamp();
                    }
                    app.render({ depthFrame }, RENDER_SINGLE);
                }
                else {
                    app.render({ colorFrame, depthFrame }, RENDER_ONE_ROW);
                }
            }
        }

        // 休眠 30ms，控制渲染显示刷新率
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // 停止 pipeline
    pipe->stop();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}