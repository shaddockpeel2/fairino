#include "../window.hpp"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <mutex>

const int maxDeviceCount = 9;

std::vector<std::shared_ptr<ob::Frame>> frames;
std::shared_ptr<ob::Frame>              colorFrames[maxDeviceCount];
std::shared_ptr<ob::Frame>              depthFrames[maxDeviceCount];
std::shared_ptr<ob::Frame>              irFrames[maxDeviceCount];
std::mutex                              frameMutex;

void StartStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes);
void StopStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes);

int main(int argc, char **argv) try {
    // 创建一个Context，用于获取设备列表
    ob::Context ctx;

    // 查询已经接入设备的列表
    auto devList = ctx.queryDeviceList();
    // 获取接入设备的数量
    int devCount = devList->deviceCount();

    // 遍历设备列表并创建pipe
    std::vector<std::shared_ptr<ob::Pipeline>> pipes;
    for(int i = 0; i < devCount; i++) {
        // 获取设备并创建pipeline
        auto dev  = devList->getDevice(i);
        auto pipe = std::make_shared<ob::Pipeline>(dev);
        pipes.push_back(pipe);
    }

    // 打开所有设备的深度流和彩色流
    StartStream(pipes);

    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("MultiDeviceViewer", 1280, 720);

    while(app) {
        // 在窗口中渲染一组帧数据，这里将渲染所有设备的深度、彩色或者红外帧，RENDER_GRID
        // 表示将所有的帧按照格子排列显示
        frames.clear();
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            int                         i = 0;
            for(auto pipe: pipes) {
                if(colorFrames[i] != nullptr) {
                    frames.emplace_back(colorFrames[i]);
                }

                if(depthFrames[i] != nullptr) {
                    frames.emplace_back(depthFrames[i]);
                }

                if(irFrames[i] != nullptr) {
                    frames.emplace_back(irFrames[i]);
                }
                i++;
            }
        }

        app.render(frames, RENDER_GRID);
    }

    frames.clear();
    StopStream(pipes);

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

void StartStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes) {
    int i = 0;
    for(auto &&pipe: pipes) {
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
        // 获取深度相机配置列表
        auto                                    depthProfileList = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
        std::shared_ptr<ob::VideoStreamProfile> depthProfile     = nullptr;
        if(depthProfileList) {
            // 打开Depth Sensor默认的profile,可通过配置文件配置
            depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        config->enableStream(depthProfile);

        // 获取彩色相机配置列表
        try {
            auto                                    colorProfileList = pipe->getStreamProfileList(OB_SENSOR_COLOR);
            std::shared_ptr<ob::VideoStreamProfile> colorProfile     = nullptr;
            if(colorProfileList) {
                // 打开Color Sensor默认的profile,可通过配置文件配置
                colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
            }
            config->enableStream(colorProfile);
        }
        catch(ob::Error &e) {
            std::cerr << "Current device is not support color sensor!" << std::endl;
        }

        // 启动pipeline，并传入配置
        pipe->start(config, [i](std::shared_ptr<ob::FrameSet> frameSet) {
            std::lock_guard<std::mutex> lock(frameMutex);
            if(frameSet->colorFrame()) {
                colorFrames[i] = frameSet->colorFrame();
            }
            if(frameSet->depthFrame()) {
                depthFrames[i] = frameSet->depthFrame();
            }
        });
        i++;
    }
}

void StopStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes) {
    int i = 0;
    for(auto &&pipe: pipes) {
        if(colorFrames[i])
            colorFrames->reset();
        if(depthFrames[i])
            depthFrames->reset();
        if(irFrames[i])
            irFrames->reset();
        // 停止pipeline
        pipe->stop();
        i++;
    }
}
