#include "../window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <mutex>
#include <thread>

#define D 68
#define d 100
#define F 70
#define f 102
#define S 83
#define s 115
#define add 43
#define reduce 45

static bool  sync         = false;
static bool  started      = true;
static bool  hd2c         = false;
static bool  sd2c         = true;
static float alpha        = 0.5;
static int   preKey       = -1;
static int   windowWidth  = 0;
static int   windowHeight = 0;

std::mutex                    framesetMutex;
std::shared_ptr<ob::FrameSet> frameSet;
bool                          quitApp = false;
/**
 * 同步对齐示例
 *
 * 该示例可能会由于深度或者彩色sensor不支持镜像而出现深度图和彩色图镜像状态不一致的情况，
 * 从而导致深度图和彩色图显示的图像是相反的，如遇到该情况，则通过设置镜像接口保持两个镜像状态一致即可
 * 另外可能存在某些设备获取到的分辨率不支持D2C功能，因此D2C功能以实际支持的D2C分辨率为准
 *
 * 例如：DaBai DCW支持的D2C的分辨率为640x360，而实际该示例获取到的分辨率可能为640x480，此时用户根据实际模组情况获取
 * 对应的640x360分辨率即可
 */
int main(int argc, char **argv) try {
    // 创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭
    // 多种类型的流并获取一组帧数据
    ob::Pipeline pipe;
    // 通过创建Config来配置Pipeline要启用或者禁用哪些流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // 获取彩色相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
        auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
        if(colorProfiles) {
            colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        config->enableStream(colorProfile);
    }
    catch(...) {
        std::cerr << "Current device is not support color sensor!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 获取深度相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    auto                                    depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
    if(depthProfiles) {
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
    }
    config->enableStream(depthProfile);

    // 配置对齐模式为硬件D2C对齐
    config->setAlignMode(ALIGN_D2C_HW_MODE);

    // 启动在Config中配置的流，如果不传参数，将启动默认配置启动流
    try {
        pipe.start(config);
    }
    catch(ob::Error &e) {
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    }

    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("SyncAlignViewer", colorProfile->width(), colorProfile->height());

    std::thread waitFrameThread([&]() {
        while(!quitApp) {
            // 以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据，
            // 并设置帧的等待超时时间为100ms
            if(started) {
                auto curFrameSet = pipe.waitForFrames(100);
                if(curFrameSet == nullptr) {
                    continue;
                }
                std::unique_lock<std::mutex> lock(framesetMutex, std::defer_lock);
                if(lock.try_lock()) {
                    frameSet = curFrameSet;
                }
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    });

    while(app) {
        ////获取按键事件的键值
        int key = app.getKey();

        // 按+键增加alpha
        if(preKey != key && key == add) {
            alpha += 0.1f;
            if(alpha >= 1.0f) {
                alpha = 1.0f;
            }
        }

        // 按-键减少alpha
        if(preKey != key && key == reduce) {
            alpha -= 0.1f;
            if(alpha <= 0.0f) {
                alpha = 0.0f;
            }
        }
        // 按D键开关硬件D2C
        if(preKey != key && (key == D || key == d)) {
            try {
                if(!hd2c) {
                    started = false;
                    pipe.stop();
                    hd2c = true;
                    sd2c = false;
                    config->setAlignMode(ALIGN_D2C_HW_MODE);
                    pipe.start(config);
                    started = true;
                }
                else {
                    started = false;
                    pipe.stop();
                    hd2c = false;
                    sd2c = false;
                    config->setAlignMode(ALIGN_DISABLE);
                    pipe.start(config);
                    started = true;
                }
            }
            catch(std::exception &e) {
                std::cerr << "Property not support" << std::endl;
            }
        }

        // 按S键开关软件D2C
        if(preKey != key && (key == S || key == s)) {
            try {
                if(!sd2c) {
                    started = false;
                    pipe.stop();
                    sd2c = true;
                    hd2c = false;
                    config->setAlignMode(ALIGN_D2C_SW_MODE);
                    pipe.start(config);
                    started = true;
                }
                else {
                    started = false;
                    pipe.stop();
                    hd2c = false;
                    sd2c = false;
                    config->setAlignMode(ALIGN_DISABLE);
                    pipe.start(config);
                    started = true;
                }
            }
            catch(std::exception &e) {
                std::cerr << "Property not support" << std::endl;
            }
        }

        // 按F键开关同步
        if(preKey != key && (key == F || key == f)) {
            sync = !sync;
            if(sync) {
                try {
                    // 开启同步功能
                    pipe.enableFrameSync();
                }
                catch(...) {
                    std::cerr << "Sync not support" << std::endl;
                }
            }
            else {
                try {
                    // 关闭同步功能
                    pipe.disableFrameSync();
                }
                catch(...) {
                    std::cerr << "Sync not support" << std::endl;
                }
            }
        }

        preKey = key;

        if(frameSet != nullptr) {
            std::unique_lock<std::mutex> lock(framesetMutex);
            // 在窗口中渲染一组帧数据，这里将渲染彩色帧及深度帧，RENDER_OVERLAY表示将彩色帧及
            // 深度帧叠加显示
            auto colorFrame = frameSet->colorFrame();
            auto depthFrame = frameSet->depthFrame();
            if(colorFrame != nullptr && depthFrame != nullptr) {
                app.render({ colorFrame, depthFrame }, alpha);
            }
            frameSet = nullptr;
        }
        else {
            app.render({}, RENDER_SINGLE);
        }

        // 清空帧缓冲队列，减少延时
        //  while (pipe.waitForFrames(10) != nullptr) {};
    }

    quitApp = true;
    waitFrameThread.join();

    // 停止Pipeline，将不再产生帧数据
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
