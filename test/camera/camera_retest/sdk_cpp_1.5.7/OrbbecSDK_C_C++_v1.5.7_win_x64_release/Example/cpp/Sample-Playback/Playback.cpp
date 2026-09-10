// #include "./RecorderPlaybackWindow.hpp"
#include "../window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/RecordPlayback.hpp"
#include <thread>
#include <mutex>

#define LOW_API 0

uint64_t lastFrameTimestamp = 0;

int main(int argc, char **argv) try {
#if LOW_API
    //使用回放文件创建回放对象
    ob::Playback playback("./Orbbec.bag");

    //设置回放状态回调
    playback.setPlaybackStateCallback([&](OBMediaState state) {
        if(state == OB_MEDIA_BEGIN) {
            std::cout << "Playback file begin." << std::endl;
        }
        else if(state == OB_MEDIA_END) {
            std::cout << "Playback file end." << std::endl;
        }
    });

    //从回放文件中读取设备信息
    auto deviceInfo = playback.getDeviceInfo();
    std::cout << "======================DeviceInfo: name : " << deviceInfo->name() << " sn: " << deviceInfo->serialNumber()
              << " firmware: " << deviceInfo->firmwareVersion() << " vid: " << deviceInfo->vid() << " pid: " << deviceInfo->pid() << std::endl;

    //从回放文件中读取内参信息
    auto cameraParam = playback.getCameraParam();
    std::cout << "======================Camera params : rgb width:" << cameraParam.rgbIntrinsic.width << " rgb height: " << cameraParam.rgbIntrinsic.height
              << " depth width: " << cameraParam.depthIntrinsic.width << " depth height: " << cameraParam.rgbIntrinsic.height << std::endl;

    Window app("Playback", 1280, 480);

    //帧数据回调
    ob::PlaybackCallback callback = ([&](std::shared_ptr<ob::Frame> frame, void *pCookies) {
        auto currentTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        lastFrameTimestamp    = currentTimestamp;
        app.render({ frame }, RENDER_ONE_ROW);
    });

    //开始回放
    playback.start(callback);

    while(app) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    //停止回放
    playback.stop();
#else
    //创建用于回放的pipeline对象
    ob::Pipeline pipe("./OrbbecPipeline.bag");

    //获取回放对象设置回放状态回调
    auto playback = pipe.getPlayback();
    playback->setPlaybackStateCallback([&](OBMediaState state) {
        if(state == OB_MEDIA_BEGIN) {
            std::cout << "Playback file begin." << std::endl;
        }
        else if(state == OB_MEDIA_END) {
            std::cout << "Playback file end." << std::endl;
        }
    });

    //从回放文件中读取设备信息
    auto deviceInfo = playback->getDeviceInfo();
    std::cout << "======================DeviceInfo: name : " << deviceInfo->name() << " sn: " << deviceInfo->serialNumber()
              << " firmware: " << deviceInfo->firmwareVersion() << " vid: " << deviceInfo->vid() << " pid: " << deviceInfo->pid() << std::endl;

    //从回放文件中读取内参信息
    auto cameraParam = pipe.getCameraParam();
    std::cout << "======================Camera params : rgb width:" << cameraParam.rgbIntrinsic.width << " rgb height: " << cameraParam.rgbIntrinsic.height
              << " depth width: " << cameraParam.depthIntrinsic.width << " depth height: " << cameraParam.rgbIntrinsic.height << std::endl;

    //开启回放
    pipe.start(NULL);
    Window app("Playback", 640, 480);

    while(app) {
        //以阻塞的方式等待一帧数据，
        //并设置帧的等待超时时间为100ms
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            app.render({}, RENDER_ONE_ROW);
            continue;
        }

        app.render({ frameSet->depthFrame() }, RENDER_ONE_ROW);
    }

    //停止回放
    pipe.stop();
#endif

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
