// #include "./RecorderPlaybackWindow.hpp"
#include "../window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/RecordPlayback.hpp"
#include "libobsensor/hpp/Filter.hpp"

#define LOW_API 0

int main(int argc, char **argv) try {

#if LOW_API
    // 创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭
    // 多种类型的流并获取一组帧数据
    ob::Pipeline pipe;

    // 获取对应流配置
    auto profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    // 根据指定的格式查找对应的Profile,优先查找Y16格式
    auto depthProfile = profiles->getVideoStreamProfile(640, 0, OB_FORMAT_Y16, 30);
    // 没找到Y16格式后不匹配格式查找对应的Profile进行开流
    if(!depthProfile) {
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(0))->as<ob::VideoStreamProfile>();
    }

    // 通过创建Config来配置Pipeline要启用或者禁用哪些流，这里将启用深度流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(depthProfile);

    // 启动在Config中配置的流，如果不传参数，将启动默认配置启动流
    pipe.start(config);

    // 创建录制器对象
    ob::Recorder recorder(pipe.getDevice());
    recorder.start("./Orbbec.bag");
    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("Recorder", 1280, 480);

    while(app) {
        // 以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据，
        // 并设置帧的等待超时时间为100ms
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        // 向录制器对象中写入帧数据
        recorder.write(frameSet);
        std::vector<std::shared_ptr<ob::Frame>> frames;
        if(frameSet->depthFrame()) {
            frames.push_back(frameSet->depthFrame());
        }
        app.render(frames, RENDER_ONE_ROW);
    }

    // 停止录制器
    recorder.stop();
    // 停止Pipeline，将不再产生帧数据
    pipe.stop();
#else
    // 创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭
    // 多种类型的流并获取一组帧数据
    ob::Pipeline pipe;

    // 获取深度流配置
    auto                                    profiles     = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
    if(profiles) {
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(0))->as<ob::VideoStreamProfile>();
    }

    // 配置Pipeline打开的流类型
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(depthProfile);

    // 启动在Config中配置的流，如果不传参数，将启动默认配置启动流
    pipe.start(config);
    pipe.startRecord("./OrbbecPipeline.bag");

    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("Recorder", 640, 480);
    while(app) {
        // 以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据，
        // 并设置帧的等待超时时间为100ms
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }
        auto depthFrame = frameSet->depthFrame();
        app.render({ depthFrame }, RENDER_ONE_ROW);
    }

    pipe.stopRecord();
    // 停止Pipeline，将不再产生帧数据
    pipe.stop();
#endif

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
