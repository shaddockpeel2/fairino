#include "../window.hpp"

#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"

int main(int argc, char **argv) try {
    //创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭
    //多种类型的流并获取一组帧数据
    ob::Pipeline pipe;

    //获取红外相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    auto profiles = pipe.getStreamProfileList(OB_SENSOR_IR);

    std::shared_ptr<ob::VideoStreamProfile> irProfile = nullptr;
    try {
        //根据指定的格式查找对应的Profile,优先查找Y16格式
        irProfile = profiles->getVideoStreamProfile(640, 0, OB_FORMAT_UNKNOWN, 30);
    }
    catch(ob::Error &e) {
        //没找到指定格式后查找默认的Profile进行开流
        irProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(0))->as<ob::VideoStreamProfile>();
    }

    //通过创建Config来配置Pipeline要启用或者禁用哪些流，这里将启用红外流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(irProfile);

    //判断是否支持切换左右IR通道
    if (pipe.getDevice()->isPropertySupported(OB_PROP_IR_CHANNEL_DATA_SOURCE_INT, OB_PERMISSION_READ_WRITE)) {
        // Gemini2产品支持SENSOR_IR选择sensor输出，0是左IR，1是右IR。
        int32_t dataSource = 0;
        pipe.getDevice()->setIntProperty(OB_PROP_IR_CHANNEL_DATA_SOURCE_INT, dataSource);
    }

    //启动在Config中配置的流，如果不传参数，将启动默认配置启动流
    pipe.start(config);

    //创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("InfraredViewer", irProfile->width(), irProfile->height());

    //获取镜像属性是否有可写的权限
    if(pipe.getDevice()->isPropertySupported(OB_PROP_IR_MIRROR_BOOL, OB_PERMISSION_WRITE)) {
        //设置镜像
        pipe.getDevice()->setBoolProperty(OB_PROP_IR_MIRROR_BOOL, true);
    }

    while(app) {
        //以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据，
        //并设置帧的等待超时时间为100ms
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        //在窗口中渲染一组帧数据，这里只渲染红外帧，但是也要以数组的形式传入，RENDER_SINGLE表示
        //只渲染数组中的第一个帧
        app.render({ frameSet->irFrame() }, RENDER_SINGLE);
    }

    //停止Pipeline，将不再产生帧数据
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
