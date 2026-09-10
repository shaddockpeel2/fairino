#include "../window.hpp"

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

extern "C" {
#include <stdlib.h>
#include <libobsensor/h/Error.h>
#include <libobsensor/h/Frame.h>
#include <libobsensor/h/ObTypes.h>
#include <libobsensor/h/Context.h>
#include <libobsensor/h/Pipeline.h>
#include <libobsensor/h/StreamProfile.h>
#include <libobsensor/h/Device.h>
#include <libobsensor/h/Sensor.h>
}

/*
 * 本例程用 C++ 代码编写，基于 OrbbecSDK 的 C 语言版本 API,
 * 用于展示 C 语言版本 API 的 Color 开关流过程。
 */

// 创建全局变量
Window *     win    = nullptr;  // 渲染显示窗，基于 opencv
ob_error *   error  = NULL;     // 用于 SDK 接口错误信息返回
ob_context * ctx    = nullptr;  // context, 用于创建 device
ob_device *  device = nullptr;  // device, 用于创建 pipeline
ob_pipeline *pipe   = nullptr;  // pipeline, 用于连接设备后打开数据流

ob_frame * current_frameset = nullptr;  // 用于渲染显示的帧
std::mutex frameset_mutex;              // frameSet 互斥锁

// 异常处理函数
void check_error(ob_error *error) {
    if(error) {
        printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(error), ob_error_args(error));
        printf("\tmessage: %s\n", ob_error_message(error));
        printf("\terror type: %d\n", ob_error_exception_type(error));
        exit(EXIT_FAILURE);
    }
}

// 数据帧集合回调函数
void frameset_callback(ob_frame *frameset, void *user_data) {
    std::unique_lock<std::mutex> lock(frameset_mutex);
    if(current_frameset != nullptr) {
        // 销毁 frameSet, 回收内存
        ob_delete_frame(current_frameset, &error);
        check_error(error);
    }
    current_frameset = frameset;
}

int main(int argc, char **args) {
    // 创建 context
    ctx = ob_create_context(&error);
    check_error(error);

    // 当前仅 FemtoMega 设备支持网络模式，其默认 ip 为：192.168.1.10
    char ip[16];
    printf("Input your device ip(default: 192.168.1.10):");
    if(fgets(ip, sizeof ip, stdin) != NULL) {
        char *newline = strchr(ip, '\n');
        if(newline)
            *newline = 0;
        if(strlen(ip) == 0) {
            strcpy(ip, "192.168.1.10");
        }
    }

    // 通过 ip 地址及端口号创建网络设备（当前 FemtoMega 端口号为：8090，且不可修改）
    device = ob_create_net_device(ctx, ip, 8090, &error);
    check_error(error);

    // 创建 PipeLine 用于连接设备后打开 Color 流
    pipe = ob_create_pipeline_with_device(device, &error);
    check_error(error);

    // 创建 config，用于配置 Color 流的 分辨率、帧率、格式
    ob_config *config = ob_create_config(&error);
    check_error(error);

    ob_stream_profile *color_profile = nullptr;
    // 获取 Color 流配置列表
    ob_stream_profile_list *color_profiles = ob_pipeline_get_stream_profile_list(pipe, OB_SENSOR_COLOR, &error);
    check_error(error);
    // 选择默认流配置
    color_profile = ob_stream_profile_list_get_profile(color_profiles, 0, &error);
    check_error(error);
    // 使能配置
    ob_config_enable_stream(config, color_profile, &error);
    check_error(error);

    ob_stream_profile *depth_profile = nullptr;
    // 获取 depth 流配置列表
    ob_stream_profile_list *depth_profiles = ob_pipeline_get_stream_profile_list(pipe, OB_SENSOR_DEPTH, &error);
    check_error(error);
    // 选择默认流配置
    depth_profile = ob_stream_profile_list_get_profile(depth_profiles, 0, &error);
    check_error(error);
    // 使能配置
    ob_config_enable_stream(config, depth_profile, &error);
    check_error(error);

    // 通过 config 启动 pipeline
    ob_pipeline_start_with_callback(pipe, config, frameset_callback, nullptr, &error);
    check_error(error);

    // 创建渲染显示窗口
    win = new Window("NetDevice", ob_video_stream_profile_width(color_profile, &error), ob_video_stream_profile_height(color_profile, &error));
    check_error(error);

    // 循环等待，在窗口接收到 "ESC" 按键后退出
    while(*win) {
        ob_frame *frameset_for_render = nullptr;

        {  // 通过大括号定义作用域，lock 退出该作用域后自动解锁。及时解锁可避免 pipeline 的 frameset 输出线程阻塞太久而导致内部缓存增加及数据帧延时增加
            std::unique_lock<std::mutex> lock(frameset_mutex);
            frameset_for_render = current_frameset;
            current_frameset    = nullptr;
        }

        if(frameset_for_render != nullptr) {
            ob_frame *color_frame = ob_frameset_color_frame(frameset_for_render, &error);
            check_error(error);

            ob_frame *depth_frame = ob_frameset_depth_frame(frameset_for_render, &error);
            check_error(error);

            if(color_frame != nullptr && depth_frame != nullptr) {
                ob_format color_format = ob_frame_format(color_frame, &error);
                check_error(error);
                if(color_format == OB_FORMAT_H264 || color_format == OB_FORMAT_H265) {
                    // 对于 H264 和 H265 格式图像帧，用户可参考 FFmpeg 等解码库及其例程完成解码和渲染显示，本例为保持工程及代码简洁，不做展示。
                    uint32_t index = ob_frame_index(color_frame, &error);
                    check_error(error);
                    if(index % 30 == 0) {
                        // 每 30 帧打印一次 Color 数据帧信息
                        uint64_t tsp = ob_frame_time_stamp(color_frame, &error);
                        check_error(error);
                        printf("Color frame: index=%d, timestamp=%lld", index, tsp);
                    }
                    win->render({ depth_frame }, RENDER_ONE_ROW);
                }
                else {
                    win->render({ color_frame, depth_frame }, RENDER_ONE_ROW);
                }
            }

            // 销毁 color frame
            if(color_frame != nullptr) {
                ob_delete_frame(color_frame, &error);
                check_error(error);
            }

            // 销毁 depth frame
            if(depth_frame != nullptr) {
                ob_delete_frame(depth_frame, &error);
                check_error(error);
            }

            // 销毁 frameset
            ob_delete_frame(frameset_for_render, &error);
            check_error(error);
        }

        // 休眠 30ms，控制渲染显示刷新率
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    };

    // 停止 pipeline
    ob_pipeline_stop(pipe, &error);
    check_error(error);

    // 销毁窗口
    delete win;

    // 销毁 frameSet， 回收内存
    if(current_frameset != nullptr) {
        ob_delete_frame(current_frameset, &error);
        check_error(error);
        current_frameset = nullptr;
    }

    // 销毁 color profile
    ob_delete_stream_profile(color_profile, &error);
    check_error(error);

    // 销毁 color profile list
    ob_delete_stream_profile_list(color_profiles, &error);
    check_error(error);

    // 销毁 depth profile
    ob_delete_stream_profile(depth_profile, &error);
    check_error(error);

    // 销毁 depth profile list
    ob_delete_stream_profile_list(depth_profiles, &error);
    check_error(error);

    // 销毁 device
    ob_delete_device(device, &error);
    check_error(error);

    // 销毁 pipeline
    ob_delete_pipeline(pipe, &error);
    check_error(error);

    // 销毁 context
    ob_delete_context(ctx, &error);
    check_error(error);

    return 0;
}
