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
#include <libobsensor/h/Pipeline.h>
#include <libobsensor/h/StreamProfile.h>
#include <libobsensor/h/Device.h>
#include <libobsensor/h/Sensor.h>
}

/*
 * 本例程用C++代码编写，基于OrbbecSDK的C语言版本API,
 * 用于展示C语言版本API的Color开关流过程。
 */

// 创建全局变量
Window *     win    = nullptr;  // 渲染显示窗，基于opencv
ob_error *   error  = NULL;     // 用于SDK 接口错误信息返回
ob_pipeline *pipe   = nullptr;  // pipeline, 用于连接设备后打开Color流
ob_device *  device = nullptr;  // device, 通过pipeline获取，并且可通过device获取对应的sensor

ob_frame *              frameset_for_render = nullptr;  // 用于渲染显示的帧
bool                    frameset_ready      = false;    // 用于渲染显示的帧是否已抓取到
bool                    thread_exit         = false;    // 线程是否需要退出
std::mutex              frameset_mutex;                 // frameSet互斥锁
std::condition_variable frameset_proc_cv;               // frameSet处理条件变量

void check_error(ob_error *error) {
    if(error) {
        printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(error), ob_error_args(error));
        printf("\tmessage: %s\n", ob_error_message(error));
        printf("\terror type: %d\n", ob_error_exception_type(error));
        exit(EXIT_FAILURE);
    }
}

// 处理和渲染显示frameSet
void frameset_proc_func() {
    while(!thread_exit) {
        std::unique_lock<std::mutex> lk(frameset_mutex);
        // 等待frameSet已准备好或线程退出信号
        frameset_proc_cv.wait(lk, [&]() { return frameset_ready || thread_exit; });
        if(!thread_exit && frameset_ready) {
            ob_frame *color_frame = ob_frameset_color_frame(frameset_for_render, &error);
            check_error(error);

            if(color_frame != nullptr) {
                std::vector<ob_frame *> frames = { color_frame };
                win->render(frames, RENDER_ONE_ROW);
            }

            // 销毁frameSet, 回收内存
            ob_delete_frame(frameset_for_render, &error);
            check_error(error);
            frameset_for_render = nullptr;

            if(color_frame != nullptr) {
                // 销毁frame, 回收内存
                ob_delete_frame(color_frame, &error);
                check_error(error);
            }

            // 复位标记
            frameset_ready = false;
        }
    }
}

// 抓取frameSet
void frameset_grab_func() {
    while(!thread_exit) {

        ob_frame *frameset = nullptr;
        frameset           = ob_pipeline_wait_for_frameset(pipe, 100, &error);
        check_error(error);

        if(frameset != nullptr) {
            if(!frameset_ready) {
                std::unique_lock<std::mutex> lk(frameset_mutex);
                frameset_for_render = frameset;
                frameset_ready      = true;
                frameset_proc_cv.notify_all();
            }
            else {
                // frameset_proc_func 未处理完此前获取的frameSet，丢弃此次获取的frameSet
                // 如果持续等待frameset_procFunc处理完此前获取的frameSet，会导致SDK缓存
                // 过多frameSet，进而导致内存占用过大和延迟过高.
                // 销毁frameSet 回收内存
                ob_delete_frame(frameset, &error);
                check_error(error);
            }
        }
    }
}

int main(int argc, char **args) {
    // 创建PipeLine 用于连接设备后打开Color流
    pipe = ob_create_pipeline(&error);
    check_error(error);

    // 创建config，用于配置 Color流的 分辨率、帧率、格式
    ob_config *config = ob_create_config(&error);
    check_error(error);

    // 配置Color流
    ob_stream_profile *     color_profile = nullptr;
    ob_stream_profile_list *profiles      = ob_pipeline_get_stream_profile_list(pipe, OB_SENSOR_COLOR, &error);
    if(error) {
        printf("Current device is not support color sensor!\n");
        exit(EXIT_FAILURE);
    }

    // 根据指定的格式查找对应的Profile,优先选择RGB888格式
    color_profile = ob_stream_profile_list_get_video_stream_profile(profiles, 1280, 0, OB_FORMAT_RGB, 30, &error);
    // 没找到指定格式后查找默认的Profile进行开流
    if(error) {
        color_profile = ob_stream_profile_list_get_profile(profiles, 0, &error);
        error         = nullptr;
    }

    // 使能配置
    ob_config_enable_stream(config, color_profile, &error);
    check_error(error);

    // 通过Pipeline获取Device
    device = ob_pipeline_get_device(pipe, &error);
    check_error(error);

    // 获取镜像属性是否有可写的权限
    ob_device_is_property_supported(device, OB_PROP_COLOR_MIRROR_BOOL, OB_PERMISSION_WRITE, &error);
    if(error && error->status == OB_STATUS_OK) {
        // 设置镜像
        ob_device_set_bool_property(device, OB_PROP_COLOR_MIRROR_BOOL, true, &error);
        check_error(error);
    }
    check_error(error);

    // 通过config 启动pipeline
    ob_pipeline_start_with_config(pipe, config, &error);
    check_error(error);

    // 创建渲染显示窗口
    win = new Window("ColorViewer", ob_video_stream_profile_width(color_profile, &error), ob_video_stream_profile_height(color_profile, &error));
    check_error(error);

    // 启动frameSet抓取和处理线程
    thread_exit    = false;
    frameset_ready = false;
    std::thread frameset_grab_thread(frameset_grab_func);
    std::thread frameset_proc_thread(frameset_proc_func);

    // 循环等待，在窗口接收到"ESC"按键后退出
    while(*win) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    // 通知结束frameSet抓取和处理线程
    thread_exit = true;
    frameset_proc_cv.notify_all();
    if(frameset_grab_thread.joinable()) {
        frameset_grab_thread.join();
    }
    if(frameset_proc_thread.joinable()) {
        frameset_proc_thread.join();
    }

    // 停止pipeline
    ob_pipeline_stop(pipe, &error);
    check_error(error);

    // 销毁窗口
    delete win;
    if(frameset_for_render != nullptr) {
        // 销毁frameSet， 回收内存
        ob_delete_frame(frameset_for_render, &error);
        check_error(error);
        frameset_for_render = nullptr;
    }

    // 销毁profile
    ob_delete_stream_profile(color_profile, &error);
    check_error(error);

    // 销毁profile list
    ob_delete_stream_profile_list(profiles, &error);
    check_error(error);

    // 销毁device
    ob_delete_device(device, &error);
    check_error(error);

    // 销毁pipeline
    ob_delete_pipeline(pipe, &error);
    check_error(error);

    return 0;
}
