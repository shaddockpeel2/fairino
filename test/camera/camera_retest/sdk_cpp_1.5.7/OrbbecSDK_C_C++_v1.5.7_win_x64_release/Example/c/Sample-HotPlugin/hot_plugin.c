#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <libobsensor/h/ObTypes.h>
#include <libobsensor/h/Frame.h>
#include <libobsensor/h/Pipeline.h>
#include <libobsensor/h/StreamProfile.h>
#include <libobsensor/h/Error.h>
#include <libobsensor/h/Device.h>
#include <libobsensor/h/Context.h>

#define ESC 27
ob_pipeline *pipeline = NULL;
ob_error *   error    = NULL;
ob_config *  config   = NULL;
bool         isExit   = false;

void check_error(ob_error *error) {
    if(error) {
        printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(error), ob_error_args(error));
        printf("\tmessage: %s\n", ob_error_message(error));
        printf("\terror type: %d\n", ob_error_exception_type(error));
        exit(EXIT_FAILURE);
    }
}

void create_and_start_with_config() {
    //按配置文件的流配置启动流，如果没有配置文件，将使用第0个流配置启动流
    ob_pipeline_start_with_config(pipeline, NULL, &error);
    check_error(error);
}

//设备连接回调
void device_connect_callback(ob_device_list *connectList) {
    uint32_t count = ob_device_list_device_count(connectList, &error);
    check_error(error);
    printf("Device connect: %d\n", count);
    if(count > 0) {
        if(pipeline == NULL) {
            pipeline = ob_create_pipeline(&error);
            check_error(error);
            create_and_start_with_config();
        }
    }
}

//设备断开回调
void device_disconnect_callback(ob_device_list *disconnectList) {
    uint32_t count = ob_device_list_device_count(disconnectList, &error);
    check_error(error);
    printf("Device disconnect: %d\n", count);
    if(count > 0) {
        isExit = true;
    }
}

//设备状态改变回调
void on_device_changed_callback(ob_device_list *removed, ob_device_list *added, void *pCallback) {
    device_disconnect_callback(removed);
    device_connect_callback(added);

    //需要手动销毁设备列表
    //销毁device list
    ob_delete_device_list(removed, &error);
    check_error(error);

    //销毁device list
    ob_delete_device_list(added, &error);
    check_error(error);
}

int main(int argc, char **argv) {

    //创建上下文
    ob_context *ctx = ob_create_context(&error);
    check_error(error);

    //注册设备回调
    ob_set_device_changed_callback(ctx, on_device_changed_callback, NULL, &error);
    check_error(error);

    //查询设备列表
    ob_device_list *dev_list = ob_query_device_list(ctx, &error);
    check_error(error);

    //获取设备数量
    uint32_t dev_count = ob_device_list_device_count(dev_list, &error);
    check_error(error);

    if(dev_count > 0) {
        if(pipeline == NULL) {
            //创建pipeline
            pipeline = ob_create_pipeline(&error);
            check_error(error);
            create_and_start_with_config();
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
                    ob_device *device = ob_pipeline_get_device(pipeline, &error);
                    check_error(error);
                    ob_device_reboot(device, &error);
                    check_error(error);
                    ob_delete_device(device, &error);
                    check_error(error);
                }
            }
        }
        if(pipeline) {
            //等待一帧数据，超时时间为100ms
            ob_frame *frameset = ob_pipeline_wait_for_frameset(pipeline, 100, &error);
            check_error(error);

            if(frameset) {
                //获取深度数据帧
                ob_frame *depth_frame = ob_frameset_depth_frame(frameset, &error);
                check_error(error);

                if(depth_frame) {
                    ob_frame_type type = ob_frame_get_type(depth_frame, &error);
                    check_error(error);
                    uint32_t index = ob_frame_index(depth_frame, &error);
                    check_error(error);
                    uint32_t width = ob_frame_width(depth_frame, &error);
                    check_error(error);
                    uint32_t height = ob_frame_height(depth_frame, &error);
                    check_error(error);
                    ob_format format = ob_frame_format(depth_frame, &error);
                    check_error(error);
                    uint64_t timestampUs = ob_frame_time_stamp_us(depth_frame, &error);
                    check_error(error);
                    float scale = ob_depth_frame_get_value_scale(depth_frame, &error);
                    check_error(error);
                    uint16_t *data = (uint16_t *)ob_frame_data(depth_frame, &error);
                    check_error(error);
                    uint16_t pixelValue = *(data + width * height / 2 + width / 2);

                    printf("=====Depth Frame Info====== FrameType:%d, index:%d, width:%d, height:%d, format:%d, timeStampUs:%lldus, centerDepth:%dmm\n", type,
                           index, width, height, format, timestampUs, (uint32_t)(pixelValue * scale));

                    //销毁深度数据帧
                    ob_delete_frame(depth_frame, &error);
                    check_error(error);
                }

                //获取Color数据帧
                ob_frame *color_frame = ob_frameset_color_frame(frameset, &error);
                check_error(error);

                if(color_frame) {
                    ob_frame_type type = ob_frame_get_type(color_frame, &error);
                    check_error(error);
                    uint32_t index = ob_frame_index(color_frame, &error);
                    check_error(error);
                    uint32_t width = ob_frame_width(color_frame, &error);
                    check_error(error);
                    uint32_t height = ob_frame_height(color_frame, &error);
                    check_error(error);
                    ob_format format = ob_frame_format(color_frame, &error);
                    check_error(error);
                    uint64_t timestampUs = ob_frame_time_stamp_us(color_frame, &error);
                    check_error(error);

                    printf("=====Color Frame Info====== FrameType:%d, index:%d, width:%d, height:%d, format:%d, timeStampUs:%lldus\n", type, index, width,
                           height, format, timestampUs);

                    //销毁Color数据帧
                    ob_delete_frame(color_frame, &error);
                    check_error(error);
                }

                //销毁frameSet
                ob_delete_frame(frameset, &error);
                check_error(error);
            }

            if(isExit) {
                //销毁pipeline
                ob_pipeline_stop(pipeline, &error);
                check_error(error);
                ob_delete_pipeline(pipeline, &error);
                check_error(error);
                pipeline = NULL;
                isExit   = false;
            }
        }
    }

    if(pipeline) {
        //停止pipeline
        ob_pipeline_stop(pipeline, &error);
        check_error(error);

        //销毁pipeline
        ob_delete_pipeline(pipeline, &error);
        check_error(error);
    }

    //销毁device list
    if(dev_list) {
        ob_delete_device_list(dev_list, &error);
        check_error(error);
    }
    //销毁context
    if(ctx) {
        ob_delete_context(ctx, &error);
        check_error(error);
    }
}
