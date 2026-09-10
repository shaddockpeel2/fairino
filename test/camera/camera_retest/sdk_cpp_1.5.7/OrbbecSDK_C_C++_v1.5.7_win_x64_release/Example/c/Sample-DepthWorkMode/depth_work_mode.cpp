#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <libobsensor/ObSensor.h>
#include <string.h>

#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif

#define ESC 27

void check_error(ob_error *error) {
    if(error) {
        printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(error), ob_error_args(error));
        printf("\tmessage: %s\n", ob_error_message(error));
        printf("\terror type: %d\n", ob_error_exception_type(error));
        exit(EXIT_FAILURE);
    }
}

static char* toHexText(const uint8_t *data, const uint32_t dataLen) {
    static char alpha[] = "0123456789abcdef";
    char *      hexStr  = new char[dataLen * 2 + 1]{ 0 };
    for(uint32_t i = 0; i < dataLen; i++) {
        uint8_t val       = *(data + i);
        *(hexStr + i)     = alpha[(val >> 4) & 0xf];
        *(hexStr + i + 1) = alpha[val & 0xf];
    }

    return hexStr;
}

int main(int argc, char **argv) {

    //打印SDK版本号
    printf("SDK version: %d.%d.%d\n", ob_get_major_version(), ob_get_minor_version(), ob_get_patch_version());

    ob_error *  error = NULL;

    //创建PipeLine 用于连接设备后打开Depth流
    ob_pipeline *pipe = ob_create_pipeline(&error);
    check_error(error);

    //通过pipeline获取device
    ob_device *dev = ob_pipeline_get_device(pipe, &error);
    check_error(error);

    // 检查是否支持相机深度工作模式，目前（2022年12月5日）仅Gemini2双目摄像头支持深度工作模式
    if(!ob_device_is_property_supported(dev, OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE, &error)) {
        printf("FAILED!!, Device not support depth work mode");
        check_error(error);
        return -1;
    }
    check_error(error);

    // 查询当前的深度工作模式
    ob_depth_work_mode cur_work_mode = ob_device_get_current_depth_work_mode(dev, &error);
    check_error(error);
    printf("Current depth work mode: %s\n", cur_work_mode.name);

    // 查询相机支持的深度工作模式列表
    ob_depth_work_mode_list *mode_list = ob_device_get_depth_work_mode_list(dev, &error);
    check_error(error);

    // 获取列表长度
    uint32_t mode_count = ob_depth_work_mode_list_count(mode_list, &error);
    printf("Support depth work mode list count: %u\n", mode_count);

    int cur_mode_index = -1;
    for(uint32_t i = 0; i < mode_count; i++) {
        ob_depth_work_mode mode = ob_depth_work_mode_list_get_item(mode_list, i, &error);
        check_error(error);

        char *hex = toHexText(mode.checksum, sizeof(mode.checksum));
        printf("depth work mode[%u], name: %s, checksum: %s", i, mode.name, hex);
        delete []hex;
        if(0 == strcmp(cur_work_mode.name, mode.name)) {
            printf(" (Current Work Mode)");
        }
        printf("\n");
    }

    int mode_index;
    printf("Please select a work mode index:");
    scanf("%d", &mode_index);
    if(mode_index >= 0 && mode_index < mode_count) {
        ob_depth_work_mode mode = ob_depth_work_mode_list_get_item(mode_list, mode_index, &error);
        check_error(error);

        // 切换到新的相机深度模式
        ob_device_switch_depth_work_mode_by_name(dev, mode.name, &error);
        check_error(error);

        ob_depth_work_mode tmp_mode = ob_device_get_current_depth_work_mode(dev, &error);
        check_error(error);
        if(0 == strcmp(mode.name, tmp_mode.name)) {
            printf("Switch depth work mode success! mode name: %s\n", mode.name);
        }
        else {
            printf("Switch depth work mode failed!");
        }
    } else {
        printf("switchDepthMode faild. invalid index: %d\n", mode_index);
    }

    // 到此切换相机深度模式结束，可以用pipeline进行打开相机取流
    // 注意： 
    // 1. 如果需要切换相机深度模式，那么打开数据流必须在切换深度工作模式之后；每个相机深度模式下支持的有效分辨率不同
    // 2. 如果已经用pipeline打开数据流，那么切花相机深度工作模式前必须把原来申请的pipeline释放； 
    //    切换相机深度工作模式后重新创建pipeline，否则会造成野指针或者内存泄露；

    printf("Press ESC to exit! \n");

    while(true) {
        //获取按下的按键的值，如果是ESC键，退出程序
        int key = getch();
        if(key == ESC)
            break;
    }

    // 销毁mode_list
    ob_delete_depth_work_mode_list(mode_list, &error);
    check_error(error);

    //销毁device
    ob_delete_device(dev, &error);
    check_error(error);

    //销毁pipeline
    ob_delete_pipeline(pipe, &error);
    check_error(error);
    

    return 0;
}
