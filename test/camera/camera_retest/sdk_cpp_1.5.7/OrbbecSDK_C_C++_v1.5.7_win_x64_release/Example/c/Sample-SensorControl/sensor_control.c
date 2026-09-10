#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libobsensor/h/Context.h"
#include "libobsensor/h/Device.h"
#include "libobsensor/h/Error.h"
#include "libobsensor/h/Frame.h"
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/h/Sensor.h"
#include "libobsensor/h/StreamProfile.h"

ob_error *g_error = NULL;

ob_device * select_device(ob_device_list *device_list);
void        get_property_list(ob_device *device, ob_property_item **property_list, int *size);
void        printf_property_list(ob_device *device);
void        set_property_value(ob_device *device, ob_property_item property_item, const char *value);
void        get_property_value(ob_device *device, ob_property_item property_item);
void        check_error();
void        check_control_property_error();
const char *permissionTypeToString(ob_permission_type permission) {
    switch(permission) {
    case OB_PERMISSION_READ:
        return "R/_";
    case OB_PERMISSION_WRITE:
        return "_/W";
    case OB_PERMISSION_READ_WRITE:
        return "R/W";

    default:
        break;
    }
    return "_/_";
}

int main(int argc, char **argv) {
    // 创建一个Context，与Pipeline不同，Context是底层API的入口，在开关流等常用操作上
    // 使用低级会稍微复杂一些，但是底层API可以提供更多灵活的操作，如获取多个设备，读写
    // 设备及相机的属性等
    ob_context *ctx = ob_create_context(&g_error);
    check_error(g_error);

    // 查询已经接入设备的列表
    ob_device_list *dev_list = ob_query_device_list(ctx, &g_error);
    check_error(g_error);

    int isSelectDevice = 1;
    while(isSelectDevice) {
        // 选择一个设备进行操作
        ob_device *device   = NULL;
        int        devCount = ob_device_list_device_count(dev_list, &g_error);
        check_error(g_error);

        if(devCount > 0) {
            // 如果插入单个设备，默认选择第一个
            if(devCount <= 1) {
                device = ob_device_list_get_device(dev_list, 0, &g_error);
                check_error(g_error);
            }
            else {
                device = select_device(dev_list);
            }
            ob_device_info *deviceInfo = ob_device_get_device_info(device, &g_error);
            check_error(g_error);
            const char *name = ob_device_info_name(deviceInfo, &g_error);
            check_error(g_error);
            int pid = ob_device_info_pid(deviceInfo, &g_error);
            check_error(g_error);
            int vid = ob_device_info_vid(deviceInfo, &g_error);
            check_error(g_error);
            const char *uid = ob_device_info_uid(deviceInfo, &g_error);
            check_error(g_error);
            printf("\n------------------------------------------------------------------------\n");
            printf("Current Device: name: %s, pid: %d, vid: %d, uid: %s\n", name, pid, vid, uid);
        }
        else {
            printf("Device Not Found.\n");
            isSelectDevice = 0;
            break;
        }
        printf("Input \"?\" to get all properties.\n");
        int isSelectProperty = 1;
        while(isSelectProperty) {
            char str[100] = { 0 };
            // fgets(str,100, stdin);
            gets(str);

            int  selectId     = 0;
            char setValue[20] = { 0 };
            char command[20]  = { 0 };
            if(strcmp(str, "?") != 0) {

                // 以空格分割三次获取字符串
                char *p = strtok(str, " ");
                if(p) {
                    selectId = atoi(p);
                }
                p = strtok(NULL, " ");
                if(p) {
                    if(strcmp(p, "get") != 0 && strcmp(p, "set") != 0) {
                        printf("Property control usage: [property index] [set] [property value] or [property index] [get]\n");
                        continue;
                    }
                    strcpy(command, p);
                }
                else {
                    printf("Property control usage: [property index] [set] [property value] or [property index] [get]\n");
                    continue;
                }

                p = strtok(NULL, " ");
                if(p) {
                    strcpy(setValue, p);
                }
                else {
                    if(strcmp(command, "set") == 0) {
                        printf("Property control usage: [property index] [set] [property value] or [property index] [get]\n");
                        continue;
                    }
                }

                p = strtok(NULL, " ");
                if(p) {
                    printf("Property control usage: [property index] [set] [property value] or [property index] [get]\n");
                    continue;
                }

                int               size          = 0;
                ob_property_item *property_list = (ob_property_item *)malloc(sizeof(ob_property_item) * 100);
                get_property_list(device, &property_list, &size);
                if(selectId >= size) {
                    printf("Your selection is out of range, please reselect.\n");
                    continue;
                }

                int              isGetValue   = (strcmp(command, "get") == 0) ? 1 : 0;
                ob_property_item propertyItem = property_list[selectId];

                if(isGetValue) {
                    get_property_value(device, propertyItem);
                }
                else {
                    set_property_value(device, propertyItem, setValue);
                }
            }
            else {
                printf_property_list(device);
                printf("Please select property.(Property control usage: [property number] [set/get] [property value])\n");
            }
        }

        // 销毁device
        ob_delete_device(device, &g_error);
        check_error(g_error);
    }

    // 销毁context
    ob_delete_context(ctx, &g_error);
    check_error(g_error);

    // 销毁device list
    ob_delete_device_list(dev_list, &g_error);
    check_error(g_error);

    return 0;
}

// 选择一个设备，这里会打印设备的名称、pid、vid、uid，选择后会创建相应的设备对象
ob_device *select_device(ob_device_list *device_list) {
    ob_error *g_error   = NULL;
    int       dev_count = ob_device_list_device_count(device_list, &g_error);
    check_error(g_error);

    printf("Device list: \n");
    for(int i = 0; i < dev_count; i++) {
        const char *name = ob_device_list_get_device_name(device_list, i, &g_error);
        check_error(g_error);
        int pid = ob_device_list_get_device_pid(device_list, i, &g_error);
        check_error(g_error);
        int vid = ob_device_list_get_device_vid(device_list, i, &g_error);
        check_error(g_error);
        const char *uid = ob_device_list_get_device_uid(device_list, i, &g_error);
        check_error(g_error);
        const char *sn = ob_device_list_get_device_serial_number(device_list, i, &g_error);
        check_error(g_error);
        printf("%d. name: %s, pid: %d, vid: %d, uid: %s, sn: %s\n", i, name, pid, vid, uid, sn);
    }
    printf("Select a device: ");

    int dev_index;
    scanf("%d", &dev_index);

    while(dev_index < 0 || dev_index >= dev_count) {
        printf("Your select is out of range, please reselect: \n");
        scanf("%d", &dev_index);
    }

    ob_device *device = ob_device_list_get_device(device_list, dev_index, &g_error);
    check_error(g_error);

    return device;
}

// 打印支持的属性列表
void printf_property_list(ob_device *device) {
    uint32_t size = ob_device_get_supported_property_count(device, &g_error);
    check_error(g_error);
    if(size == 0) {
        printf("No supported property!\n");
    }
    printf("\n------------------------------------------------------------------------\n");
    int i = 0, index = 0;
    for(i = 0; i < size; i++) {
        ob_property_item property_item = ob_device_get_supported_property(device, i, &g_error);
        check_error(g_error);
        if(property_item.type != OB_STRUCT_PROPERTY && (property_item.permission & OB_PERMISSION_READ)) {
            char str[100] = "";

            ob_int_property_range   int_range;
            ob_float_property_range float_range;
            ob_bool_property_range  bool_range;
            switch(property_item.type) {
            case OB_BOOL_PROPERTY:
                sprintf(str, "Bool value(min:0, max:1, step:1)");
                break;
            case OB_INT_PROPERTY:
                int_range = ob_device_get_int_property_range(device, property_item.id, &g_error);
                check_control_property_error(g_error);
                sprintf(str, "Int value(min:%d, max:%d, step:%d,)", int_range.min, int_range.max, int_range.step);
                break;
            case OB_FLOAT_PROPERTY:
                float_range = ob_device_get_float_property_range(device, property_item.id, &g_error);
                check_control_property_error(g_error);
                sprintf(str, "Float value(min:%f, max:%f, step:%f,)", float_range.min, float_range.max, float_range.step);
                break;
            default:
                break;
            }

            printf("%d: %s, permission=%s, range=%s\n", index, property_item.name, permissionTypeToString(property_item.permission), str);
            index++;
        }
    }
    printf("\n------------------------------------------------------------------------\n");
}

// 获取属性列表
void get_property_list(ob_device *device, ob_property_item **property_list, int *size) {
    uint32_t propertySize = ob_device_get_supported_property_count(device, &g_error);
    check_error(g_error);
    int i     = 0;
    int index = 0;
    for(i = 0; i < propertySize; i++) {
        ob_property_item property_item = ob_device_get_supported_property(device, i, &g_error);
        check_error(g_error);
        if(property_item.type != OB_STRUCT_PROPERTY && property_item.permission != OB_PERMISSION_DENY) {
            (*property_list)[index] = property_item;
            index++;
        }
    }
    (*size) = index;
}

// 设置属性
void set_property_value(ob_device *device, ob_property_item property_item, const char *value) {
    int   bool_value  = 0;
    int   int_value   = 0;
    float float_value = 0.0f;
    switch(property_item.type) {
    case OB_BOOL_PROPERTY:
        bool_value = atoi(value);
        ob_device_set_bool_property(device, property_item.id, bool_value, &g_error);
        check_control_property_error(g_error);
        printf("property name:%s,set bool value:%d\n", property_item.name, bool_value);
        break;
    case OB_INT_PROPERTY:
        int_value = atoi(value);
        ob_device_set_int_property(device, property_item.id, int_value, &g_error);
        check_control_property_error(g_error);
        printf("property name:%s,set int value:%d\n", property_item.name, int_value);
        break;
    case OB_FLOAT_PROPERTY:
        float_value = atoi(value);
        ob_device_set_float_property(device, property_item.id, float_value, &g_error);
        check_control_property_error(g_error);
        printf("property name:%s,set float value:%f\n", property_item.name, float_value);
        break;
    }
}

// 获取属性
void get_property_value(ob_device *device, ob_property_item property_item) {
    int   bool_ret;
    int   int_ret;
    float float_ret;
    switch(property_item.type) {
    case OB_BOOL_PROPERTY:
        bool_ret = ob_device_get_bool_property(device, property_item.id, &g_error);
        check_control_property_error();
        printf("property name:%s,get bool value:%d\n", property_item.name, bool_ret);
        break;
    case OB_INT_PROPERTY:
        int_ret = ob_device_get_int_property(device, property_item.id, &g_error);
        check_control_property_error();
        printf("property name:%s,get bool value:%d\n", property_item.name, int_ret);
        break;
    case OB_FLOAT_PROPERTY:
        float_ret = ob_device_get_float_property(device, property_item.id, &g_error);
        check_control_property_error();
        printf("property name:%s,get bool value:%f\n", property_item.name, float_ret);
        break;
    }
}

void check_error() {
    if(g_error) {
        printf("ob_error was raised: \n\tcall: %s(%s)\n", ob_error_function(g_error), ob_error_args(g_error));
        printf("\tmessage: %s\n", ob_error_message(g_error));
        printf("\terror type: %d\n", ob_error_exception_type(g_error));
        exit(EXIT_FAILURE);
    }
}

void check_control_property_error() {
    if(g_error) {
        printf("ob_error was raised when calling %s(%s):\n", ob_error_function(g_error), ob_error_args(g_error));
        printf("g_error message: %s\n", ob_error_message(g_error));
        printf("g_error Type: %d\n", ob_error_exception_type(g_error));
        printf("Set/Get property failed !\n");
    }
    ob_delete_error(g_error);
    g_error = NULL;
}
