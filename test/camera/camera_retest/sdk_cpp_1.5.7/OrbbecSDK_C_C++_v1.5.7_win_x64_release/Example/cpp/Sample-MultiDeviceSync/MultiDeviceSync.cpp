#include "../window.hpp"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

#include "cJSON.h"

#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>

#define MAX_DEVICE_COUNT 9
#define CONFIG_FILE "./MultiDeviceSyncConfig.json"

typedef struct DeviceConfigInfo_t {
    std::string        deviceSN;
    OBDeviceSyncConfig syncConfig;
} DeviceConfigInfo;

std::mutex                                    frameMutex;
std::map<uint8_t, std::shared_ptr<ob::Frame>> colorFrames;
std::map<uint8_t, std::shared_ptr<ob::Frame>> depthFrames;

std::vector<std::shared_ptr<ob::Device>>       streamDevList;
std::vector<std::shared_ptr<ob::Device>>       configDevList;
std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;

std::condition_variable                      waitRebootCompleteCondition;
std::mutex                                   rebootingDevInfoListMutex;
std::vector<std::shared_ptr<ob::DeviceInfo>> rebootingDevInfoList;

OBSyncMode  textToOBSyncMode(const char *text);
std::string readFileContent(const char *filePath);
bool        loadConfigFile();
int         configMultiDeviceSync();
int         testMultiDeviceSync();
bool        checkDevicesWithDeviceConfigs(const std::vector<std::shared_ptr<ob::Device>> &deviceList);

void startStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType, int deviceIndexBase = 0);
void stopStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType);

void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame);
void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame);

ob::Context context;

void wait_any_key() {
    system("pause");
}

int main(int argc, char **argv) {
    std::cout << "Please select options: " << std::endl;
    std::cout << " 0 --> config devices" << std::endl;
    std::cout << " 1 --> start stream" << std::endl;
    std::cout << "input: ";
    int index = -1;
    std::cin >> index;
    std::cout << std::endl;

    int exitValue = -1;
    if(index == 0) {
        exitValue = configMultiDeviceSync();
        // 只有配置成功了才允许继续后续测试
        if(exitValue == 0) {
            exitValue = testMultiDeviceSync();
        }
    }
    else if(index == 1) {
        exitValue = testMultiDeviceSync();
    }
    else {
        std::cout << "invalid index. " << std::endl;
        std::cout << "Please press any key to exit" << std::endl;
    }

    if(exitValue != 0) {
        wait_any_key();
    }

    return exitValue;
}

int configMultiDeviceSync() try {
    if(!loadConfigFile()) {
        std::cout << "load config failed" << std::endl;
        return -1;
    }

    if(deviceConfigList.empty()) {
        std::cout << "DeviceConfigList is empty. please check config file: " << CONFIG_FILE << std::endl;
        return -1;
    }

    // 查询已经接入设备的列表
    auto devList = context.queryDeviceList();

    // 获取接入设备的数量
    int devCount = devList->deviceCount();
    for(int i = 0; i < devCount; i++) {
        configDevList.push_back(devList->getDevice(i));
    }

    if(configDevList.empty()) {
        std::cerr << "Device list is empty. please check device connection state" << std::endl;
        return -1;
    }

    // 向设备写入配置
    for(auto config: deviceConfigList) {
        auto findItr = std::find_if(configDevList.begin(), configDevList.end(), [config](std::shared_ptr<ob::Device> device) {
            auto serialNumber = device->getDeviceInfo()->serialNumber();
            auto cmpSize      = (std::min)(strlen(serialNumber), config->deviceSN.size());
            return strncmp(serialNumber, config->deviceSN.c_str(), cmpSize) == 0;
        });
        if(findItr != configDevList.end()) {
            auto curConfig = (*findItr)->getSyncConfig();

            // 将配置文件的配置项更新，其他项保留原有配置
            curConfig.syncMode                    = config->syncConfig.syncMode;
            curConfig.irTriggerSignalInDelay      = config->syncConfig.irTriggerSignalInDelay;
            curConfig.rgbTriggerSignalInDelay     = config->syncConfig.rgbTriggerSignalInDelay;
            curConfig.deviceTriggerSignalOutDelay = config->syncConfig.deviceTriggerSignalOutDelay;
            curConfig.deviceId                    = config->syncConfig.deviceId;

            (*findItr)->setSyncConfig(curConfig);
        }
    }

    // 重启设备
    for(auto device: configDevList) {
        rebootingDevInfoList.push_back(device->getDeviceInfo());
        std::cout << "Device sn[" << std::string(device->getDeviceInfo()->serialNumber()) << "] is configured, rebooting..." << std::endl;
        try {
            device->reboot();
        }
        catch(ob::Error) {
            std::cout << "Device sn[" << std::string(device->getDeviceInfo()->serialNumber()) << "] is not configured, skipping...";
            // 部分型号设备的早期固件版本，在接收到重启命令后会立即重启，导致SDK接收不到命令请求的响应而抛异常
        }
    }
    configDevList.clear();

    // 注册设备变化监听，用于辅助监听设备重启后重新连接
    context.setDeviceChangedCallback([&](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
        if(addedList && addedList->deviceCount() > 0) {
            auto deviceCount = addedList->deviceCount();
            for(uint32_t i = 0; i < deviceCount; i++) {
                auto device     = addedList->getDevice(i);
                auto deviceInfo = device->getDeviceInfo();
                std::cout << "addedList sn: " << std::string(deviceInfo->serialNumber()) << std::endl;
                auto findItr = std::find_if(rebootingDevInfoList.begin(), rebootingDevInfoList.end(), [&deviceInfo](std::shared_ptr<ob::DeviceInfo> tmp) {
                    return strcmp(tmp->serialNumber(), deviceInfo->serialNumber()) == 0;
                });

                std::lock_guard<std::mutex> lk(rebootingDevInfoListMutex);
                if(findItr != rebootingDevInfoList.end()) {
                    rebootingDevInfoList.erase(findItr);
                    std::cout << "Device sn[" << std::string(deviceInfo->serialNumber()) << "] reboot complete." << std::endl;

                    if(rebootingDevInfoList.empty()) {
                        waitRebootCompleteCondition.notify_all();
                    }
                }
            }
        }
    });

    // 等待设备重启
    {
        // 等待60s
        std::unique_lock<std::mutex> lk(rebootingDevInfoListMutex);
        waitRebootCompleteCondition.wait_for(lk, std::chrono::milliseconds(60000), [&]() { return rebootingDevInfoList.empty(); });

        // 设备重启失败
        if(!rebootingDevInfoList.empty()) {
            std::cerr << "Device not found after reboot. not found deviceCount: " << rebootingDevInfoList.size() << std::endl;
            for(auto devInfo: rebootingDevInfoList) {
                std::cout << "not found deviceSN: " << std::string(devInfo->serialNumber()) << std::endl;
            }
            return -1;
        }

        // 重启成功
        std::cout << "All device update config and reboot complete." << std::endl;
    }

    // 注销回调，避免影响接下来的测试多机同步
    context.setDeviceChangedCallback(nullptr);

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    wait_any_key();
    exit(EXIT_FAILURE);
}

int testMultiDeviceSync() try {
    streamDevList.clear();
    // 查询已经接入设备的列表
    auto devList = context.queryDeviceList();

    // 获取接入设备的数量
    int devCount = devList->deviceCount();
    for(int i = 0; i < devCount; i++) {
        streamDevList.push_back(devList->getDevice(i));
    }

    if(streamDevList.empty()) {
        std::cerr << "Device list is empty. please check device connection state" << std::endl;
        return -1;
    }

    // 遍历设备列表并创建设备
    std::vector<std::shared_ptr<ob::Device>> primary_devices;
    std::vector<std::shared_ptr<ob::Device>> secondary_devices;
    for(auto dev: streamDevList) {
        auto config = dev->getSyncConfig();
        if(config.syncMode == OB_SYNC_MODE_PRIMARY || config.syncMode == OB_SYNC_MODE_PRIMARY_MCU_TRIGGER || config.syncMode == OB_SYNC_MODE_PRIMARY_IR_TRIGGER
           || config.syncMode == OB_SYNC_MODE_PRIMARY_SOFT_TRIGGER) {
            primary_devices.push_back(dev);
        }
        else {
            secondary_devices.push_back(dev);
        }
    }

    if(primary_devices.empty()) {
        std::cerr << "WARNING primary_devices is empty!!!" << std::endl;
    }

    // 启动多设备时间同步功能
    context.enableMultiDeviceSync(60000);  // 每一分钟更新同步一次

    std::cout << "Secondary devices start..." << std::endl;
    startStream(secondary_devices, OB_SENSOR_DEPTH);
    startStream(secondary_devices, OB_SENSOR_COLOR);

    // 延时等待5s, 以保证从设备初始化完成
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::cout << "Primary device start..." << std::endl;
    startStream(primary_devices, OB_SENSOR_DEPTH, secondary_devices.size());
    startStream(primary_devices, OB_SENSOR_COLOR, secondary_devices.size());

    // 创建一个用于渲染的窗口，并设置窗口的分辨率
    Window app("MultiDeviceViewer", 1600, 900);
    app.setShowInfo(true);

    while(app) {
        // 获取按键事件的键值
        auto key = app.getKey();

        if(key == 'S' || key == 's') {
            std::cout << "syncDevicesTime..." << std::endl;
            context.enableMultiDeviceSync(60000);  // 手动更新同步
        }

        // 在窗口中渲染一组帧数据，这里将渲染所有设备的深度、彩色帧，
        // RENDER_GRID 表示将所有的帧按照格子排列显示
        std::vector<std::shared_ptr<ob::Frame>> framesVec;
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            for(int i = 0; i < MAX_DEVICE_COUNT; i++) {
                if(depthFrames[i] != nullptr) {
                    framesVec.emplace_back(depthFrames[i]);
                }
                if(colorFrames[i] != nullptr) {
                    framesVec.emplace_back(colorFrames[i]);
                }
            }
        }
        app.render(framesVec, RENDER_GRID);
    }

    // 关闭数据流
    stopStream(primary_devices, OB_SENSOR_COLOR);
    stopStream(primary_devices, OB_SENSOR_DEPTH);

    stopStream(secondary_devices, OB_SENSOR_COLOR);
    stopStream(secondary_devices, OB_SENSOR_DEPTH);

    std::lock_guard<std::mutex> lock(frameMutex);
    depthFrames.clear();
    colorFrames.clear();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    wait_any_key();
    exit(EXIT_FAILURE);
}

void startStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType, int deviceIndexBase) {
    for(auto &&dev: devices) {
        // 获取相机列表
        auto sensorList = dev->getSensorList();
        for(uint32_t i = 0; i < sensorList->count(); i++) {
            auto sensor = sensorList->getSensor(i);
            if(sensorType == sensor->type()) {
                auto profiles = sensor->getStreamProfileList();
                auto profile  = profiles->getProfile(0);
                switch(sensorType) {
                case OB_SENSOR_DEPTH:
                    if(profile) {
                        sensor->start(profile, [deviceIndexBase](std::shared_ptr<ob::Frame> frame) { handleDepthStream(deviceIndexBase, frame); });
                    }
                    break;
                case OB_SENSOR_COLOR:
                    if(profile) {
                        sensor->start(profile, [deviceIndexBase](std::shared_ptr<ob::Frame> frame) { handleColorStream(deviceIndexBase, frame); });
                    }
                    break;
                default:
                    break;
                }
            }
        }
        deviceIndexBase++;
    }
}

void stopStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType) {
    for(auto &&dev: devices) {
        // 获取相机列表
        auto sensorList = dev->getSensorList();
        for(uint32_t i = 0; i < sensorList->count(); i++) {
            if(sensorList->type(i) == sensorType) {
                sensorList->getSensor(i)->stop();
                break;
            }
        }
    }
}

void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
    std::cout << "Device#" << devIndex << ", color frame index=" << frame->index() << ", timestamp=" << frame->timeStamp()
              << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

    std::lock_guard<std::mutex> lock(frameMutex);
    colorFrames[devIndex] = frame;
}

void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
    std::cout << "Device#" << devIndex << ", depth frame index=" << frame->index() << ", timestamp=" << frame->timeStamp()
              << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

    std::lock_guard<std::mutex> lock(frameMutex);
    depthFrames[devIndex] = frame;
}

std::string readFileContent(const char *filePath) {
    std::ostringstream oss;

    long          length   = 0;
    long          readSum  = 0;
    int           readSize = 0;
    char          buf[512];
    bool          isOpened = false;
    bool          success  = false;
    std::ifstream file;
    file.exceptions(std::fstream::badbit | std::fstream::failbit);
    try {
        file.open(filePath, std::fstream::in);
        isOpened = true;
        file.seekg(0, std::fstream::end);
        length = file.tellg();
        file.seekg(0, std::fstream::beg);

        while(!file.eof()) {
            readSize = (std::min)((long)512, length - readSum);
            file.read(buf, readSize);
            if(file.gcount() > 0) {
                oss << std::string(buf, file.gcount());
                readSum += file.gcount();
            }
        }
        success = true;
    }
    catch(std::fstream::failure e) {
        if((file.rdstate() & std::fstream::failbit) != 0 && (file.rdstate() & std::fstream::eofbit) != 0) {
            if(readSize > 0 && file.gcount() > 0) {
                oss << std::string(buf, file.gcount());
                readSum += file.gcount();
            }
            success = true;
        }
        else {
            std::string errorMsg = (nullptr != e.what() ? std::string(e.what()) : "");
            std::cerr << "open or reading file: " << std::string(filePath) << ", errorMsg: " << errorMsg << std::endl;
        }
    }

    if(isOpened) {
        try {
            file.close();
        }
        catch(std::fstream::failure e) {
            std::string errorMsg = (nullptr != e.what() ? std::string(e.what()) : "");
            std::cerr << "close file: " << std::string(filePath) << ", errorMsg: " << errorMsg << std::endl;
        }
    }

    return success ? oss.str() : "";
}

bool loadConfigFile() {
    auto content = readFileContent(CONFIG_FILE);
    if(content.empty()) {
        std::cerr << "load config file failed." << std::endl;
        return false;
    }

    int deviceCount = 0;

    cJSON *rootElem = cJSON_Parse(content.c_str());
    if(rootElem == nullptr) {
        const char *errMsg = cJSON_GetErrorPtr();
        std::cout << std::string(errMsg) << std::endl;
        cJSON_Delete(rootElem);
        return true;
    }

    std::shared_ptr<DeviceConfigInfo> devConfigInfo = nullptr;
    cJSON *                           deviceElem    = nullptr;
    cJSON *                           devicesElem   = cJSON_GetObjectItem(rootElem, "devices");
    cJSON_ArrayForEach(deviceElem, devicesElem) {
        devConfigInfo = std::make_shared<DeviceConfigInfo>();
        memset(&devConfigInfo->syncConfig, 0, sizeof(devConfigInfo->syncConfig));
        devConfigInfo->syncConfig.syncMode = OB_SYNC_MODE_UNKNOWN;

        cJSON *snElem = cJSON_GetObjectItem(deviceElem, "sn");
        if(cJSON_IsString(snElem) && snElem->valuestring != nullptr) {
            devConfigInfo->deviceSN = std::string(snElem->valuestring);
        }

        cJSON *deviceConfigElem = cJSON_GetObjectItem(deviceElem, "syncConfig");
        if(cJSON_IsObject(deviceConfigElem)) {
            cJSON *numberElem = nullptr;
            cJSON *strElem    = nullptr;
            strElem           = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "syncMode");
            if(cJSON_IsString(strElem) && strElem->valuestring != nullptr) {
                devConfigInfo->syncConfig.syncMode = textToOBSyncMode(strElem->valuestring);
                std::cout << "config[" << (deviceCount++) << "]: SN=" << std::string(devConfigInfo->deviceSN) << ", mode=" << strElem->valuestring << std::endl;
            }

            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "irTriggerSignalInDelay");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.irTriggerSignalInDelay = numberElem->valueint;
            }

            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "rgbTriggerSignalInDelay");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.rgbTriggerSignalInDelay = numberElem->valueint;
            }

            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "deviceTriggerSignalOutDelay");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.deviceTriggerSignalOutDelay = numberElem->valueint;
            }

            numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "deviceId");
            if(cJSON_IsNumber(numberElem)) {
                devConfigInfo->syncConfig.deviceId = numberElem->valueint;
            }
        }

        if(OB_SYNC_MODE_UNKNOWN != devConfigInfo->syncConfig.syncMode) {
            deviceConfigList.push_back(devConfigInfo);
        }
        else {
            std::cerr << "invalid sync mode of deviceSN: " << devConfigInfo->deviceSN << std::endl;
        }

        devConfigInfo = nullptr;
    }

    cJSON_Delete(rootElem);
    return true;
}

OBSyncMode textToOBSyncMode(const char *text) {
    if(strcmp(text, "OB_SYNC_MODE_CLOSE") == 0) {
        return OB_SYNC_MODE_CLOSE;
    }
    else if(strcmp(text, "OB_SYNC_MODE_STANDALONE") == 0) {
        return OB_SYNC_MODE_STANDALONE;
    }
    else if(strcmp(text, "OB_SYNC_MODE_PRIMARY") == 0) {
        return OB_SYNC_MODE_PRIMARY;
    }
    else if(strcmp(text, "OB_SYNC_MODE_SECONDARY") == 0) {
        return OB_SYNC_MODE_SECONDARY;
    }
    else if(strcmp(text, "OB_SYNC_MODE_PRIMARY_MCU_TRIGGER") == 0) {
        return OB_SYNC_MODE_PRIMARY_MCU_TRIGGER;
    }
    else if(strcmp(text, "OB_SYNC_MODE_PRIMARY_IR_TRIGGER") == 0) {
        return OB_SYNC_MODE_PRIMARY_IR_TRIGGER;
    }
    else if(strcmp(text, "OB_SYNC_MODE_PRIMARY_SOFT_TRIGGER") == 0) {
        return OB_SYNC_MODE_PRIMARY_SOFT_TRIGGER;
    }
    else if(strcmp(text, "OB_SYNC_MODE_SECONDARY_SOFT_TRIGGER") == 0) {
        return OB_SYNC_MODE_SECONDARY_SOFT_TRIGGER;
    }
    else {
        return OB_SYNC_MODE_UNKNOWN;
    }
}
