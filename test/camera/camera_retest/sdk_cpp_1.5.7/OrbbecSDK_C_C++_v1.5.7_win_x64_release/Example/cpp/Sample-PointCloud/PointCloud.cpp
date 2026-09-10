#include "libobsensor/ObSensor.hpp"
// #include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#if defined(__linux__)
#include "../conio.h"
#else
#include <conio.h>
#endif

#define KEY_ESC 27
#define KEY_R 82
#define KEY_r 114

// 保存点云数据到ply
void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    OBPoint *point = (OBPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

// 保存彩色点云数据到ply
void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    OBColorPoint *point = (OBColorPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r, (int)point->g, (int)point->b);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

int main(int argc, char **argv) try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    // 创建pipeline
    ob::Pipeline pipeline;

    // 通过创建Config来配置Pipeline要启用或者禁用哪些流
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // 获取深度相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    auto                                    depthProfiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
    if(depthProfiles) {
        // 打开Depth Sensor默认的profile,可通过配置文件配置
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
    }
    config->enableStream(depthProfile);

    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // 获取彩色相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if(colorProfiles) {
            colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        config->enableStream(colorProfile);
    }
    catch(ob::Error &e) {
        // 如果不存在Color Sensor 点云转换分辨率配置为深度分辨率
        config->setDepthScaleRequire(false);
        config->setD2CTargetResolution(depthProfile->width(), depthProfile->height());
        std::cerr << "Current device is not support color sensor!" << std::endl;
    }

    // 开启D2C对齐, 生成RGBD点云时需要开启
    config->setAlignMode(ALIGN_D2C_HW_MODE);

    // 启动
    pipeline.start(config);

    // 创建点云Filter对象（点云Filter创建时会在Pipeline内部获取设备参数, 所以尽量在Filter创建前配置好设备）
    ob::PointCloudFilter pointCloud;

    auto cameraParam = pipeline.getCameraParam();
    pointCloud.setCameraParam(cameraParam);

    // 操作提示
    std::cout << "Press R or r to create RGBD PointCloud and save to ply file! " << std::endl;
    std::cout << "Press D or d to create Depth PointCloud and save to ply file! " << std::endl;
    std::cout << "Press ESC to exit! " << std::endl;

    int count = 0;
    while(true) {
        auto frameset = pipeline.waitForFrames(100);
        if(kbhit()) {
            int key = getch();
            // 按ESC键退出
            if(key == KEY_ESC) {
                break;
            }
            if(key == 'R' || key == 'r') {
                count = 0;
                // 限制最多重复10次
                while(count++ < 10) {
                    // 等待一帧数据，超时时间为100ms
                    auto frameset = pipeline.waitForFrames(100);
                    if(frameset != nullptr && frameset->depthFrame() != nullptr && frameset->colorFrame() != nullptr) {
                        try {
                            // 生成彩色点云并保存
                            std::cout << "Save RGBD PointCloud ply file..." << std::endl;
                            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                            saveRGBPointsToPly(frame, "RGBPoints.ply");
                            std::cout << "RGBPoints.ply Saved" << std::endl;
                        }
                        catch(std::exception &e) {
                            std::cout << "Get point cloud failed" << std::endl;
                        }
                        break;
                    }
                    else {
                        std::cout << "Get color frame or depth frame failed!" << std::endl;
                    }
                }
            }
            else if(key == 'D' || key == 'd') {
                count = 0;
                // 限制最多重复10次
                while(count++ < 10) {
                    // 等待一帧数据，超时时间为100ms
                    auto frameset = pipeline.waitForFrames(100);
                    if(frameset != nullptr && frameset->depthFrame() != nullptr) {
                        try {
                            // 生成点云并保存
                            std::cout << "Save Depth PointCloud to ply file..." << std::endl;
                            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
                            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                            savePointsToPly(frame, "DepthPoints.ply");
                            std::cout << "DepthPoints.ply Saved" << std::endl;
                        }
                        catch(std::exception &e) {
                            std::cout << "Get point cloud failed" << std::endl;
                        }
                        break;
                    }
                }
            }
        }
    }
    // 停止pipeline
    pipeline.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
