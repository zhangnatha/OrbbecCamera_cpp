/** @file OrbbecCameraInterface.cpp
 *  @brief Orbbec相机接口实现
 *  @date 2025.09.28
 */

#include "OrbbecCameraInterface.h"
#include <libobsensor/h/ObTypes.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>

// 构造函数，初始化相机接口
OrbbecCameraInterface::OrbbecCameraInterface() {
    std::cout << "    ✔ Initializing Orbbec context..." << std::endl;
    try {
        config_ = std::make_shared<ob::Config>();
        point_cloud_filter_ = std::make_shared<ob::PointCloudFilter>();
        point_cloud_filter_->setCreatePointFormat(OB_FORMAT_RGB_POINT);
        align_filter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        std::cout << "    ✔ Pipeline initialized successfully" << std::endl;
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Failed to initialize pipeline: " << e.getMessage() << std::endl;
        throw std::runtime_error("Failed to initialize Orbbec pipeline");
    }
}

// 析构函数，释放相机资源
OrbbecCameraInterface::~OrbbecCameraInterface() {
    if (status_ != CAMERA_STATUS_INIT) {
        disconnect();
    }
}

// 扫描可用的Orbbec相机
std::vector<CameraInfo> OrbbecCameraInterface::scanCameras() {
    std::vector<CameraInfo> cameras;
    try {
        auto device_list = ctx_.queryDeviceList();
        for (uint32_t i = 0; i < device_list->deviceCount(); i++) {
            auto dev = device_list->getDevice(i);
            auto dev_info = dev->getDeviceInfo();
            CameraInfo info;
            info.serial_number = dev_info->serialNumber();
            info.vendor = dev_info->name();
            info.manufacturer = dev_info->firmwareVersion();
            cameras.push_back(info);
            std::cout << "    ✔ [" << i << "] Camera found: Serial Number=" << info.serial_number
                      << ", Vendor=" << info.vendor << ", Firmware Version=" << info.manufacturer << std::endl;
        }
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Failed to scan cameras: " << e.getMessage() << std::endl;
    }
    return cameras;
}

// 选择指定序列号的相机
bool OrbbecCameraInterface::selectCamera(const std::string& sn) {
    selected_sn_ = sn;
    try {
        auto device_list = ctx_.queryDeviceList();
        for (uint32_t i = 0; i < device_list->deviceCount(); i++) {
            auto dev = device_list->getDevice(i);
            if (dev->getDeviceInfo()->serialNumber() == sn) {
                device_ = dev;
                selected_name_ = dev->getDeviceInfo()->name();
                std::cout << "    ✔ Camera selected: Serial Number=" << selected_sn_
                          << ", Model=" << selected_name_ << std::endl;
                return true;
            }
        }
        std::cerr << "    ✘ Camera not found: Serial Number=" << sn << std::endl;
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Failed to select camera: " << e.getMessage() << std::endl;
    }
    return false;
}

// 连接到选定的相机
bool OrbbecCameraInterface::connect() {
    if (selected_sn_.empty()) {
        std::cerr << "    ✘ Error: No camera selected" << std::endl;
        return false;
    }
    if (status_ == CAMERA_STATUS_CONNECTED) {
        std::cerr << "    ✘ Camera already connected" << std::endl;
        return false;
    }

    std::cout << "    ✔ Starting connection..." << std::endl;
    try {
        pipeline_ = std::make_shared<ob::Pipeline>(device_);
        std::cout << "    ✔ Connected to Serial Number=" << selected_sn_ << ", Model=" << selected_name_ << std::endl;
        status_ = CAMERA_STATUS_CONNECTED;
        return true;
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Failed to connect: " << e.getMessage() << std::endl;
        status_ = CAMERA_STATUS_INIT;
        return false;
    }
}

// 重新连接相机
bool OrbbecCameraInterface::reconnect() {
    std::cout << "    ✔ Attempting to reconnect..." << std::endl;
    disconnect();
    if (!device_) {
        std::cerr << "    ✘ Reconnect failed: No valid device available" << std::endl;
        return false;
    }
    try {
        pipeline_ = std::make_shared<ob::Pipeline>(device_);
        std::cout << "    ✔ Reconnected to Serial Number=" << selected_sn_ << ", Model=" << selected_name_ << std::endl;
        if (!configureParameters(params_)) {
            std::cerr << "    ✘ Failed to reconfigure parameters after reconnect" << std::endl;
            status_ = CAMERA_STATUS_INIT;
            return false;
        }
        status_ = CAMERA_STATUS_CONNECTED;
        return true;
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Reconnect failed: " << e.getMessage() << std::endl;
        status_ = CAMERA_STATUS_INIT;
        return false;
    }
}

// 配置相机参数
bool OrbbecCameraInterface::configureParameters(const OrbbecParam& params) {
    params_ = params;

    if (status_ != CAMERA_STATUS_CONNECTED) {
        std::cerr << "    ✘ Failed to configure parameters: Camera not connected" << std::endl;
        return false;
    }

    // 打印支持的属性及其范围
    std::cout << "    ✔ Querying supported properties..." << std::endl;
    struct PropertyInfo {
        OBPropertyID id;
        std::string name;
    };
    std::vector<PropertyInfo> properties = {
        {OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, "Color Auto Exposure"},
        {OB_PROP_COLOR_EXPOSURE_INT, "Color Exposure"},
        {OB_PROP_COLOR_GAIN_INT, "Color Gain"},
        {OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, "Color Auto White Balance"},
        {OB_PROP_COLOR_WHITE_BALANCE_INT, "Color White Balance"},
        {OB_PROP_COLOR_BRIGHTNESS_INT, "Color Brightness"},
        {OB_PROP_COLOR_CONTRAST_INT, "Color Contrast"},
        {OB_PROP_COLOR_SATURATION_INT, "Color Saturation"},
        {OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, "Depth Auto Exposure"},
        {OB_PROP_DEPTH_EXPOSURE_INT, "Depth Exposure"},
        {OB_PROP_DEPTH_GAIN_INT, "Depth Gain"},
        {OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, "Color Backlight Compensation"}
    };

    // 配置数据流
    try {
        auto depth_profiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
        auto depth_profile = depth_profiles->getVideoStreamProfile(params.depth_width, params.depth_height, OB_FORMAT_Y16, params.depth_fps);
        if (!depth_profile) {
            throw std::runtime_error("Depth profile not found");
        }
        config_->enableStream(depth_profile);

        auto color_profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
        auto color_profile = color_profiles->getVideoStreamProfile(params.rgb_width, params.rgb_height, OB_FORMAT_RGB, params.rgb_fps);
        if (!color_profile) {
            throw std::runtime_error("Color profile not found");
        }
        config_->enableStream(color_profile);

        config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        pipeline_->enableFrameSync();

        try {
            pipeline_->start(config_);
            std::cout << "    ✔ Pipeline started successfully" << std::endl;
        } catch (const ob::Error& e) {
            std::cerr << "    ✘ Pipeline start failed: " << e.getMessage() << std::endl;
            status_ = CAMERA_STATUS_INIT;
            return false;
        }

        // 应用参数，仅设置支持的属性
        auto trySetProperty = [&](OBPropertyID prop, auto value, const std::string& prop_name) {
            try {
                if (device_->isPropertySupported(prop, OB_PERMISSION_WRITE)) {
                    // 检查值是否在范围内
                    auto range = device_->getIntPropertyRange(prop);
                    if constexpr (std::is_same_v<decltype(value), bool>) {
                        if (value != 0 && value != 1) {
                            std::cerr << "Warning: Invalid value " << value << " for " << prop_name << ", must be 0 or 1" << std::endl;
                            return;
                        }
                    } else {
                        if (value < range.min || value > range.max) {
                            std::cerr << "Warning: Value " << value << " for " << prop_name << " out of range [" << range.min << ", " << range.max << "]" << std::endl;
                            return;
                        }
                    }
                    if constexpr (std::is_same_v<decltype(value), bool>) {
                        device_->setBoolProperty(prop, value);
                        bool actual = device_->getBoolProperty(prop);
                        std::cout << "    ✔ Set " << prop_name << " to " << (value ? "true" : "false") << ", Actual: " << (actual ? "true" : "false") << std::endl;
                    } else {
                        device_->setIntProperty(prop, value);
                        int actual = device_->getIntProperty(prop);
                        std::cout << "    ✔ Set " << prop_name << " to " << value << ", Actual: " << actual << std::endl;
                    }
                } else {
                    std::cerr << "Warning: Property " << prop_name << " (ID: " << prop << ") is not writable, skipping" << std::endl;
                }
            } catch (const ob::Error& e) {
                std::cerr << "Warning: Could not set property " << prop_name << " (ID: " << prop << "): " << e.getMessage() << std::endl;
            }
        };

        // 根据相机型号设置参数
        if (selected_name_.find("Gemini 336") != std::string::npos) {
            // 设置彩色参数
            trySetProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, params.color_auto_exposure, "Color Auto Exposure");
            trySetProperty(OB_PROP_COLOR_EXPOSURE_INT, params.color_exposure, "Color Exposure");
            trySetProperty(OB_PROP_COLOR_GAIN_INT, params.color_gain, "Color Gain");
            trySetProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, params.color_auto_white_balance, "Color Auto White Balance");
            trySetProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, params.color_white_balance, "Color White Balance");
            trySetProperty(OB_PROP_COLOR_BRIGHTNESS_INT, params.color_brightness, "Color Brightness");
            trySetProperty(OB_PROP_COLOR_CONTRAST_INT, params.color_contrast, "Color Contrast");
            trySetProperty(OB_PROP_COLOR_SATURATION_INT, params.color_saturation, "Color Saturation");
            trySetProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, params.color_backlight_compensation, "Color Backlight Compensation");

            // 设置深度参数
            trySetProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, params.depth_auto_exposure, "Depth Auto Exposure");
            trySetProperty(OB_PROP_DEPTH_EXPOSURE_INT, params.depth_exposure, "Depth Exposure");
            trySetProperty(OB_PROP_DEPTH_GAIN_INT, params.depth_gain, "Depth Gain");
        } else if (selected_name_.find("Orbbec Femto Bolt") != std::string::npos) {
            // 设置彩色参数（不包括背光补偿）
            trySetProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, params.color_auto_exposure, "Color Auto Exposure");
            trySetProperty(OB_PROP_COLOR_EXPOSURE_INT, params.color_exposure, "Color Exposure");
            trySetProperty(OB_PROP_COLOR_GAIN_INT, params.color_gain, "Color Gain");
            trySetProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, params.color_auto_white_balance, "Color Auto White Balance");
            trySetProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, params.color_white_balance, "Color White Balance");
            trySetProperty(OB_PROP_COLOR_BRIGHTNESS_INT, params.color_brightness, "Color Brightness");
            trySetProperty(OB_PROP_COLOR_CONTRAST_INT, params.color_contrast, "Color Contrast");
            trySetProperty(OB_PROP_COLOR_SATURATION_INT, params.color_saturation, "Color Saturation");
        } else {
            std::cerr << "Warning: Unknown camera model '" << selected_name_ << "', skipping parameter configuration" << std::endl;
        }

        return true;
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Failed to configure parameters: " << e.getMessage() << std::endl;
        status_ = CAMERA_STATUS_INIT;
        return false;
    }
}

// 捕获一帧数据
bool OrbbecCameraInterface::capture() {
    if (!pipeline_ || status_ != CAMERA_STATUS_CONNECTED) {
        std::cerr << "    ✘ Capture failed: Camera not connected (status: " << status_ << ", pipeline: " << (pipeline_ ? "valid" : "null") << ")" << std::endl;
        status_ = CAMERA_STATUS_LOST_CONNECTION;
        is_capturing_ = false;
        return false;
    }

    if (is_capturing_) {
        std::cerr << "    ✘ Capture failed: Capture in progress, skipping" << std::endl;
        return false;
    }

    is_capturing_ = true;

    try {
        auto frameset = pipeline_->waitForFrameset(2000);
        if (!frameset) {
            std::cerr << "    ✘ Capture failed: No frameset received" << std::endl;
            is_capturing_ = false;
            return false;
        }

        auto depth_frame = frameset->depthFrame();
        auto color_frame = frameset->colorFrame();
        if (!depth_frame) {
            std::cerr << "    ✘ Capture failed: Missing depth frame" << std::endl;
            is_capturing_ = false;
            return false;
        }
        if (!color_frame) {
            std::cerr << "    ✘ Capture failed: Missing color frame" << std::endl;
            is_capturing_ = false;
            return false;
        }

        auto aligned_frameset = align_filter_->process(frameset);
        if (!aligned_frameset) {
            std::cerr << "    ✘ Capture failed: Frame alignment failed" << std::endl;
            is_capturing_ = false;
            return false;
        }

        OBCameraParam camera_params = pipeline_->getCameraParam();
        point_cloud_filter_->setCameraParam(camera_params);

        auto point_cloud_frame = point_cloud_filter_->process(aligned_frameset);
        if (!point_cloud_frame || point_cloud_frame->dataSize() == 0) {
            std::cerr << "    ✘ Capture failed: No point cloud data generated (size: " << (point_cloud_frame ? point_cloud_frame->dataSize() : 0) << ")" << std::endl;
            is_capturing_ = false;
            return false;
        }

        // 保存彩色图像
        size_t width = color_frame->width();
        size_t height = color_frame->height();
        image_color_ = cv::Mat(height, width, CV_8UC3, color_frame->data());
        cv::cvtColor(image_color_, image_color_, cv::COLOR_RGB2BGR);

        // 保存深度图像
        image_depth_ = cv::Mat(depth_frame->height(), depth_frame->width(), CV_16UC1, depth_frame->data());

        // 保存点云
        pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        uint32_t points_size = point_cloud_frame->dataSize() / sizeof(OBColorPoint);
        OBColorPoint* point = (OBColorPoint*)point_cloud_frame->data();

        pointcloud_->width = depth_frame->width();
        pointcloud_->height = depth_frame->height();
        pointcloud_->points.resize(points_size);
        pointcloud_->is_dense = false;

        for (uint32_t i = 0; i < points_size; i++) {
            pointcloud_->points[i].x = point->x * 1000.0f;
            pointcloud_->points[i].y = point->y * 1000.0f;
            pointcloud_->points[i].z = point->z * 1000.0f;
            pointcloud_->points[i].r = point->r;
            pointcloud_->points[i].g = point->g;
            pointcloud_->points[i].b = point->b;

            if (std::isnan(point->x) || std::isnan(point->y) || std::isnan(point->z) ||
                point->z <= 0 || point->z > 10000) {
                pointcloud_->points[i].x = std::numeric_limits<float>::quiet_NaN();
                pointcloud_->points[i].y = std::numeric_limits<float>::quiet_NaN();
                pointcloud_->points[i].z = std::numeric_limits<float>::quiet_NaN();
            } else if (params_.confidence_threshold_enable &&
                       (point->r + point->g + point->b) / 3.0 < params_.confidence_threshold_min / 65535.0 * 255) {
                pointcloud_->points[i].x = std::numeric_limits<float>::quiet_NaN();
                pointcloud_->points[i].y = std::numeric_limits<float>::quiet_NaN();
                pointcloud_->points[i].z = std::numeric_limits<float>::quiet_NaN();
            }

            point++;
        }

        is_capturing_ = false;
        return true;
    } catch (const ob::Error& e) {
        std::cerr << "    ✘ Capture failed: " << e.getMessage() << std::endl;
        is_capturing_ = false;
        return false;
    }
}

// 断开相机连接
bool OrbbecCameraInterface::disconnect() {
    if (status_ == CAMERA_STATUS_CONNECTED || status_ == CAMERA_STATUS_CAPTURE_FAILED || status_ == CAMERA_STATUS_LOST_CONNECTION) {
        try {
            if (pipeline_) {
                std::cout << "    ✔ Stopping pipeline..." << std::endl;
                pipeline_->stop();
            }
            pipeline_.reset();
            device_.reset();
            config_.reset();
            status_ = CAMERA_STATUS_INIT;
            std::cout << "    ✔ Camera disconnected" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "    ✘ Disconnection failed: " << e.getMessage() << std::endl;
            status_ = CAMERA_STATUS_INIT;
            return false;
        }
    } else {
        std::cout << "    ✔ No active connection to disconnect" << std::endl;
        pipeline_.reset();
        device_.reset();
        config_.reset();
        status_ = CAMERA_STATUS_INIT;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        return true;
    }
}

// 获取相机当前状态
CameraStatus OrbbecCameraInterface::getStatus() {
    return status_;
}