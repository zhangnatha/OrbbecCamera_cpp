/** @file main.cpp
 *  @brief Orbbec相机接口示例：实时展示RGB、深度和点云数据
 *  @date 2025.09.28
 */

#include "OrbbecCameraInterface.h"
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <cstdlib>

bool parseFrame(OrbbecCameraInterface& camera) {
    std::cout << "正在解析帧数据 ..." << std::endl;

    // 创建PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    // viewer->addCoordinateSystem(0.1, "camera_origin"); // 坐标轴长度0.1米
    viewer->addCoordinateSystem(100000, 0.0, 0.0, 0.0, "camera_origin");

    cv::namedWindow("RGB Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

    // 帧率计算变量
    auto last_time = std::chrono::steady_clock::now();
    double fps = 0.0;
    int frame_count = 0;

    // 实时采集和显示循环
    bool running = true;
    bool success = false;
    while (running && !viewer->wasStopped()) {
        if (!camera.capture()) {
            std::cerr << "    ✘ 采集帧失败，尝试重新连接..." << std::endl;
            if (!camera.reconnect()) {
                std::cerr << "    ✘ 重新连接失败，继续尝试下一帧" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒后重试
            continue;
        }

        // 获取 RGB 图像
        cv::Mat rgb_image = camera.get_color_image();
        std::cout << "    ✔ Captured RGB image successfully, size: " << rgb_image.size() << ", channels: " << rgb_image.channels() << std::endl;

        // 获取深度图像
        cv::Mat depth_image = camera.get_depth_image();
        std::cout << "    ✔ Captured depth image successfully, size: " << depth_image.size() << ", channels: " << depth_image.channels() << std::endl;

        // 获取点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud = camera.get_pointcloud();
        std::cout << "    ✔ Captured point cloud, containing " << pt_cloud->size() << " points" << std::endl;

        // 计算帧率 FPS
        auto current_time = std::chrono::steady_clock::now();
        double delta_time = std::chrono::duration<double, std::milli>(current_time - last_time).count();
        if (delta_time > 0) {
            double current_fps = 1000.0 / delta_time;
            fps = current_fps;
            frame_count++;
        }
        last_time = current_time;

        // 显示 FPS
        try {
            std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
            cv::putText(rgb_image, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            if (!depth_image.empty()) {
                cv::Mat depth_vis;
                depth_image.convertTo(depth_vis, CV_8UC1, 255.0 / 5000.0); // 归一化到0-255
                cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
                cv::putText(depth_vis, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                cv::imshow("Depth Image", depth_vis);
            }
        } catch (const cv::Exception& e) {
            std::cerr << "    ✘ Failed to display FPS: " << e.what() << std::endl;
        }

        // 显示图像和点云
        try {
            cv::imshow("RGB Image", rgb_image);
            cv::setWindowProperty("RGB Image", cv::WND_PROP_VISIBLE, 1);
            cv::setWindowProperty("Depth Image", cv::WND_PROP_VISIBLE, 1);
        } catch (const cv::Exception& e) {
            std::cerr << "    ✘ Failed to display images: " << e.what() << std::endl;
        }

        try {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZRGB>(pt_cloud, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            viewer->spinOnce(30);
        } catch (const pcl::PCLException& e) {
            std::cerr << "    ✘ Failed to render point cloud: " << e.what() << std::endl;
        }

        // 检查用户输入
        int key = cv::waitKey(30);
        if (key == 27) { // ESC 键退出
            running = false;
        }

        success = true;
    }

    // 销毁窗口
    try {
        cv::destroyAllWindows();
    } catch (const cv::Exception& e) {
        std::cerr << "    ✘ Failed to destroy windows: " << e.what() << std::endl;
    }

    viewer->close();
    camera.disconnect();
    return success;
}

// 主函数
int main() {
    // 设置环境变量
    setenv("DISPLAY", ":0", 1);
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);

    bool exceptionThrown = false;
    try {
        std::cout << "🚀 启动 Orbbec 相机接口 ..." << std::endl;
        OrbbecCameraInterface camera;

        // 步骤1: 扫描相机
        std::cout << "🔧️ Step 1: Scanning cameras ..." << std::endl;
        auto cameras = camera.scanCameras();
        if (cameras.empty()) {
            throw std::logic_error("未找到相机");
        }

        // 显示相机信息
        std::cout << "找到 " << cameras.size() << " 台相机" << std::endl;
        for (size_t i = 0; i < cameras.size(); ++i) {
            std::cout << "【相机 " << (i + 1) << " / " << cameras.size() << "】" << std::endl;
            std::cout << "序列号: " << cameras[i].serial_number << std::endl;
            std::cout << "供应商: " << cameras[i].vendor << std::endl;
            std::cout << "相机固件版本: " << cameras[i].manufacturer << std::endl;
            std::cout << "----------------------------------------" << std::endl;
        }

        // 步骤2: 选择相机
        int camera_index;
        std::cout << "🔧️ Step 2: Enter camera index (0 to " << cameras.size() - 1 << "): ";
        std::cin >> camera_index;
        if (camera_index < 0 || camera_index >= static_cast<int>(cameras.size())) {
            std::cerr << "    ✘ Invalid camera index" << std::endl;
            return -1;
        }
        if (!camera.selectCamera(cameras[camera_index].serial_number)) {
            std::cerr << "    ✘ Failed to select camera" << std::endl;
            return -1;
        }

        // 步骤3: 配置相机参数
        std::cout << "🔧️ Step 3: Configuring camera parameters ..." << std::endl;
        OrbbecParam params;
        bool is_femto_bolt = cameras[camera_index].vendor.find("Femto Bolt") != std::string::npos;

        // 彩色传感器参数（基于设备支持的范围）
        params.color_auto_exposure = false;      // 自动曝光, 范围: [0, 1], 默认: 1
        params.color_exposure = 300;            // 曝光, 范围: [1, 300], 默认: 200
        params.color_gain = 32;                 // 曝光增益，范围: [0, 80], 默认: 32
        params.color_auto_white_balance = false; // 自动白平衡,范围: [0, 1], 默认: 0
        params.color_white_balance = 6500;      // 白平衡,范围: [2000, 11000], 默认: 6500
        params.color_brightness = 10;           // 亮度， 范围: [1, 20], 默认: 10
        params.color_contrast = 64;             // 对比度， 范围: [1, 99], 默认: 50
        params.color_saturation = 64;           // 饱和度， 范围: [1, 255], 默认: 64
        params.color_backlight_compensation = false; // 背光补偿, 范围: [0, 1], 默认: 0

        // 深度传感器参数
        params.depth_auto_exposure = false;     // 自动曝光开关, 默认: 0
        params.depth_exposure = 125;            // 自动曝光，范围: [20, 125], 默认: 125
        params.depth_gain = 16;                 // 数字增益，范围: [16, 248], 默认: 16

        // 流使能和分辨率/帧率参数
        params.enable_rgb = true;
        params.enable_depth = true;
        params.enable_pointcloud = true;
        params.align_method = 1;                // 对齐方式: 0 (DEPTH), 1 (COLOR)

        // 分辨率和帧率参数 is_femto_bolt
        params.depth_width = is_femto_bolt ? 512 : 1280;;
        params.depth_height = is_femto_bolt ? 512 : 800;
        params.depth_fps = is_femto_bolt ? 30 : 30;
        params.rgb_width = 1920;
        params.rgb_height = 1080;
        params.rgb_fps = is_femto_bolt ? 30 : 30;

        // 点云参数
        params.spatial_filter_enable = false;
        params.confidence_threshold_enable = false;
        params.confidence_threshold_min = 1000;
        params.image_accumulation = 1;

        // 步骤4: 连接相机
        std::cout << "🔧️ Step 4: Connecting camera ..." << std::endl;
        if (!camera.connect()) {
            throw std::logic_error("无法连接相机");
        }

        if (!camera.configureParameters(params)) {
            std::cerr << "    ✘ 无法配置相机参数" << std::endl;
            return -1;
        }

        // 等待相机稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // 步骤5: 解析相机数据
        std::cout << "🚀 Step 5: Parsing camera data ..." << std::endl;
        if (parseFrame(camera)) {
            std::cout << "相机数据解析成功!" << std::endl;
        } else {
            std::cout << "相机数据解析失败!" << std::endl;
        }

        // 步骤6: 断开相机
        std::cout << "🔧️ Step 6: Disconnecting camera ..." << std::endl;
        camera.disconnect();
    } catch (const std::exception& ex) {
        std::cout << "\n异常抛出: " << ex.what() << "\n";
        exceptionThrown = true;
    } catch (...) {
        std::cout << "\n未知异常抛出\n";
        exceptionThrown = true;
    }

    return exceptionThrown ? -1 : 0;
}