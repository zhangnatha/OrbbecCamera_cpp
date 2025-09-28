/** @file OrbbecCameraInterface.h
 *  @brief Orbbec相机接口定义
 *  @date 2025.09.28
 */

#ifndef S_DEVICE_ORBBECCAMERA_H
#define S_DEVICE_ORBBECCAMERA_H

#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <vector>

/** @enum CameraStatus
 *  @brief 相机状态枚举
 */
enum CameraStatus {
    CAMERA_STATUS_INIT,           // 相机初始化状态
    CAMERA_STATUS_CONNECTED,      // 相机已连接
    CAMERA_STATUS_LOST_CONNECTION,// 相机连接丢失
    CAMERA_STATUS_CAPTURE_SUCCESS,// 捕获成功
    CAMERA_STATUS_CAPTURE_FAILED  // 捕获失败
};

/** @struct OrbbecParam
 *  @brief 相机参数结构体
 */
struct OrbbecParam {
    // 彩色传感器参数
    bool color_auto_exposure = false;     // 自动曝光: false (禁用), true (启用)
    int color_exposure = 200;             // 曝光值（us） [1,300]
    int color_gain = 16;                  // 曝光增益 [0,80]
    bool color_auto_white_balance = false;// 自动白平衡: false (禁用), true (启用)
    int color_white_balance = 6500;       // 白平衡色温（K） [2000,11000]
    int color_brightness = 0;             // 亮度 [1,20]
    int color_contrast = 64;              // 对比度 [1,99]
    int color_saturation = 64;            // 饱和度 [1,255]
    bool color_backlight_compensation = false; // 背光补偿: false (禁用), true (启用)

    // 深度传感器参数
    bool depth_auto_exposure = false;     // 自动曝光开关
    int depth_exposure = 200;             // 曝光值（us） [1,300]
    int depth_gain = 16;                  // 数字增益 [16,248]

    // 流使能和分辨率/帧率参数
    bool enable_rgb = true;               // 彩色图使能
    bool enable_depth = true;             // 深度图使能
    bool enable_pointcloud = true;        // 点云使能
    int align_method = 1;                 // 对齐方式: 0 (DEPTH), 1 (COLOR)

    // 分辨率和帧率参数
    int depth_width = 640;                // 深度流宽度
    int depth_height = 576;               // 深度流高度
    int depth_fps = 30;                   // 深度流帧率
    int rgb_width = 1920;                 // RGB流宽度
    int rgb_height = 1080;                // RGB流高度
    int rgb_fps = 30;                     // RGB流帧率

    // 点云参数
    bool spatial_filter_enable = false;   // 空间滤波使能
    bool confidence_threshold_enable = true; // 置信阈值使能
    int confidence_threshold_min = 1000;  // 置信阈值 [0,65535]
    int image_accumulation = 1;           // 图像累计 [1,20]
};

/** @struct CameraInfo
 *  @brief 相机信息结构体
 */
struct CameraInfo {
    std::string serial_number;    // 相机序列号
    std::string vendor;           // 相机供应商
    std::string manufacturer;     // 相机固件版本
};

/** @class OrbbecCameraInterface
 *  @brief Orbbec相机接口类
 */
class OrbbecCameraInterface {
public:
    /** @brief 构造函数，初始化相机接口 */
    OrbbecCameraInterface();

    /** @brief 析构函数，释放相机资源 */
    ~OrbbecCameraInterface();

    /** @brief 扫描可用的Orbbec相机 */
    std::vector<CameraInfo> scanCameras();

    /** @brief 选择指定序列号的相机 */
    bool selectCamera(const std::string& sn);

    /** @brief 连接到选定的相机 */
    bool connect();

    /** @brief 重新连接相机 */
    bool reconnect();

    /** @brief 配置相机参数 */
    bool configureParameters(const OrbbecParam& params);

    /** @brief 捕获一帧数据 */
    bool capture();

    /** @brief 断开相机连接 */
    bool disconnect();

    /** @brief 获取相机当前状态 */
    CameraStatus getStatus();

    /** @brief 获取彩色图 */
    cv::Mat get_color_image() { return image_color_; }

    /** @brief 获取深度图 */
    cv::Mat get_depth_image() { return image_depth_; }

    /** @brief 获取点云 */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_pointcloud() { return pointcloud_; }

private:
    std::shared_ptr<ob::Pipeline> pipeline_; // 相机数据管道
    std::shared_ptr<ob::Config> config_;     // 相机配置对象
    std::shared_ptr<ob::PointCloudFilter> point_cloud_filter_; // 点云滤波器
    std::shared_ptr<ob::Align> align_filter_; // 对齐滤波器
    ob::Context ctx_;                        // 相机上下文
    std::shared_ptr<ob::Device> device_;     // 当前选中的相机设备
    std::string selected_sn_;                // 选中的相机序列号
    std::string selected_name_;              // 选中的相机型号名称
    OrbbecParam params_;                     // 相机参数
    CameraStatus status_ = CAMERA_STATUS_INIT; // 相机当前状态
    bool is_capturing_ = false;              // 是否正在捕获数据

private:
    cv::Mat image_color_;
    cv::Mat image_depth_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;
};

#endif // S_DEVICE_ORBBECCAMERA_H