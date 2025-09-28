# Orbbec 3D Camera Data Processing and Visualization

This program demonstrates the acquisition, processing, and visualization of Orbbec 3D camera data using the Orbbec SDK, OpenCV, and PCL. It captures RGB images, depth images, and 3D point cloud data from an Orbbec camera, displaying all three in real-time with frame rate (FPS) overlaid on the images.

## Supported Orbbec Camera Models

### Orbbec Gemini 336

- **Depth Technology**: Active Stereo
- **Depth Range**: 0.15m - 10m
- **Depth Resolution**: Up to 1280 × 720 @ 30fps
- **RGB Sensor**: 1920 × 1080 @ 30fps
- **Field of View**: Depth: 90° × 65°, Color: 90° × 65°
- **Features**: High-resolution depth and color streams, supports advanced parameter settings

### Orbbec Femto Bolt

- **Depth Technology**: Active Stereo
- **Depth Range**: 0.25m - 10m
- **Depth Resolution**: Up to 1280 × 720 @ 30fps
- **RGB Sensor**: 1920 × 1080 @ 30fps
- **Field of View**: Depth: 87° × 58°, Color: 87° × 58°
- **Features**: Optimized for robust performance, compact design

## Camera Performance Parameters Comparison

| Parameter          | Gemini 336       | Femto Bolt       |
| :----------------- | :--------------- | :--------------- |
| Depth Technology   | Active Stereo    | Active Stereo    |
| Optimal Depth Range| 0.3-5m           | 0.25-6m          |
| Max Depth Resolution | 1280×720@30fps | 1280×720@30fps   |
| RGB Resolution     | 1920×1080@30fps  | 1920×1080@30fps  |
| Depth Accuracy     | <2%@1m           | <2%@1m           |
| Field of View      | 90°×65°          | 87°×58°          |

## Camera Parameter Configuration

The program supports configuration of the following camera parameters:

### Color Sensor Parameters

- **Auto Exposure** (`color_auto_exposure`): Enable/disable auto exposure (true/false)
- **Exposure** (`color_exposure`): 1-300, step 1, default 200
- **Gain** (`color_gain`): 0-80, step 1, default 32
- **Auto White Balance** (`color_auto_white_balance`): Enable/disable auto white balance (true/false)
- **White Balance** (`color_white_balance`): 2000-11000 (Kelvin), step 1, default 6500
- **Brightness** (`color_brightness`): 1-20, step 1, default 10
- **Contrast** (`color_contrast`): 1-99, step 1, default 50
- **Saturation** (`color_saturation`): 1-255, step 1, default 64
- **Backlight Compensation** (`color_backlight_compensation`): Enable/disable (true/false, Gemini 336 only)

### Depth Sensor Parameters (Gemini 336 only)

- **Auto Exposure** (`depth_auto_exposure`): Enable/disable auto exposure (true/false)
- **Exposure** (`depth_exposure`): 20-125, step 1, default 125
- **Gain** (`depth_gain`): 16-248, step 1, default 16

### Data Stream Parameters

- **RGB Stream Enable** (`enable_rgb`): Enable color image stream
- **Depth Stream Enable** (`enable_depth`): Enable depth image stream
- **Point Cloud Enable** (`enable_pointcloud`): Enable point cloud generation
- **Alignment Method** (`align_method`): 0 = Align to Depth, 1 = Align to Color
- **Depth Stream Resolution**: Configurable width and height (e.g., 640×576)
- **Depth Stream Frame Rate**: Configurable frame rate (e.g., 15fps)
- **RGB Stream Resolution**: Configurable width and height (e.g., 1280×720)
- **RGB Stream Frame Rate**: Configurable frame rate (e.g., 15fps)

### Point Cloud Parameters

- **Spatial Filter Enable** (`spatial_filter_enable`): Enable/disable spatial filtering
- **Confidence Threshold Enable** (`confidence_threshold_enable`): Enable/disable confidence threshold
- **Confidence Threshold Minimum** (`confidence_threshold_min`): 0-65535, default 1000
- **Image Accumulation** (`image_accumulation`): 1-20, default 1

## Execution Flow

The program follows these steps to acquire, process, and visualize camera data:

1. **Initialize Orbbec Context**: Create Orbbec SDK context and pipeline
2. **Scan Available Cameras**: Detect connected Orbbec cameras
3. **Select Camera**: Prompt user to choose a camera by index
4. **Configure Camera Parameters**: Set parameters based on camera model and application needs
5. **Connect Camera**: Establish connection and start data streams
6. **Data Acquisition Loop**:
   - Wait for and acquire frame data (RGB, depth)
   - Align frames based on configured alignment method
   - Extract and process data types
7. **Data Processing**:
   - **RGB Image Processing**: Convert color space for display
   - **Depth Image Processing**: Normalize depth data for visualization
   - **Point Cloud Generation**: Convert depth data to 3D point cloud with optional color mapping
8. **Real-time Visualization**:
   - Use OpenCV to display RGB and depth images
   - Use PCL to display 3D point cloud
   - Overlay real-time FPS information on images
9. **User Interaction**: Exit program with ESC key or by closing PCL viewer
10. **Resource Cleanup**: Disconnect camera and release resources

## Dependency Installation

### Install CMake

```shell
./install_cmake.sh
```

### Install OpenCV

```shell
./build_opencv.sh
```

### Install PCL (VTK required first)

```shell
./build_vtk.sh
./build_pcl.sh
```

### Install Orbbec SDK

```shell
./build_orbbec.sh
```

## USB Bandwidth Optimization

For high-resolution or high-frame-rate applications, increase USB buffer size to prevent frame drops:

### Temporary Setting (resets after reboot)

```shell
# Recommended: 128MB or 256MB for high-bandwidth USB devices
# Use 512MB only for systems with 16GB+ RAM
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

### Permanent Setting

Edit GRUB configuration:

```shell
sudo nano /etc/default/grub
```

Add to `GRUB_CMDLINE_LINUX`:

```shell
GRUB_CMDLINE_LINUX="usbcore.usbfs_memory_mb=256"
```

Update GRUB and reboot:

```shell
sudo update-grub
sudo reboot
```

**Note**:
- 128MB is suitable for most applications
- 256MB is recommended for high-resolution (e.g., 1080p) or high-frame-rate streams
- 512MB should only be used with sufficient system memory (16GB+)
- Excessive buffer sizes may impact system performance

## Compilation and Running

### Compile the Program

```shell
mkdir build && cd build
cmake ..
make -j4
```

### Run the Program

Set library paths and execute:

```shell
source_path="your/source/path"
export LD_LIBRARY_PATH=$source_path/3rdparty/orbbec/lib:$source_path/3rdparty/pcl/lib:$source_path/3rdparty/vtk/lib:$source_path/3rdparty/opencv/lib:
./OrbbecCameraTest
```

The program will:
1. Scan and list detected Orbbec cameras
2. Prompt user to select a camera by index
3. Configure parameters based on the camera model
4. Display real-time RGB images, depth images, and 3D point cloud
5. Overlay real-time FPS information on images

### Operation Instructions

- **ESC Key**: Exit the program
- **PCL Window Closure**: Exit the program
- **Console Output**: Displays runtime status and error messages

## Troubleshooting

### Common Issues

1. **Camera Not Detected**: Verify USB connection and ensure sufficient power supply
2. **Permission Issues**: Add user to `video` group: `sudo usermod -a -G video $USER`
3. **Low Frame Rate**: Reduce resolution or disable unnecessary streams
4. **Point Cloud Artifacts**: Adjust confidence threshold or enable spatial filtering
5. **USB Bandwidth Issues**: Increase USB buffer size as described above

### Error Messages

- **Camera Not Selected or Already Connected**: Ensure a valid camera index is provided
- **Camera Not Found**: Check USB connection and SDK installation

## Performance Optimization Suggestions

1. **Resolution Selection**: Choose resolution based on application needs; lower resolutions yield higher frame rates
2. **Stream Management**: Enable only required streams (RGB, Depth, Point Cloud)
3. **USB 3.0 Port**: Connect camera to a USB 3.0 or higher port
4. **Power Management**: Use external power if USB power is insufficient
5. **Parameter Tuning**: Adjust exposure and gain based on lighting conditions

## Example Output

Upon successful execution, the program displays three windows:
- **RGB Image**: Color image with FPS overlay
- **Depth Image**: Color-mapped depth image with FPS overlay
- **Point Cloud Viewer**: Interactive 3D point cloud visualization

The console outputs camera information, configuration details, and runtime status.

---

# Orbbec 3D Camera Data Processing and Visualization (English)

This program demonstrates the acquisition, processing, and visualization of Orbbec 3D camera data using the Orbbec SDK, OpenCV, and PCL. It captures RGB images, depth images, and 3D point cloud data, displaying them in real-time with FPS information overlaid.

## Dependencies Installation

### Install CMake

```shell
./install_cmake.sh
```

### Install Eigen

```shell
./build_eigen.sh
```

### Install OpenCV

```shell
./build_opencv.sh
```

### Install PCL (VTK required first)

```shell
./build_vtk.sh
./build_pcl.sh
```

### Install Orbbec SDK

```shell
./build_orbbec.sh
```

## USB Bandwidth Optimization

For high-resolution/high-frame-rate applications, increase USB buffer size:

### Temporary Setting

```shell
# Recommended: 128MB or 256MB
# 512MB only for systems with 16GB+ RAM
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

### Permanent Setting

Edit GRUB configuration:

```shell
sudo nano /etc/default/grub
```

Add to `GRUB_CMDLINE_LINUX`:

```shell
GRUB_CMDLINE_LINUX="usbcore.usbfs_memory_mb=256"
```

Update GRUB and reboot:

```shell
sudo update-grub
sudo reboot
```

## Usage

1. Ensure the Orbbec camera is connected and compatible with the Orbbec SDK
2. Install dependencies using the provided scripts
3. Optimize USB bandwidth if needed
4. Compile the program:

```shell
mkdir build && cd build
cmake ..
make -j4
```

5. Run the executable:

```shell
source_path="your/source/path"
export LD_LIBRARY_PATH=$source_path/3rdparty/orbbec/lib:$source_path/3rdparty/pcl/lib:$source_path/3rdparty/vtk/lib:$source_path/3rdparty/opencv/lib:
./OrbbecCameraTest
```

6. Follow on-screen instructions to select and configure the camera
7. View real-time RGB image, depth image, and 3D point cloud
8. Press ESC or close the PCL viewer to exit

## Key Features

- Real-time data acquisition and visualization
- Automatic camera detection and model-specific configuration
- Support for Gemini 336 and Femto Bolt cameras
- Configurable camera parameters for optimal performance
- Real-time FPS display
- Interactive 3D point cloud visualization
- USB bandwidth optimization for high-performance applications


# Orbbec 3D 相机数据处理与可视化

本程序展示了使用 Orbbec SDK、OpenCV 和 PCL 进行 Orbbec 3D 相机数据采集、处理和可视化的过程。它从相机捕获 RGB 图像、深度图像和 3D 点云数据，并实时显示三者，同时在图像上叠加帧率（FPS）信息。

## 支持的 Orbbec 相机型号

### Orbbec Gemini 336

- **深度技术**：主动立体视觉
- **深度范围**：0.15m - 10m
- **深度分辨率**：最高 1280 × 720 @ 30fps
- **RGB 传感器**：1920 × 1080 @ 30fps
- **视野角度**：深度：90° × 65°，彩色：90° × 65°
- **特点**：高分辨率深度和彩色流，支持高级参数设置

### Orbbec Femto Bolt

- **深度技术**：主动立体视觉
- **深度范围**：0.25m - 10m
- **深度分辨率**：最高 1280 × 720 @ 30fps
- **RGB 传感器**：1920 × 1080 @ 30fps
- **视野角度**：深度：87° × 58°，彩色：87° × 58°
- **特点**：性能稳定，设计紧凑

## 相机性能参数对比

| 参数           | Gemini 336       | Femto Bolt       |
| :------------- | :--------------- | :--------------- |
| 深度技术       | 主动立体视觉     | 主动立体视觉     |
| 最佳深度范围   | 0.3-5m           | 0.25-6m          |
| 最大深度分辨率 | 1280×720@30fps   | 1280×720@30fps   |
| RGB 分辨率     | 1920×1080@30fps  | 1920×1080@30fps  |
| 深度精度       | <2%@1m           | <2%@1m           |
| 视野角度       | 90°×65°          | 87°×58°          |

## 相机参数配置

程序支持配置以下相机参数：

### 彩色传感器参数

- **自动曝光** (`color_auto_exposure`)：启用/禁用自动曝光（true/false）
- **曝光时间** (`color_exposure`)：1-300，步长1，默认200
- **增益** (`color_gain`)：0-80，步长1，默认32
- **自动白平衡** (`color_auto_white_balance`)：启用/禁用自动白平衡（true/false）
- **白平衡** (`color_white_balance`)：2000-11000（开尔文），步长1，默认6500
- **亮度** (`color_brightness`)：1-20，步长1，默认10
- **对比度** (`color_contrast`)：1-99，步长1，默认50
- **饱和度** (`color_saturation`)：1-255，步长1，默认64
- **背光补偿** (`color_backlight_compensation`)：启用/禁用（true/false，仅 Gemini 336 支持）

### 深度传感器参数（仅 Gemini 336）

- **自动曝光** (`depth_auto_exposure`)：启用/禁用自动曝光（true/false）
- **曝光时间** (`depth_exposure`)：20-125，步长1，默认125
- **增益** (`depth_gain`)：16-248，步长1，默认16

### 数据流参数

- **RGB 流使能** (`enable_rgb`)：启用彩色图像流
- **深度流使能** (`enable_depth`)：启用深度图像流
- **点云使能** (`enable_pointcloud`)：启用点云生成
- **对齐方式** (`align_method`)：0=对齐到深度，1=对齐到彩色
- **深度流分辨率**：可配置宽度和高度（例如 640×576）
- **深度流帧率**：可配置帧率（例如 15fps）
- **RGB 流分辨率**：可配置宽度和高度（例如 1280×720）
- **RGB 流帧率**：可配置帧率（例如 15fps）

### 点云参数

- **空间滤波使能** (`spatial_filter_enable`)：启用/禁用空间滤波
- **置信度阈值使能** (`confidence_threshold_enable`)：启用/禁用置信度阈值
- **置信度阈值最小值** (`confidence_threshold_min`)：0-65535，默认1000
- **图像累计** (`image_accumulation`)：1-20，默认1

## 执行流程

程序按照以下步骤采集、处理和可视化相机数据：

1. **初始化 Orbbec 上下文**：创建 Orbbec SDK 上下文和管道
2. **扫描可用相机**：检测系统中连接的 Orbbec 相机设备
3. **选择相机**：根据用户输入选择要使用的相机
4. **配置相机参数**：根据相机型号和应用需求配置各项参数
5. **连接相机**：建立与相机的连接并启动数据流
6. **数据采集循环**：
   - 等待并获取帧数据（RGB、深度）
   - 根据配置的对齐方式对齐帧数据
   - 提取和处理各类型数据
7. **数据处理**：
   - **RGB 图像处理**：转换颜色空间以便显示
   - **深度图像处理**：归一化深度数据以便可视化
   - **点云生成**：将深度数据转换为 3D 点云，可选颜色映射
8. **实时可视化**：
   - 使用 OpenCV 显示 RGB 和深度图像
   - 使用 PCL 显示 3D 点云
   - 在图像上叠加实时 FPS 信息
9. **用户交互**：支持通过 ESC 键或关闭 PCL 窗口退出程序
10. **资源清理**：断开相机连接，释放资源

## 依赖项安装

### 安装 CMake

```shell
./install_cmake.sh
```

### 安装 OpenCV

```shell
./build_opencv.sh
```

### 安装 PCL（需要先安装 VTK）

```shell
./build_vtk.sh
./build_pcl.sh
```

### 安装 Orbbec SDK

```shell
./build_orbbec.sh
```

## USB 带宽优化（针对高分辨率/高帧率应用）

Orbbec 相机在传输高分辨率数据时需要较大的 USB 带宽。如果遇到帧丢失或性能问题，建议增加 USB 缓冲区大小：

### 临时设置（重启后失效）

```shell
# 对于高带宽 USB 设备，推荐设置 128MB 或 256MB
# 系统内存 16GB 以上可设置 512MB，设置过高会影响系统内存
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

### 永久设置

编辑 GRUB 配置文件：

```shell
sudo nano /etc/default/grub
```

在 `GRUB_CMDLINE_LINUX` 行中添加：

```shell
GRUB_CMDLINE_LINUX="usbcore.usbfs_memory_mb=256"
```

更新 GRUB 并重启：

```shell
sudo update-grub
sudo reboot
```

**注意**：
- 128MB 适用于大多数应用场景
- 256MB 适用于高分辨率（例如 1080p）或高帧率应用
- 512MB 仅建议在系统内存充足（16GB+）的情况下使用
- 设置过大会占用系统内存，影响其他应用性能

## 编译和运行

### 编译程序

```shell
mkdir build && cd build
cmake ..
make -j4
```

### 运行程序

设置库路径并运行：

```shell
source_path="your/source/path"
export LD_LIBRARY_PATH=$source_path/3rdparty/orbbec/lib:$source_path/3rdparty/pcl/lib:$source_path/3rdparty/vtk/lib:$source_path/3rdparty/opencv/lib:
./OrbbecCameraTest
```

程序运行后将：
1. 扫描并列出检测到的 Orbbec 相机
2. 提示用户选择要使用的相机（输入索引）
3. 根据相机型号自动配置参数
4. 开始实时显示 RGB 图像、深度图像和 3D 点云
5. 在图像上显示实时帧率信息

### 操作说明

- **ESC 键**：退出程序
- **PCL 窗口关闭**：退出程序
- **控制台输出**：显示运行状态和错误信息

## 故障排除

### 常见问题

1. **相机未检测到**：检查 USB 连接，确保相机供电充足
2. **权限问题**：将用户添加到 `video` 组：`sudo usermod -a -G video $USER`
3. **帧率过低**：降低分辨率或关闭不需要的数据流
4. **点云空洞**：调整置信度阈值或启用空间滤波
5. **USB 带宽不足**：按照上述方法增加 USB 缓冲区大小

### 错误消息

- **相机未选择或已连接**：确保提供有效的相机索引
- **相机未找到**：检查 USB 连接和 SDK 安装

## 性能优化建议

1. **分辨率选择**：根据应用需求选择合适的分辨率，低分辨率可获得更高帧率
2. **数据流管理**：只启用需要的数据流（RGB、深度、点云）
3. **USB 3.0 端口**：确保相机连接到 USB 3.0 或更高版本的端口
4. **电源管理**：使用外部电源供电避免 USB 供电不足
5. **参数调优**：根据环境光照调整曝光、增益等参数

## 示例输出

程序成功运行后，将显示三个窗口：
- **RGB 图像**：彩色图像，顶部有 FPS 信息
- **深度图像**：伪彩色映射的深度图像，顶部有 FPS 信息
- **点云视图**：交互式 3D 点云可视化

控制台将输出相机信息、配置参数和运行状态。
