#!/bin/bash
# 脚本用于在Linux环境下编译并安装Orbbec SDK v2到3rdparty/orbbec目录
# 如果任何步骤失败，将报错并退出，不生成任何文件

# 开启错误检查，任何命令失败立即退出
set -e

# 设置安装路径
INSTALL_DIR="$(pwd)/3rdparty/orbbec"
SDK_DIR="$(pwd)/OrbbecSDK_v2"
BUILD_DIR="$SDK_DIR/build"

# 1. 安装依赖
echo "Installing dependencies..."
sudo apt-get update || { echo "Failed to update package lists"; exit 1; }
sudo apt-get install -y build-essential || { echo "Failed to install dependencies"; exit 1; }

# 2. 克隆Orbbec SDK v2仓库
if [ ! -d "$SDK_DIR" ]; then
    echo "Cloning OrbbecSDK_v2 repository..."
    git clone https://github.com/orbbec/OrbbecSDK_v2.git || { echo "Failed to clone OrbbecSDK_v2 repository"; exit 1; }
else
    echo "OrbbecSDK_v2 directory already exists, pulling latest changes..."
    cd $SDK_DIR
    git pull || { echo "Failed to pull latest changes"; exit 1; }
    cd ..
fi

# 3. 创建构建目录
echo "Creating build directory..."
mkdir -p $BUILD_DIR || { echo "Failed to create build directory"; exit 1; }
cd $BUILD_DIR

# 4. 配置CMake
echo "Configuring CMake..."
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR || { echo "CMake configuration failed"; exit 1; }

# 5. 编译Orbbec SDK
echo "Building Orbbec SDK..."
cmake --build . --config Release || { echo "Build failed"; exit 1; }

# 6. 安装到3rdparty/orbbec
echo "Installing Orbbec SDK to $INSTALL_DIR..."
make install || { echo "Installation failed"; exit 1; }

# 7. 配置udev规则（Linux下需要）
echo "Installing udev rules..."
cd $INSTALL_DIR/shared || { echo "Failed to locate shared directory"; exit 1; }
sudo chmod +x ./install_udev_rules.sh || { echo "Failed to set executable permissions for install_udev_rules.sh"; exit 1; }
sudo ./install_udev_rules.sh || { echo "Failed to install udev rules"; exit 1; }
sudo udevadm control --reload || { echo "Failed to reload udev rules"; exit 1; }
sudo udevadm trigger || { echo "Failed to trigger udev rules"; exit 1; }

# 8. 验证安装
echo "Verifying installation..."
if [ -d "$INSTALL_DIR/lib" ] && [ -d "$INSTALL_DIR/include" ]; then
    echo "Orbbec SDK installed successfully at $INSTALL_DIR"
else
    echo "Installation failed: lib or include directory not found"
    exit 1
fi

# 9. 清除
echo "清除..."
rm -rf ${SDK_DIR}
