cmake_minimum_required(VERSION 3.15)
project(OrbbecCameraTest)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置第三方库路径
set(THIRDPARTY_DIR "${CMAKE_SOURCE_DIR}/3rdparty")
message(STATUS "THIRDPARTY_DIR: ${THIRDPARTY_DIR}")

# 手动查找Orbbec SDK
set(OrbbecSDK_DIR "${THIRDPARTY_DIR}/orbbec")
find_library(OrbbecSDK_LIBRARY NAMES OrbbecSDK orbbecsdk PATHS "${OrbbecSDK_DIR}/lib" NO_DEFAULT_PATH)
find_path(OrbbecSDK_INCLUDE_DIR NAMES libobsensor/ObSensor.hpp PATHS "${OrbbecSDK_DIR}/include" NO_DEFAULT_PATH)
if (NOT OrbbecSDK_LIBRARY)
    message(FATAL_ERROR "Orbbec SDK library not found in ${OrbbecSDK_DIR}/lib")
endif()
if (NOT OrbbecSDK_INCLUDE_DIR)
    message(FATAL_ERROR "Orbbec SDK header (ObSensor.hpp) not found in ${OrbbecSDK_DIR}/include")
endif()
message(STATUS "OrbbecSDK_LIBRARY: ${OrbbecSDK_LIBRARY}")
message(STATUS "OrbbecSDK_INCLUDE_DIR: ${OrbbecSDK_INCLUDE_DIR}")

# 寻找OpenCV
set(OpenCV_DIR "${THIRDPARTY_DIR}/opencv/lib/cmake/opencv4")
if (NOT EXISTS ${OpenCV_DIR})
    message(WARNING "OpenCV path does not exist: ${OpenCV_DIR}. Trying system OpenCV...")
    find_package(OpenCV REQUIRED)
else()
    find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR})
endif()

# 设置 VTK 路径
set(VTK_DIR "${THIRDPARTY_DIR}/vtk/lib/cmake/vtk-8.2")
find_package(VTK REQUIRED)
message(STATUS "VTK_DIR: ${VTK_DIR}")
message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}")

# 设置 PCL 路径
set(PCL_DIR "${THIRDPARTY_DIR}/pcl/share/pcl-1.12")
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)## 添加visualization
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# 包含目录
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${OrbbecSDK_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

# 创建可执行文件
add_executable(${PROJECT_NAME}
        main.cpp
        src/OrbbecCameraInterface.cpp
)

# 链接库
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${OrbbecSDK_LIBRARY}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES})

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Linux运行时库路径
if(UNIX AND NOT APPLE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH "${OrbbecSDK_DIR}/lib")
endif()
