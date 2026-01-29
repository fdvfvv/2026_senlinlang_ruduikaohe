cmake_minimum_required(VERSION 3.10)
project(Armor3DDetect)

# 设置C++标准（兼容std::optional）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV 4.5.x（用户指定版本）
find_package(OpenCV 4.5 REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Error: OpenCV 4.5.x not found! Please install OpenCV 4.5.4.")
endif()

# 包含头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# 源文件列表（新增卡尔曼滤波文件）
set(SOURCES
    src/light_bar_detect.cpp
    src/armor_match.cpp
    src/sanweizuobiao.cpp
    src/kalman.cpp  # 新增卡尔曼滤波源文件
)

# 生成可执行文件
add_executable(armor_detect ${SOURCES})

# 链接OpenCV库
target_link_libraries(armor_detect
    ${OpenCV_LIBS}
)

# 可选：将视频文件复制到构建目录（避免路径问题）
configure_file(src/test.mp4 ${PROJECT_BINARY_DIR}/test.mp4 COPYONLY)
message(STATUS "Video file copied to build directory.")
