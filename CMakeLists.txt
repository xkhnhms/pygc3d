cmake_minimum_required(VERSION 3.10)
project(GC3DTest VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/GCI
)

# 查找 OpenMP
find_package(OpenMP REQUIRED)

# 静态库 + 动态库路径
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 可执行文件
add_executable(gc3d_test gc3d_test.cpp)

# 链接
target_link_libraries(gc3d_test
    ${LIB_DIR}/libGC3D.a          # GC3D 静态库
    ${CMAKE_THREAD_LIBS_INIT}     # pthread
    dl                            # dlopen
    OpenMP::OpenMP_CXX            # OpenMP
)

# 链接相机 SDK 动态库
target_link_directories(gc3d_test PRIVATE ${LIB_DIR})
target_link_libraries(gc3d_test
    MvCameraControl
    gxiapi
    cudart
)

# 设置 RPATH 让运行时能找到 .so
set_target_properties(gc3d_test PROPERTIES
    BUILD_RPATH ${LIB_DIR}
    INSTALL_RPATH ${LIB_DIR}
)
