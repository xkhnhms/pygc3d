cmake_minimum_required(VERSION 3.10)
project(GC3D_Example_x64 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    message(FATAL_ERROR "GC3D_release_examples_x64 仅支持 x86-64 架构, 当前宿主机: ${CMAKE_SYSTEM_PROCESSOR}. 请在 x64 主机上编译;")
endif()
set(HOST_ARCH_LABEL "x86-64")

# ============================================================================
# 本地路径 
# ============================================================================
set(GC3D_INCLUDE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(GC3D_LIBRARY      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libGC3D.a)

# ============================================================================
# 系统依赖
# ============================================================================

# CUDA
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgproc imgcodecs)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

# ============================================================================
# 相机 SDK 库
# ============================================================================
set(CAMERA_LIBS "")

function(append_matching_sdk _label _path)
    if(EXISTS "${_path}")
        execute_process(COMMAND file -L "${_path}" OUTPUT_VARIABLE _fx OUTPUT_STRIP_TRAILING_WHITESPACE)
        if(_fx MATCHES "${HOST_ARCH_LABEL}")
            set(CAMERA_LIBS "${CAMERA_LIBS}" "${_path}" PARENT_SCOPE)
            message(STATUS "${_label}: ${_path} (x86-64 ✓)")
        else()
            message(WARNING "${_label}: ${_path} wrong arch (${_fx})")
        endif()
    else()
        message(STATUS "${_label}: not found at ${_path}")
    endif()
endfunction()

append_matching_sdk("Haikang SDK" "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Haikang/lib/libMvCameraControl.so")
append_matching_sdk("Daheng SDK"  "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Daheng/lib/x64/libgxiapi.so")

# ============================================================================
# 可执行目标
# ============================================================================

add_executable(gc3d_viewer main.cpp)

target_include_directories(gc3d_viewer PRIVATE
    ${GC3D_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(gc3d_viewer PRIVATE
    ${GC3D_LIBRARY}
    ${CAMERA_LIBS}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    OpenMP::OpenMP_CXX
    dl
    Threads::Threads
)

set_target_properties(gc3d_viewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    LINK_FLAGS "-Wl,-rpath,${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Haikang/lib -Wl,-rpath,${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Daheng/lib/x64"
)
