/*///////////////////////////////////////////////////////////////////////////////////////
// GC3D C++ 示例 — 完整功能测试
//
// 演示 SDK 全部输出能力:
//   1. RGB 纹理图
//   2. 原始点云 (含无效点)
//   3. 有效点云
//   4. 度量深度图 (convert_depth, float32 mm)
//   5. 归一化深度图 (uint8 可视化)
//   6. 相机内参 / 零平面高度
//   7. 2D 快照
//
// 依赖: OpenCV
//
// 操作说明:
//   SPACE / s  — 触发 3D 扫描
//   1          — 保存有效点云 (.xyz)
//   2          — 保存原始点云 (含无效点, .xyz)
//   3          — 保存度量深度图 (.bin float32)
//   4          — 保存纹理图       (.png)
//   5          — 保存归一化深度图 (.png)
//   i          — 显示相机内参
//   z          — 显示零平面高度
//   ESC / q    — 退出
///////////////////////////////////////////////////////////////////////////////////////*/

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <fstream>
#include <csignal>
#include <chrono>
#include <cstring>
#include <cstdio>

// GC3D SDK
#include "gc3d.h"
#include "GCIPropertyUtils.h"

// OpenCV
#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// 信号处理 (优雅退出)
// ---------------------------------------------------------------------------
static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

// ---------------------------------------------------------------------------
// 扫描结果容器
// ---------------------------------------------------------------------------
struct ScanResult {
    gc3d::GC3DMetaData metaData{};
    cv::Mat            textureImg;      // CV_8UC1 纹理图
    cv::Mat            depthColor;      // CV_8UC3 伪彩色深度图
    cv::Mat            depthRawFloat;   // CV_32FC1 度量深度图 (convertDepth)
    cv::Mat            previewImg;      // CV_8UC1 条纹预览图
    bool               hasColor2D = false;
    cv::Mat            color2DImg;      // 2D 彩色快照
    double             scanTimeMs = 0;
};

// ---------------------------------------------------------------------------
// 保存有效点云 (.xyz 格式)
// ---------------------------------------------------------------------------
bool saveValidPointCloud(const gc3d::GC3DMetaData& meta, const std::string& filepath) {
    FILE* fp = std::fopen(filepath.c_str(), "w");
    if (!fp) { std::cerr << "  无法写入 " << filepath << std::endl; return false; }
    int count = 0;
    for (int i = 0; i < meta.imgW * meta.imgH; ++i) {
        if (meta.maskflag[i]) {
            std::fprintf(fp, "%.6f %.6f %.6f\n", meta.x[i], meta.y[i], meta.z[i]);
            ++count;
        }
    }
    std::fclose(fp);
    std::cout << "  已保存有效点云: " << filepath << "  (" << count << " 点)" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// 保存原始点云 (含无效点, .xyz 格式)
// ---------------------------------------------------------------------------
bool saveRawPointCloud(const gc3d::GC3DMetaData& meta, const std::string& filepath) {
    FILE* fp = std::fopen(filepath.c_str(), "w");
    if (!fp) { std::cerr << "  无法写入 " << filepath << std::endl; return false; }
    int total = meta.imgW * meta.imgH;
    for (int i = 0; i < total; ++i) {
        std::fprintf(fp, "%.6f %.6f %.6f\n", meta.x[i], meta.y[i], meta.z[i]);
    }
    std::fclose(fp);
    std::cout << "  已保存原始点云: " << filepath << "  (" << total << " 点)" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// 保存度量深度图 (float32 binary)
// ---------------------------------------------------------------------------
bool saveConvertDepth(const float* data, int w, int h, const std::string& filepath) {
    FILE* fp = std::fopen(filepath.c_str(), "wb");
    if (!fp) { std::cerr << "  无法写入 " << filepath << std::endl; return false; }
    size_t sz = static_cast<size_t>(w) * h;
    std::fwrite(data, sizeof(float), sz, fp);
    std::fclose(fp);

    // 统计
    float zMin = 1e9f, zMax = -1e9f;
    int valid = 0;
    for (size_t i = 0; i < sz; ++i) {
        if (data[i] > 0) {
            if (data[i] < zMin) zMin = data[i];
            if (data[i] > zMax) zMax = data[i];
            ++valid;
        }
    }
    std::cout << "  已保存度量深度图: " << filepath
              << "  (float32, " << w << "x" << h
              << ", 有效像素=" << valid
              << ", z=[" << zMin << ", " << zMax << "] mm)" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// 保存图像 (.png)
// ---------------------------------------------------------------------------
bool saveImage(const cv::Mat& img, const std::string& filepath) {
    if (img.empty()) return false;
    cv::imwrite(filepath, img);
    std::cout << "  已保存图像: " << filepath
              << "  (" << img.cols << "x" << img.rows
              << ", " << img.channels() << "ch)" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// 一次完整采集
// ---------------------------------------------------------------------------
bool captureScan(gc3d::GC3DDevice& device, ScanResult& out, int scanNo) {
    auto t0 = std::chrono::steady_clock::now();

    // ---- 3D 扫描 ----
    uint32_t ret = device.snapShot3D();
    if (ret != GC3D_SUCCESS) {
        std::cerr << "  snapShot3D 失败: " << gc3d::GC3DDevice::getErrMsg(ret) << std::endl;
        return false;
    }

    // ---- 获取 3D 数据 ----
    ret = device.getGC3DMetaData(out.metaData);
    if (ret != GC3D_SUCCESS) {
        std::cerr << "  getGC3DMetaData 失败" << std::endl;
        return false;
    }

    // ---- 获取度量深度图 (未经 zeroPlaneHeight 转换的原始 z) ----
    int w = out.metaData.imgW;
    int h = out.metaData.imgH;
    if (w <= 0 || h <= 0) return false;

    std::vector<float> rawZ(static_cast<size_t>(w) * h);
    device.getConvertDepthData(rawZ.data());
    out.depthRawFloat = cv::Mat(h, w, CV_32FC1);
    std::memcpy(out.depthRawFloat.data, rawZ.data(), rawZ.size() * sizeof(float));

    // ---- 包装为 OpenCV Mat ----
    if (out.metaData.textureData) {
        out.textureImg = cv::Mat(h, w, CV_8UC1, out.metaData.textureData).clone();
    }
    if (out.metaData.depthImageData) {
        cv::Mat depthU8(h, w, CV_8UC1, out.metaData.depthImageData);
        cv::applyColorMap(depthU8, out.depthColor, cv::COLORMAP_JET);
    }
    if (out.metaData.previewImgData) {
        out.previewImg = cv::Mat(h, w, CV_8UC1, out.metaData.previewImgData).clone();
    }

    // ---- 2D 快照 ----
    gc3d::GC3DImageData img2d{};
    ret = device.snapShot2D(img2d);
    if (ret == GC3D_SUCCESS && img2d.data && img2d.width > 0) {
        if (img2d.channel == 3) {
            cv::Mat rgb(h, w, CV_8UC3, img2d.data);
            cv::cvtColor(rgb, out.color2DImg, cv::COLOR_RGB2BGR);
        } else {
            out.color2DImg = cv::Mat(h, w, CV_8UC1, img2d.data).clone();
        }
        out.hasColor2D = true;
    }

    auto t1 = std::chrono::steady_clock::now();
    out.scanTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return true;
}

// ---------------------------------------------------------------------------
// 在图像上绘制信息叠加层
// ---------------------------------------------------------------------------
void drawOverlay(cv::Mat& img, const std::string& text,
                 cv::Scalar color = {0, 255, 0}, int yOff = 0) {
    int baseline = 0;
    cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int y = 5 + sz.height + yOff;
    cv::rectangle(img, {0, y - sz.height - 5, sz.width + 10, sz.height + 8},
                  {0, 0, 0}, -1);
    cv::putText(img, text, {5, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
}

// ---------------------------------------------------------------------------
// 显示度量深度图 (伪彩色)
// ---------------------------------------------------------------------------
cv::Mat renderConvertDepth(const cv::Mat& depthFloat) {
    if (depthFloat.empty()) return {};
    cv::Mat vis;
    double dMin, dMax;
    cv::minMaxIdx(depthFloat, &dMin, &dMax);
    if (dMax <= dMin) dMax = dMin + 1;
    cv::Mat norm;
    depthFloat.convertTo(norm, CV_8UC1, 255.0 / (dMax - dMin), -dMin * 255.0 / (dMax - dMin));
    cv::applyColorMap(norm, vis, cv::COLORMAP_TURBO);
    return vis;
}

// ---------------------------------------------------------------------------
// 加载 INI 配置
// ---------------------------------------------------------------------------
void loadConfigFromINI(gc3d::GC3DDevice& device) {
    const std::string CONFIG_PATH = "CommonParameters.ini";
    if (!std::ifstream(CONFIG_PATH).good()) {
        std::cout << "  未找到 " << CONFIG_PATH << ", 使用默认参数" << std::endl;
        return;
    }

    GCIPropertyUtils::ConfigFile* cf =
        GCIPropertyUtils::OpenConfigFile(CONFIG_PATH.c_str(), true);
    if (!cf) return;

    auto getFloat = [&](const char* key, float def) -> float {
        std::string s = GCIPropertyUtils::GetPropertyString(cf, key, "");
        return s.empty() ? def : static_cast<float>(std::stod(s));
    };
    auto getDouble = [&](const char* key, double def) -> double {
        std::string s = GCIPropertyUtils::GetPropertyString(cf, key, "");
        return s.empty() ? def : std::stod(s);
    };

    device.setReconThreshold(
        GCIPropertyUtils::GetPropertyInteger(cf, "gminThreshold", 30),
        GCIPropertyUtils::GetPropertyInteger(cf, "gmaxThreshold", 220));
    device.setDenoiseParameters(
        GCIPropertyUtils::GetPropertyInteger(cf, "gfmr", 3),
        getFloat("gdenoiseIndex1", 20.0f),
        getFloat("gdenoiseIndex2", 1.0f),
        getFloat("gdenoiseIndex3", 2.0f));
    device.setSmoothParam(
        GCIPropertyUtils::GetPropertyInteger(cf, "gsmoothParam", 0));
    device.setHeightRange(
        getFloat("gminHeight", -1000.0f), getFloat("gmaxHeight", 1000.0f));
    device.setErodeMetaDataSize(
        GCIPropertyUtils::GetPropertyInteger(cf, "gErodeSize", 0));
    device.setNeedGridData(
        GCIPropertyUtils::GetPropertyBool(cf, "gNeedGridData", false));
    device.setGridSearchRange(
        GCIPropertyUtils::GetPropertyInteger(cf, "gridSearchRange", 30));
    device.setUseBasePlane(
        GCIPropertyUtils::GetPropertyBool(cf, "gUseBasePlane", false));
    device.setHeartBeatTimeout(
        GCIPropertyUtils::GetPropertyInteger(cf, "heartBeatTimeout", 3000));

    gc3d::GC3DCameraParameters camParams = device.getCameraParameters();
    camParams.exposureNum   = GCIPropertyUtils::GetPropertyInteger(cf, "exposureNum", 1);
    camParams.exposureTime  = GCIPropertyUtils::GetPropertyInteger(cf, "exposureTime", 800);
    camParams.exposureTime2 = GCIPropertyUtils::GetPropertyInteger(cf, "exposureTime2", 2000);
    camParams.exposureTime3 = GCIPropertyUtils::GetPropertyInteger(cf, "exposureTime3", 3000);
    camParams.gain          = getDouble("gain", 0.0);
    device.setCameraParameters(camParams);

    int exp2D   = GCIPropertyUtils::GetPropertyInteger(cf, "expTime2D", 1000);
    double gain2D = getDouble("gain2D", 5.0);
    device.setCamParam2D(exp2D, gain2D);

    GCIPropertyUtils::Close(cf);
    std::cout << "  ✓ 已加载 " << CONFIG_PATH << std::endl;
}

// ---------------------------------------------------------------------------
// 主函数
// ---------------------------------------------------------------------------
int main() {
    signal(SIGINT, signalHandler);

    std::cout << "================================================" << std::endl;
    std::cout << "  GC3D 完整示例 — RGB / 点云 / 度量深度图" << std::endl;
    std::cout << "================================================" << std::endl;

    // ---- 1. 初始化 ----
    gc3d::DeviceInformation* devInfos = nullptr;
    size_t devCount = 0;
    uint32_t ret = gc3d::initialDevice(devInfos, devCount);
    if (ret != GC3D_SUCCESS) {
        std::cerr << "[错误] initialDevice 失败: "
                  << gc3d::GC3DDevice::getErrMsg(ret) << std::endl;
        return -1;
    }

    std::cout << "检测到 " << devCount << " 台设备:" << std::endl;
    for (size_t i = 0; i < devCount; ++i) {
        std::cout << "  [" << i << "] " << devInfos[i].productType
                  << "  S/N: " << devInfos[i].serialNum
                  << "  " << devInfos[i].sensorWidth << "x" << devInfos[i].sensorHeight
                  << "  calib: fx=" << devInfos[i].calibInfo.fx
                  << " fy=" << devInfos[i].calibInfo.fy
                  << " cx=" << devInfos[i].calibInfo.cx
                  << " cy=" << devInfos[i].calibInfo.cy;
        std::cout << std::endl;
    }

    if (devCount == 0) {
        std::cerr << "[错误] 未检测到相机" << std::endl;
        gc3d::unInitialDevice();
        return -1;
    }

    // ---- 2. 打开设备 ----
    gc3d::GC3DDevice device;
    ret = device.openDeviceByIndex(0);
    if (ret != GC3D_SUCCESS) {
        std::cerr << "[错误] 打开失败: " << gc3d::GC3DDevice::getErrMsg(ret) << std::endl;
        gc3d::unInitialDevice();
        return -1;
    }

    std::cout << "\n已打开相机: " << device.getDevSerial() << std::endl;
    std::cout << "彩色相机: " << (device.isRGBDevice() ? "是" : "否") << std::endl;

    // ---- 3. 加载配置 ----
    loadConfigFromINI(device);

    // ---- 4. 显示设备参数 ----
    gc3d::DeviceInformation devInfo;
    device.getDeviceInfo(devInfo);
    std::cout << "\n设备信息:" << std::endl;
    std::cout << "  分辨率:    " << devInfo.sensorWidth << "x" << devInfo.sensorHeight << std::endl;
    std::cout << "  零平面:    " << device.getZeroPlaneHeight() << " mm" << std::endl;
    std::cout << "  最大曝光:  " << device.getMaxExposureTime() << " us" << std::endl;
    std::cout << "  内参:      fx=" << devInfo.calibInfo.fx
              << " fy=" << devInfo.calibInfo.fy
              << " cx=" << devInfo.calibInfo.cx
              << " cy=" << devInfo.calibInfo.cy << std::endl;
    std::cout << "  畸变:      k1=" << devInfo.calibInfo.kc1
              << " k2=" << devInfo.calibInfo.kc2
              << " k3=" << devInfo.calibInfo.kc3
              << " k4=" << devInfo.calibInfo.kc4
              << " k5=" << devInfo.calibInfo.kc5 << std::endl;

    // ---- 5. 窗口 ----
    cv::namedWindow("Texture",             cv::WINDOW_NORMAL);
    cv::namedWindow("Depth (Color Map)",   cv::WINDOW_NORMAL);
    cv::namedWindow("Convert Depth (mm)",  cv::WINDOW_NORMAL);

    std::cout << "\n========================================" << std::endl;
    std::cout << "  操作:" << std::endl;
    std::cout << "    SPACE / s  — 触发 3D 扫描" << std::endl;
    std::cout << "    1          — 保存有效点云" << std::endl;
    std::cout << "    2          — 保存原始点云 (含无效点)" << std::endl;
    std::cout << "    3          — 保存度量深度图 (.bin float32)" << std::endl;
    std::cout << "    4          — 保存纹理图" << std::endl;
    std::cout << "    5          — 保存归一化深度图" << std::endl;
    std::cout << "    6          — 保存条纹帧 BMP (debug_frames/)" << std::endl;
    std::cout << "    m          — 错误码翻译演示 (getErrMsg)" << std::endl;
    std::cout << "    i          — 显示相机内参" << std::endl;
    std::cout << "    z          — 显示零平面高度" << std::endl;
    std::cout << "    e / E      — 减小/增大 曝光" << std::endl;
    std::cout << "    g / G      — 减小/增大 增益" << std::endl;
    std::cout << "    ESC / q    — 退出" << std::endl;
    std::cout << "========================================" << std::endl;

    int scanCount = 0;
    ScanResult result;
    bool hasValidScan = false;

    // ---- 6. 先执行一次扫描初始显示 ----
    std::cout << "\n执行初始扫描..." << std::endl;
    hasValidScan = captureScan(device, result, ++scanCount);
    if (hasValidScan) {
        std::cout << "  分辨率=" << result.metaData.imgW << "x" << result.metaData.imgH
                  << "  有效点=" << result.metaData.validPointsNum
                  << "  扫描时间=" << result.scanTimeMs << "ms" << std::endl;

        cv::imshow("Texture", result.textureImg);
        cv::imshow("Depth (Color Map)", result.depthColor);

        cv::Mat convertVis = renderConvertDepth(result.depthRawFloat);
        drawOverlay(convertVis,
            "ConvertDepth (mm)  " + std::to_string(result.metaData.imgW) + "x"
            + std::to_string(result.metaData.imgH));
        cv::imshow("Convert Depth (mm)", convertVis);

        if (result.hasColor2D) {
            cv::namedWindow("2D Snapshot", cv::WINDOW_NORMAL);
            cv::imshow("2D Snapshot", result.color2DImg);
        }
    }

    // ---- 7. 主循环 ----
    while (g_running) {
        int key = cv::waitKey(50);
        key = (key >= 0) ? key : 0;

        // --- 退出 ---
        if (key == 27 || key == 'q' || key == 'Q') break;

        // --- 触发扫描 ---
        if (key == ' ' || key == 's' || key == 'S') {
            std::cout << "\n[" << ++scanCount << "] 扫描中..." << std::flush;
            hasValidScan = captureScan(device, result, scanCount);

            if (hasValidScan) {
                int w = result.metaData.imgW;
                int h = result.metaData.imgH;
                std::cout << " 完成  " << result.scanTimeMs << " ms" << std::endl;
                std::cout << "  " << w << "x" << h
                          << "  有效点=" << result.metaData.validPointsNum
                          << "  扫描=" << device.getScanTime() << "ms"
                          << "  零平面=" << device.getZeroPlaneHeight() << "mm" << std::endl;

                cv::Mat dispTex = result.textureImg.clone();
                drawOverlay(dispTex, "Tex  " + std::to_string(w) + "x" + std::to_string(h)
                    + "  #" + std::to_string(scanCount));

                cv::Mat dispDepth = result.depthColor.clone();
                drawOverlay(dispDepth,
                    "DepthMap  pts=" + std::to_string(result.metaData.validPointsNum)
                    + "  " + std::to_string((int)result.scanTimeMs) + "ms");

                cv::Mat convertVis = renderConvertDepth(result.depthRawFloat);
                drawOverlay(convertVis,
                    "ConvertDepth " + std::to_string(w) + "x" + std::to_string(h), {0,255,255});

                cv::imshow("Texture", dispTex);
                cv::imshow("Depth (Color Map)", dispDepth);
                cv::imshow("Convert Depth (mm)", convertVis);

                if (result.hasColor2D) {
                    cv::imshow("2D Snapshot", result.color2DImg);
                }
            } else {
                std::cout << " 失败!" << std::endl;
            }
        }

        // --- 保存有效点云 (key '1') ---
        if (key == '1' && hasValidScan) {
            std::string path = "scan_" + std::to_string(scanCount) + "_valid.xyz";
            saveValidPointCloud(result.metaData, path);
        }

        // --- 保存原始点云 (key '2') ---
        if (key == '2' && hasValidScan) {
            std::string path = "scan_" + std::to_string(scanCount) + "_raw.xyz";
            saveRawPointCloud(result.metaData, path);
        }

        // --- 保存度量深度图 (key '3') ---
        if (key == '3' && hasValidScan && !result.depthRawFloat.empty()) {
            std::string path = "scan_" + std::to_string(scanCount) + "_convertDepth.bin";
            const float* data = reinterpret_cast<const float*>(result.depthRawFloat.data);
            int w = result.metaData.imgW, h = result.metaData.imgH;
            saveConvertDepth(data, w, h, path);
        }

        // --- 保存纹理图 (key '4') ---
        if (key == '4' && hasValidScan && !result.textureImg.empty()) {
            std::string path = "scan_" + std::to_string(scanCount) + "_texture.png";
            saveImage(result.textureImg, path);
        }

        // --- 保存归一化深度图 (key '5') ---
        if (key == '5' && hasValidScan && !result.depthColor.empty()) {
            std::string path = "scan_" + std::to_string(scanCount) + "_depthColor.png";
            saveImage(result.depthColor, path);
        }

        // --- 保存条纹帧 (key '6') ---
        if (key == '6' && hasValidScan) {
            uint32_t ret6 = device.saveFringeFrames("debug_frames");
            if (ret6 == GC3D_SUCCESS) {
                std::cout << "  条纹帧已保存到 debug_frames/" << std::endl;
            } else {
                std::cout << "  saveFringeFrames 失败: "
                          << gc3d::GC3DDevice::getErrMsg(ret6) << std::endl;
            }
        }

        // --- 错误码翻译演示 (key 'm') ---
        if (key == 'm') {
            std::cout << "\nerrMsg 演示:" << std::endl;
            std::cout << "  GC3D_SUCCESS            = "
                      << gc3d::GC3DDevice::getErrMsg(0x00000000) << std::endl;
            std::cout << "  METADATA_NONE           = "
                      << gc3d::GC3DDevice::getErrMsg(0x80040000) << std::endl;
            std::cout << "  DEVICENOTOPEN           = "
                      << gc3d::GC3DDevice::getErrMsg(0x80022000) << std::endl;
        }

        // --- 显示内参 (key 'i') ---
        if (key == 'i') {
            gc3d::DeviceInformation info;
            if (GC3D_SUCCESS == device.getDeviceInfo(info)) {
                auto& c = info.calibInfo;
                std::cout << "\n相机内参:" << std::endl;
                std::cout << "  fx=" << c.fx << "  fy=" << c.fy << std::endl;
                std::cout << "  cx=" << c.cx << "  cy=" << c.cy << std::endl;
                std::cout << "  k1=" << c.kc1 << "  k2=" << c.kc2 << "  k3=" << c.kc3
                          << "  k4=" << c.kc4 << "  k5=" << c.kc5 << std::endl;
            }
        }

        // --- 显示零平面高度 (key 'z') ---
        if (key == 'z') {
            std::cout << "\n零平面高度: " << device.getZeroPlaneHeight() << " mm" << std::endl;
        }

        // --- 参数调整 ---
        if (key == 'e') {
            auto cur = device.getCameraParameters();
            cur.exposureTime = std::max(100, cur.exposureTime - 200);
            device.setCameraParameters(cur);
            std::cout << "  曝光: " << cur.exposureTime << " us" << std::endl;
        }
        if (key == 'E') {
            auto cur = device.getCameraParameters();
            cur.exposureTime = std::min(device.getMaxExposureTime(), cur.exposureTime + 200);
            device.setCameraParameters(cur);
            std::cout << "  曝光: " << cur.exposureTime << " us" << std::endl;
        }
        if (key == 'g') {
            auto cur = device.getCameraParameters();
            cur.gain = std::max(0.0, cur.gain - 0.5);
            device.setCameraParameters(cur);
            std::cout << "  增益: " << cur.gain << std::endl;
        }
        if (key == 'G') {
            auto cur = device.getCameraParameters();
            cur.gain = std::min(20.0, cur.gain + 0.5);
            device.setCameraParameters(cur);
            std::cout << "  增益: " << cur.gain << std::endl;
        }
        if (key == 'r' || key == 'R') {
            gc3d::GC3DCameraParameters def{};
            device.setCameraParameters(def);
            std::cout << "  参数已重置" << std::endl;
        }
    }

    // ---- 8. 清理 ----
    std::cout << "\n关闭相机..." << std::endl;
    device.closeDevice();
    gc3d::unInitialDevice();
    cv::destroyAllWindows();
    std::cout << "程序正常退出。" << std::endl;

    return 0;
}
