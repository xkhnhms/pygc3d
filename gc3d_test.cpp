/*///////////////////////////////////////////////////////////////////////////////////////
// gc3d_test.cpp — GC3D 相机 C++ 测试用例
//
// 功能:
//   1. 枚举设备信息
//   2. 打开相机并配置参数 (曝光、增益、降噪、重建阈值、高度范围)
//   3. 拍摄 3D 扫描:
//      - 获取点云 (x, y, z)
//      - 获取纹理图 (RGB/灰度) → 保存 BMP
//      - 获取深度图 → 保存 BMP
//      - 获取网格化数据 (可选)
//      - 保存点云为 .txt / .xyz / .gci
//   4. 拍摄 2D 图像 → 保存 BMP
//   5. 关闭设备并清理
//
// 编译:
//   ./build.sh
//
// 依赖:
//   - libGC3D.a (或 libGC3D.so)
//   - libMvCameraControl.so / libgxiapi.so / libcudart.so
//   - Eigen3 (头文件)
//   - OpenMP
///////////////////////////////////////////////////////////////////////////////////////*/
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>
#include <ctime>

#include "GCI/gc3d.h"
#include "save_bmp.h"

// ========== 辅助函数 ==========

static std::string getTimestamp() {
    std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    return buf;
}

static std::string getOutputDir() {
    std::string dir = "output/" + getTimestamp();
    std::string mkdirCmd = "mkdir -p " + dir;
    int ret = system(mkdirCmd.c_str());
    (void)ret;  // 忽略 mkdir 返回值
    return dir;
}

/// 保存点云为 XYZ 格式 (每行: x y z)
static bool savePointCloudXYZ(const std::string& filename,
                              const float* x, const float* y, const float* z,
                              const bool* mask, int imgW, int imgH,
                              bool onlyValid = true) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;

    int count = 0;
    for (int i = 0; i < imgW * imgH; ++i) {
        if (onlyValid && (!mask || !mask[i])) continue;
        ofs << x[i] << " " << y[i] << " " << z[i] << "\n";
        ++count;
    }
    ofs.close();
    std::cout << "  [保存] " << filename << "  (" << count << " 个点)" << std::endl;
    return true;
}

/// 保存点云为 TXT 格式 (含有效标志, 每行: x y z valid)
static bool savePointCloudTXT(const std::string& filename,
                              const float* x, const float* y, const float* z,
                              const bool* mask, int imgW, int imgH) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;

    for (int i = 0; i < imgW * imgH; ++i) {
        ofs << x[i] << " " << y[i] << " " << z[i] << " "
            << (mask && mask[i] ? "1" : "0") << "\n";
    }
    ofs.close();
    std::cout << "  [保存] " << filename << "  (全部 " << imgW * imgH << " 个点)"
              << std::endl;
    return true;
}

/// 打印错误码信息
static void checkError(uint32_t ret, const std::string& stage) {
    if (ret != GC3D_SUCCESS) {
        std::cerr << "  [错误] " << stage << ": 0x" << std::hex << ret
                  << " (" << gc3d::GC3DDevice::getErrMsg(ret) << ")" << std::dec
                  << std::endl;
    }
}

// ========== 主函数 ==========

int main() {
    std::cout << "═══════════════════════════════════════════════\n"
              << "  GC3D 相机 C++ 测试程序\n"
              << "  SDK 版本: " << gc3d::GC3DDevice::getGCIVersion() << "\n"
              << "═══════════════════════════════════════════════\n"
              << std::endl;

    // ---------------------------------------------------------------
    // 第 1 步: 初始化设备, 枚举相机
    // ---------------------------------------------------------------
    gc3d::DeviceInformation* devInfos = nullptr;
    size_t devNum = 0;

    uint32_t ret = gc3d::initialDevice(devInfos, devNum);
    if (ret != GC3D_SUCCESS) {
        std::cerr << "[错误] initialDevice 失败: 0x" << std::hex << ret
                  << " " << gc3d::GC3DDevice::getErrMsg(ret) << std::dec
                  << std::endl;
        return 1;
    }

    std::cout << "发现 " << devNum << " 个设备:" << std::endl;
    for (size_t i = 0; i < devNum; ++i) {
        std::cout << "  [" << i << "] "
                  << "型号: " << devInfos[i].productType
                  << "  序列号: " << devInfos[i].serialNum
                  << "  分辨率: " << devInfos[i].sensorWidth
                  << "x" << devInfos[i].sensorHeight
                  << std::endl;
    }

    if (devNum == 0) {
        std::cout << "未检测到相机, 退出。" << std::endl;
        gc3d::unInitialDevice();
        return 0;
    }

    // ---------------------------------------------------------------
    // 第 2 步: 打开第一个相机
    // ---------------------------------------------------------------
    gc3d::GC3DDevice dev;
    ret = dev.openDeviceByIndex(0);
    if (ret != GC3D_SUCCESS) {
        std::cerr << "[错误] openDeviceByIndex(0) 失败: 0x" << std::hex << ret
                  << " " << gc3d::GC3DDevice::getErrMsg(ret) << std::dec
                  << std::endl;
        gc3d::unInitialDevice();
        return 1;
    }

    std::cout << "\n相机已打开: " << dev.getDevSerial() << std::endl;

    bool isRGB = dev.isRGBDevice();
    std::cout << "彩色相机: " << (isRGB ? "是" : "否") << std::endl;

    // ---------------------------------------------------------------
    // 第 3 步: 配置相机参数
    // ---------------------------------------------------------------
    std::cout << "\n--- 配置相机参数 ---" << std::endl;

    // 3a. 获取当前参数和最大曝光时间
    int maxExp = dev.getMaxExposureTime();
    gc3d::GC3DCameraParameters camParam = dev.getCameraParameters();
    std::cout << "  当前参数: 曝光=" << camParam.exposureTime
              << "us, 增益=" << camParam.gain
              << ", 曝光次数=" << camParam.exposureNum
              << ", 最大曝光=" << maxExp << "us" << std::endl;

    // 修改曝光参数 (使用 SDK 默认曝光时间附近的值, 避免触发失败)
    // camParam.exposureTime = std::min(10000, maxExp);  // 保守值 10ms
    camParam.exposureTime = std::min(45000, maxExp);  // 保守值 10ms
    camParam.exposureNum  = 1;       // 单次曝光
    ret = dev.setCameraParameters(camParam);
    checkError(ret, "setCameraParameters");
    std::cout << "  设置参数: 曝光=" << camParam.exposureTime
              << "us, 增益=" << camParam.gain
              << ", 曝光次数=" << camParam.exposureNum << std::endl;

    // 3b. 设置重建阈值 (亮度)
    ret = dev.setReconThreshold(30, 200);
    checkError(ret, "setReconThreshold");

    int minThr = 0, maxThr = 0;
    dev.getReconThreshold(minThr, maxThr);
    std::cout << "  重建阈值: [" << minThr << ", " << maxThr << "]" << std::endl;

    // 3c. 设置降噪参数
    //     fmr: 去噪半径 (1-3)
    //     denoiseIndex1: 半径内最少有效点数
    //     denoiseIndex2: Z 方差阈值
    //     denoiseIndex3: 局部高度差阈值
    ret = dev.setDenoiseParameters(3, 50, 5.0f, 5.0f);
    checkError(ret, "setDenoiseParameters");

    int fmr;
    float den1, den2, den3;
    dev.getDenoiseParameters(fmr, den1, den2, den3);
    std::cout << "  降噪参数: fmr=" << fmr
              << ", ft1=" << den1
              << ", ft2=" << den2
              << ", ft3=" << den3 << std::endl;

    // 3d. 设置高度范围
    float zeroPlane = dev.getZeroPlaneHeight();
    float maxHeight = 50.0f;
    ret = dev.setHeightRange(zeroPlane - maxHeight, zeroPlane + maxHeight);
    checkError(ret, "setHeightRange");

    float hMin, hMax;
    dev.getHeightRange(hMin, hMax);
    std::cout << "  高度范围: [" << hMin << ", " << hMax << "] mm"
              << "  (零平面: " << zeroPlane << " mm)" << std::endl;

    // 3e. 设置平滑参数
    ret = dev.setSmoothParam(1);
    checkError(ret, "setSmoothParam");
    std::cout << "  平滑参数: " << dev.getSmoothParam() << std::endl;

    // 3f. 启用网格化数据 (可选)
    dev.setNeedGridData(true);
    std::cout << "  网格化数据: " << (dev.getNeedGridData() ? "启用" : "关闭")
              << std::endl;

    // 3g. 设置 2D 曝光和增益
    ret = dev.setCamParam2D(30000, 5.0);
    checkError(ret, "setCamParam2D");
    int exp2D;
    double gain2D;
    dev.getCamParam2D(exp2D, gain2D);
    std::cout << "  2D 参数: 曝光=" << exp2D << "us, 增益=" << gain2D << std::endl;

    // ---------------------------------------------------------------
    // 第 4 步: 拍摄 3D 扫描
    // ---------------------------------------------------------------
    std::cout << "\n--- 拍摄 3D 扫描 ---" << std::endl;

    ret = dev.snapShot3D();
    if (ret != GC3D_SUCCESS) {
        std::cerr << "[错误] snapShot3D 失败: 0x" << std::hex << ret
                  << " (" << gc3d::GC3DDevice::getErrMsg(ret) << ")" << std::dec
                  << std::endl;

        // 如果是相机触发失败, 尝试恢复默认曝光后重试
        if (ret == 0x82010000) {
            std::cerr << "\n  ⚠ 相机未触发 (Projector trigger failed)\n"
                      << "    可能的原因:\n"
                      << "    1. 曝光时间设置不当 — 某些曝光值投影仪不支持\n"
                      << "    2. 相机投影仪触发线未连接或接触不良\n"
                      << "    3. 相机需要重新上电初始化投影模块\n"
                      << "\n"
                      << "    尝试恢复默认曝光重新触发...\n"
                      << std::endl;

            gc3d::GC3DCameraParameters defaultParam;
            dev.setCameraParameters(defaultParam);
            std::cout << "  [重试] 曝光=" << defaultParam.exposureTime << "us" << std::endl;
            ret = dev.snapShot3D();
        }

        // 重试后再次判断
        if (ret != GC3D_SUCCESS) {
            std::cerr << "[错误] 扫描失败, 退出。" << std::endl;
            if (ret == 0x82010000) {
                std::cerr << "  建议: 在 GC3DExample 目录下用 ./gc3dexample 测试硬件是否正常" << std::endl;
            }
            dev.closeDevice();
            gc3d::unInitialDevice();
            return 1;
        }

        std::cout << "  [重试] 成功!" << std::endl;
    }

    double scanTimeUs = dev.getScanTime();
    std::cout << "  扫描耗时: " << (scanTimeUs / 1000.0) << " ms"
              << "  (" << scanTimeUs << " us)" << std::endl;

    // 获取 3D 元数据
    gc3d::GC3DMetaData data;
    ret = dev.getGC3DMetaData(data);
    if (ret != GC3D_SUCCESS) {
        std::cerr << "[错误] getGC3DMetaData 失败: 0x" << std::hex << ret
                  << " " << gc3d::GC3DDevice::getErrMsg(ret) << std::dec
                  << std::endl;
        dev.closeDevice();
        gc3d::unInitialDevice();
        return 1;
    }

    int imgW = data.imgW;
    int imgH = data.imgH;
    std::cout << "  图像尺寸: " << imgW << "x" << imgH
              << "\n  有效点数: " << data.validPointsNum
              << "  / 总计: " << (imgW * imgH) << std::endl;

    if (data.validPointsNum == 0) {
        std::cout << "\n  ⚠ 有效点数为 0, 可能的原因:\n"
                  << "    1. 曝光值不合适 (当前 " << camParam.exposureTime << "us)\n"
                  << "    2. 重建阈值 [30, 200] 过滤了所有点\n"
                  << "    3. 物体不在高度范围 [" << hMin << ", " << hMax << "] mm 内\n"
                  << "    建议: 检查保存的 BMP 图像 (texture.bmp / preview.bmp)\n"
                  << "    如果图像全黑 → 增加曝光; 如果全白 → 减少曝光\n"
                  << "    或用 GC3DExample/gc3dexample 默认参数测试\n"
                  << std::endl;
    }

    // 打印纹理/深度图像统计信息 (帮助诊断 0 有效点)
    if (data.textureData) {
        unsigned char tMin = 255, tMax = 0;
        for (int i = 0; i < imgW * imgH; ++i) {
            if (data.textureData[i] < tMin) tMin = data.textureData[i];
            if (data.textureData[i] > tMax) tMax = data.textureData[i];
        }
        std::cout << "  纹理图: " << (int)tMin << "~" << (int)tMax;
        if (tMax <= 5) std::cout << " ⚠ 图像过暗, 需要增加曝光";
        else if (tMin >= 250) std::cout << " ⚠ 图像过曝, 需要减少曝光";
        std::cout << std::endl;
    }
    if (data.previewImgData) {
        unsigned char pMin = 255, pMax = 0;
        for (int i = 0; i < imgW * imgH; ++i) {
            if (data.previewImgData[i] < pMin) pMin = data.previewImgData[i];
            if (data.previewImgData[i] > pMax) pMax = data.previewImgData[i];
        }
        std::cout << "  预览图: " << (int)pMin << "~" << (int)pMax << std::endl;
    }

    // 打印中心点 + 四角
    int cx = imgW / 2, cy = imgH / 2;
    int idx_center = cy * imgW + cx;
    std::cout << "\n  中心点 (" << cx << "," << cy << "):  "
              << "(" << data.x[idx_center] << ", "
              << data.y[idx_center] << ", "
              << data.z[idx_center] << ")"
              << "  valid=" << (data.maskflag ? data.maskflag[idx_center] : -1)
              << std::endl;

    // ---------------------------------------------------------------
    // 第 5 步: 保存数据
    // ---------------------------------------------------------------
    std::string outDir = getOutputDir();
    std::cout << "\n--- 保存数据到 " << outDir << " ---" << std::endl;

    // 5a. 保存纹理图 (RGB 或灰度)
    if (data.textureData) {
        int texChannels = isRGB ? 3 : 1;
        std::string texFile = outDir + "/texture.bmp";
        saveBMP(texFile, data.textureData, imgW, imgH, texChannels);
    }

    // 5b. 保存深度图 (从 z 数据生成可视化深度图)
    if (data.z && data.maskflag) {
        // 只对有效点计算深度范围
        float zMin = 1e10f, zMax = -1e10f;
        int validCnt = 0;
        for (int i = 0; i < imgW * imgH; ++i) {
            if (data.maskflag[i]) {
                if (data.z[i] < zMin) zMin = data.z[i];
                if (data.z[i] > zMax) zMax = data.z[i];
                ++validCnt;
            }
        }
        if (validCnt > 0) {
            std::string depthFile = outDir + "/depth.bmp";
            saveDepthAsBMP(depthFile, data.z, imgW, imgH, zMin, zMax);

            // 额外保存深度图像数据 (如果存在)
            if (data.depthImageData) {
                std::string depthImgFile = outDir + "/depth_image.bmp";
                // depthImageData 是 SDK 内部算好的深度灰度图
                saveBMP(depthImgFile, data.depthImageData, imgW, imgH, 1);
            }
        }
    }

    // 5c. 保存预览图
    if (data.previewImgData) {
        std::string previewFile = outDir + "/preview.bmp";
        saveBMP(previewFile, data.previewImgData, imgW, imgH, 1);
    }

    // 5d. 保存点云
    if (data.x && data.y && data.z) {
        // XYZ 格式 (仅有效点)
        savePointCloudXYZ(outDir + "/pointcloud.xyz",
                          data.x, data.y, data.z,
                          data.maskflag, imgW, imgH, true);
        // TXT 格式 (全部点, 含 valid 标志)
        savePointCloudTXT(outDir + "/pointcloud.txt",
                          data.x, data.y, data.z,
                          data.maskflag, imgW, imgH);
        // GCI 原生格式
        gc3d::GC3DDevice::saveGCIP(data, outDir + "/pointcloud.gci");
        std::cout << "  [保存] " << outDir << "/pointcloud.gci" << std::endl;
    }

    // 5e. 保存法向信息 (如果存在)
    if (data.nx && data.ny && data.nz) {
        std::ofstream nf(outDir + "/normals.txt");
        for (int i = 0; i < imgW * imgH; ++i) {
            nf << data.nx[i] << " " << data.ny[i] << " " << data.nz[i] << "\n";
        }
        nf.close();
        std::cout << "  [保存] " << outDir << "/normals.txt" << std::endl;
    }

    // ---------------------------------------------------------------
    // 第 6 步: 获取网格化数据
    // ---------------------------------------------------------------
    std::cout << "\n--- 网格化数据 ---" << std::endl;
    gc3d::GC3DGridData gridData;
    ret = dev.getGC3DGridData(gridData);
    if (ret == GC3D_SUCCESS && gridData.depthImageData) {
        std::cout << "  网格尺寸: " << gridData.width << "x" << gridData.height
                  << "  dx=" << gridData.dx << "  dy=" << gridData.dy
                  << "  有效点数: " << gridData.validPointsNum << std::endl;

        // 保存网格深度图
        std::string gridFile = outDir + "/grid_depth.bmp";
        saveDepthAsBMP(gridFile, gridData.depthImageData,
                       gridData.width, gridData.height);

        // 保存网格纹理
        if (gridData.textureData) {
            int texCh = isRGB ? 3 : 1;
            std::string gridTexFile = outDir + "/grid_texture.bmp";
            saveBMP(gridTexFile, gridData.textureData,
                    gridData.width, gridData.height, texCh);
        }

        // 保存网格深度为 CSV
        std::ofstream gcsv(outDir + "/grid_depth.csv");
        gcsv << "x,y,depth,valid\n";
        for (int row = 0; row < gridData.height; ++row) {
            for (int col = 0; col < gridData.width; ++col) {
                int idx = row * gridData.width + col;
                float px = col * gridData.dx;
                float py = row * gridData.dy;
                gcsv << px << "," << py << ","
                     << gridData.depthImageData[idx] << ","
                     << (gridData.maskflag && gridData.maskflag[idx] ? "1" : "0")
                     << "\n";
            }
        }
        gcsv.close();
        std::cout << "  [保存] " << outDir << "/grid_depth.csv" << std::endl;

        // 保存深度数据
        gc3d::GC3DDevice::saveDepthData(gridData, outDir + "/grid_depth.txt");
        std::cout << "  [保存] " << outDir << "/grid_depth.txt" << std::endl;

    } else {
        std::cout << "  网格化数据不可用 (需要先 setNeedGridData(true))" << std::endl;
    }

    // ---------------------------------------------------------------
    // 第 7 步: 拍摄 2D 图像
    // ---------------------------------------------------------------
    std::cout << "\n--- 拍摄 2D 图像 ---" << std::endl;

    gc3d::GC3DImageData img2D;
    ret = dev.snapShot2D(img2D);
    if (ret == GC3D_SUCCESS && img2D.data && img2D.width > 0) {
        std::cout << "  2D 图像: " << img2D.width << "x" << img2D.height
                  << "  通道: " << img2D.channel << std::endl;
        std::string img2DFile = outDir + "/snapshot_2d.bmp";
        saveBMP(img2DFile, img2D.data, img2D.width, img2D.height, img2D.channel);
    } else {
        std::cout << "  2D 拍摄不可用或失败" << std::endl;
    }

    // ---------------------------------------------------------------
    // 第 8 步: 读取并打印更多设备信息
    // ---------------------------------------------------------------
    std::cout << "\n--- 设备信息 ---" << std::endl;
    gc3d::DeviceInformation devInfo;
    ret = dev.getDeviceInfo(devInfo);
    if (ret == GC3D_SUCCESS) {
        std::cout << "  序列号: " << devInfo.serialNum
                  << "\n  型号: " << devInfo.productType
                  << "\n  传感器: " << devInfo.sensorWidth << "x" << devInfo.sensorHeight;
    }
    std::cout << "\n  版本: " << gc3d::GC3DDevice::getGCIVersion()
              << "\n  扫描时间: " << (dev.getScanTime() / 1000.0) << " ms"
              << "\n  最大曝光: " << dev.getMaxExposureTime() << " us"
              << std::endl;

    // ---------------------------------------------------------------
    // 清理
    // ---------------------------------------------------------------
    std::cout << "\n--- 清理 ---" << std::endl;

    // 重要: 先关闭设备, 再反初始化
    ret = dev.closeDevice();
    checkError(ret, "closeDevice");

    ret = gc3d::unInitialDevice();
    checkError(ret, "unInitialDevice");

    std::cout << "\n 测试完成! 所有输出文件保存在: " << outDir << std::endl;
    std::cout << "  输出文件列表:\n"
              << "    texture.bmp         纹理图\n"
              << "    depth.bmp           深度图 (根据 Z 值渲染)\n"
              << "    depth_image.bmp     SDK 深度图\n"
              << "    preview.bmp         预览图\n"
              << "    pointcloud.xyz      点云 (仅有效点)\n"
              << "    pointcloud.txt      点云 (全部点, 含 valid 标志)\n"
              << "    pointcloud.gci      GCI 原生格式\n"
              << "    normals.txt         法向信息\n"
              << "    grid_depth.bmp      网格化深度图\n"
              << "    grid_texture.bmp    网格化纹理图\n"
              << "    grid_depth.csv      网格化深度 CSV\n"
              << "    snapshot_2d.bmp      2D 快照\n"
              << std::endl;

    return 0;
}
