#ifndef SAVE_BMP_H
#define SAVE_BMP_H

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <algorithm>
#include <vector>

#pragma pack(push, 1)
struct BMPHeader {
    uint16_t bfType      = 0x4D42;   // "BM"
    uint32_t bfSize      = 0;
    uint16_t bfReserved1 = 0;
    uint16_t bfReserved2 = 0;
    uint32_t bfOffBits   = 54;

    uint32_t biSize          = 40;
    int32_t  biWidth         = 0;
    int32_t  biHeight        = 0;
    uint16_t biPlanes        = 1;
    uint16_t biBitCount      = 8;    // 8=灰度, 24=RGB
    uint32_t biCompression   = 0;
    uint32_t biSizeImage     = 0;
    int32_t  biXPelsPerMeter = 2835; // 72 DPI
    int32_t  biYPelsPerMeter = 2835;
    uint32_t biClrUsed       = 0;
    uint32_t biClrImportant  = 0;
};
#pragma pack(pop)

/// 保存灰度图 (8-bit) 或 RGB 图 (24-bit) 为 BMP 文件
inline bool saveBMP(const std::string& filename,
                    const unsigned char* data,
                    int width, int height,
                    int channels = 1) {
    if (!data || width <= 0 || height <= 0) return false;
    if (channels != 1 && channels != 3) return false;

    int bitCount = (channels == 1) ? 8 : 24;
    int rowSize = ((width * bitCount + 31) / 32) * 4;  // 每行对齐到 4 字节
    int pixelDataSize = rowSize * height;

    BMPHeader hdr;
    hdr.biWidth    = width;
    hdr.biHeight   = height;
    hdr.biBitCount = bitCount;
    hdr.biSizeImage = pixelDataSize;
    hdr.bfOffBits  = (bitCount == 8) ? 54 + 256 * 4 : 54;
    hdr.bfSize     = hdr.bfOffBits + pixelDataSize;

    FILE* fp = std::fopen(filename.c_str(), "wb");
    if (!fp) return false;

    std::fwrite(&hdr, sizeof(hdr), 1, fp);

    // 灰度图需要调色板 (256 级灰度)
    if (bitCount == 8) {
        for (int i = 0; i < 256; ++i) {
            uint8_t gray[4] = { (uint8_t)i, (uint8_t)i, (uint8_t)i, 0 };
            std::fwrite(gray, 1, 4, fp);
        }
    }

    // BMP 是 bottom-up 存储 (最后一行先写)
    std::vector<uint8_t> rowBuf(rowSize, 0);
    for (int y = height - 1; y >= 0; --y) {
        const unsigned char* src = data + y * width * channels;
        if (channels == 1) {
            std::memcpy(rowBuf.data(), src, width);
        } else {
            // RGB -> BGR 转换 (BMP 存储顺序是 BGR)
            for (int x = 0; x < width; ++x) {
                rowBuf[x * 3 + 0] = src[x * 3 + 2];  // B
                rowBuf[x * 3 + 1] = src[x * 3 + 1];  // G
                rowBuf[x * 3 + 2] = src[x * 3 + 0];  // R
            }
        }
        std::fwrite(rowBuf.data(), 1, rowSize, fp);
    }

    std::fclose(fp);
    return true;
}

/// 将 float 深度数组映射到 0-255 保存为灰度 BMP
/// depthFloat: 长度为 width*height 的 float 数组
/// minVal/maxVal: 映射范围, 若传 NaN 则自动取实际 min/max
inline bool saveDepthAsBMP(const std::string& filename,
                           const float* depthFloat,
                           int width, int height,
                           float minVal = -1.0f,
                           float maxVal = -1.0f) {
    if (!depthFloat || width <= 0 || height <= 0) return false;

    // 自动范围
    float dMin = minVal, dMax = maxVal;
    if (dMin >= dMax) {
        dMin = depthFloat[0];
        dMax = depthFloat[0];
        for (int i = 0; i < width * height; ++i) {
            if (depthFloat[i] < dMin) dMin = depthFloat[i];
            if (depthFloat[i] > dMax) dMax = depthFloat[i];
        }
    }
    float range = dMax - dMin;
    if (range < 1e-6f) range = 1.0f;

    std::vector<unsigned char> gray(width * height);
    for (int i = 0; i < width * height; ++i) {
        float v = (depthFloat[i] - dMin) / range;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        gray[i] = static_cast<unsigned char>(v * 255.0f);
    }

    return saveBMP(filename, gray.data(), width, height, 1);
}

#endif // SAVE_BMP_H
