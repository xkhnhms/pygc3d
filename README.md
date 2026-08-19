# GC3D Viewer Example (x64)

GCI 3D 结构光相机 SDK 示例程序, 演示如何调用 GC3D 库获取 RGB 纹理和深度图像并实时可视化。

## 依赖安装

### 编译工具

```bash
sudo apt install cmake build-essential
```

### CUDA Toolkit

GC3D 3D 重建依赖 CUDA (GPU 加速):

```bash
ls /usr/local/cuda/include/cuda.h
nvcc --version
```

### OpenCV

用于图像显示:

```bash
sudo apt install libopencv-dev
```

## 编译

```bash
cd GC3D_release_examples_x64
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译完成后, 可执行文件 `gc3d_viewer` 生成在本目录下。

> 在非 x86-64 主机 (如 aarch64) 上配置会直接报错终止 — 这是本工程的预期行为。

## 运行

### 前置条件

1. **x86-64 Linux 主机**, 已安装 CUDA + OpenCV (同上)
2. **连接 GCI 3D 相机** 
3. **GData 校准文件** — 程序运行时在 `./GData/` 目录下查找与相机序列号匹配的 `.gdata` 文件:
   - `HG*` 前缀 — 海康相机 
   - `DG*` 前缀 — 大恒相机 
   - 若相机序列号不在现有 GData 中, 请向厂商索取对应 `.gdata` / `.param` 放入 `./GData/`
4. **相机 SDK 运行时库** — 已内置在本工程 `dependencies/`, 无需手动配置
5. **GenTL 配置文件** — `.cti` 文件已在目录中, 用于相机发现和连接

### 启动

```bash
# 在工作目录下运行 (需要 GData/ 和 .cti 在当前目录)
cd GC3D_release_examples_x64
./gc3d_viewer
```

### 操作键位

| 按键 | 功能 |
|------|------|
| **Space / s** | 触发一次 3D 扫描 → 显示纹理图和伪彩色深度图 |
| **e** | 减小曝光时间 200μs |
| **E** | 增大曝光时间 200μs |
| **g** | 减小增益 0.5 |
| **G** | 增大增益 0.5 |
| **r** | 重置相机参数为默认值 |
| **ESC / q** | 退出程序 |

## 配置文件

`CommonParameters.ini` 存放 GC3D 相机默认参数 (x64 版调参), 程序启动时自动读取。修改后重新运行即可生效, 无需重新编译。



