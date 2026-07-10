# pygc3dAPI — GC3D 3D 结构光相机 Python SDK

GC3D 3D 结构光相机的 Python SDK，提供相机控制、3D 数据采集、参数配置等功能。

## 功能

- **设备管理** — 列出所有连接的 GC3D 相机，获取设备信息
- **3D 采集** — 采集纹理图 (RGB)、深度图 (Depth)、点云 (PointCloud)
- **文件保存** — RGB/Depth 保存为 PNG，点云保存为 PLY
- **参数控制** — 曝光、增益、高度范围、重建阈值、降噪、平滑等

## 安装

### 安装 wheel

```bash
pip install pygc3dAPI-0.0.1-cp310-cp310-manylinux2014_x86_64.whl
```

## 快速开始

```python
from pygc3dAPI import GC3DCamera, list_devices

# 列出设备
devices = list_devices()
for d in devices:
    print(f"  [{d['index']}] {d['product']}  S/N: {d['serial']}")

# 打开相机
cam = GC3DCamera()
cam.open(dev_index=0)

# 设置参数
cam.set_exposure(10000)           # 曝光 10000us
cam.set_height_range(500, 1200)   # 高度范围 500~1200mm

# 采集
rgb, depth, points = cam.capture()
print(f"  有效点数: {cam.valid_points}")

# 保存
cam.save_all("scan_01")           # → scan_01_rgb.png, scan_01_depth.png, scan_01.ply

# 关闭
cam.close()
```

## API 参考

### GC3DCamera 类

#### 相机控制

| 方法 | 说明 |
|---|---|
| `open(dev_index=0, config_file=None)` | 打开相机，`config_file` 默认为 `CommonParameters.ini` |
| `close()` | 关闭相机 |
| `capture()` | 采集一帧 → `(rgb, depth, pointcloud)` |

#### 属性

| 属性 | 类型 | 说明 |
|---|---|---|
| `is_opened` | bool | 相机是否已打开 |
| `serial` | str | 相机序列号 |
| `is_color` | bool | 是否为彩色相机 |
| `width` | int | 图像宽度 |
| `height` | int | 图像高度 |
| `valid_points` | int | 有效点云点数 |

#### 参数设置

| 方法 | 默认值 | 说明 |
|---|---|---|
| `set_exposure(us=800)` | 800us | 投影仪曝光时间 |
| `set_gain(gain=0.0)` | 0.0 | 增益 |
| `set_exposure_2d(exp=1000, gain=5.0)` | 1000us / 5.0 | 2D 相机曝光和增益 |
| `set_height_range(min_h=-1000, max_h=1000)` | -1000~1000mm | 重建高度范围 |
| `set_threshold(min_t=30, max_t=220)` | 30~220 | 重建阈值 (0~255) |
| `set_denoise(fmr=3, idx1=20, idx2=1, idx3=2)` | — | 降噪参数 |
| `set_smooth(smooth=0)` | 0 (关闭) | 平滑参数 |
| `set_grid(enable=False)` | 关闭 | 网格化数据输出 |
| `apply_config()` | — | 应用当前配置 |
| `print_config()` | — | 打印当前全部参数 |

#### 文件保存

| 方法 | 说明 |
|---|---|
| `save_rgb(path)` | 保存纹理图为 PNG |
| `save_depth(path)` | 保存深度图为 PNG |
| `save_pointcloud(path)` | 保存点云为 ASCII PLY |
| `save_all(prefix)` | 同时保存以上三种 |


