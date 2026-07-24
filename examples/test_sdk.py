"""
GC3D SDK 测试用例

测试所有 SDK 功能:
  1. 列出设备
  2. 打开/关闭相机
  3. 采集 RGB + Depth + PointCloud
  4. 保存 PNG / PLY
  5. 设置相机参数 (曝光、增益、高度范围、阈值、降噪)
  6. 异常处理

用法:
  python test_sdk.py
"""

import sys
import os
import numpy as np

from pygc3dAPI import GC3DCamera, list_devices

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="GC3D SDK 测试工具")
    parser.add_argument("--list", action="store_true", help="列出设备")
    parser.add_argument("-e", "--exposure", type=int, default=10000, help="曝光 (us)")
    parser.add_argument("--min-h", type=float, default=500, help="最小高度 (mm)")
    parser.add_argument("--max-h", type=float, default=1200, help="最大高度 (mm)")
    parser.add_argument("-o", "--output", default="gc3d_sdk_scan", help="输出前缀")
    args = parser.parse_args()

    # 列出设备
    devices = list_devices()
    if not devices:
        print("未检测到相机")
        sys.exit(0)

    print(f"发现 {len(devices)} 台相机:")
    for d in devices:
        print(f"  [{d['index']}] {d['product']}  S/N: {d['serial']}")

    if args.list:
        sys.exit(0)

    # 测试: 打开 → 设参数 → 采集 → 保存 → 关闭
    print("\n" + "=" * 50)
    print("  SDK 测试: 打开 → 设置参数 → 采集 → 保存")
    print("=" * 50)

    try:
        cam = GC3DCamera()
        cam.open(dev_index=0)
        print(f"相机: {cam.serial}  彩色: {cam.is_color}")

        print("\n[参数设置]")
        cam.set_exposure(args.exposure)
        cam.set_height_range(args.min_h, args.max_h)
        cam.set_threshold(30, 200)
        cam.print_config()

        print("\n[采集]")
        rgb, depth, points, raw_points = cam.capture()
        print(f"  有效点数: {cam.valid_points}  / {cam.width}x{cam.height}")

        if rgb is not None:
            print(f"  纹理图: {rgb.shape}  [{rgb.min()}, {rgb.max()}]")
        if depth is not None:
            print(f"  深度图: {depth.shape}  [{depth.min()}, {depth.max()}]")
        if points is not None and len(points) > 0:
            print(f"  点云: {len(points)} 个点")
        if raw_points is not None and len(raw_points) > 0:
            print(f"  原始点云: {len(raw_points)} 个点 (含无效点)")

        print(f"\n[保存] 前缀: {args.output}")
        cam.save_all(args.output)

        cam.close()
        print("\n 测试完成")

    except Exception as e:
        print(f"\n 测试失败: {e}")
        sys.exit(1)
