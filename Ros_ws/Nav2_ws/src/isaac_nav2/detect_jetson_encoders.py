#!/usr/bin/env python3
"""
Jetson 硬件编码器检测脚本
检测 PyAV 和 FFmpeg 支持的所有 H.264 编码器
"""

import subprocess
import sys

def check_ffmpeg_encoders():
    """检查 FFmpeg 支持的编码器"""
    print("=" * 60)
    print("1. FFmpeg 支持的 H.264 编码器")
    print("=" * 60)
    
    try:
        result = subprocess.run(
            ['ffmpeg', '-hide_banner', '-encoders'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        output = result.stdout + result.stderr
        h264_encoders = []
        
        for line in output.split('\n'):
            if 'h264' in line.lower() and line.strip():
                print(f"  {line.strip()}")
                # 提取编码器名称
                parts = line.split()
                if len(parts) >= 2:
                    encoder_name = parts[1]
                    h264_encoders.append(encoder_name)
        
        print()
        return h264_encoders
        
    except Exception as e:
        print(f"  ❌ 无法运行 FFmpeg: {e}\n")
        return []

def check_pyav_encoders():
    """检查 PyAV 支持的编码器"""
    print("=" * 60)
    print("2. PyAV 可以使用的编码器")
    print("=" * 60)
    
    try:
        import av
        
        # 常见的 H.264 编码器列表
        encoders_to_test = [
            "h264_nvmpi",       # Jetson Multimedia API
            "h264_nvenc",       # NVIDIA NVENC
            "nvenc",            # NVENC 别名
            "nvenc_h264",       # NVENC H.264
            "h264_v4l2m2m",     # Video4Linux2
            "h264_omx",         # OpenMAX
            "h264_vaapi",       # VA-API
            "h264_qsv",         # Intel Quick Sync
            "libx264",          # 软件编码器
            "libx264rgb",       # x264 RGB
        ]
        
        available = []
        
        for encoder in encoders_to_test:
            try:
                codec = av.codec.Codec(encoder, 'w')
                print(f"  ✅ {encoder:20} - 可用")
                available.append(encoder)
            except Exception as e:
                print(f"  ❌ {encoder:20} - 不可用")
        
        print()
        return available
        
    except ImportError:
        print("  ❌ PyAV 未安装\n")
        return []
    except Exception as e:
        print(f"  ❌ 检查 PyAV 时出错: {e}\n")
        return []

def check_gstreamer():
    """检查 GStreamer 硬件编码支持"""
    print("=" * 60)
    print("3. GStreamer 硬件编码插件")
    print("=" * 60)
    
    try:
        result = subprocess.run(
            ['gst-inspect-1.0', 'nvv4l2h264enc'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            print("  ✅ nvv4l2h264enc 可用 (Jetson 硬件编码器)")
        else:
            print("  ❌ nvv4l2h264enc 不可用")
            
    except FileNotFoundError:
        print("  ⚠️  GStreamer 未安装")
    except Exception as e:
        print(f"  ❌ 检查失败: {e}")
    
    print()

def check_jetson_multimedia_api():
    """检查 Jetson Multimedia API"""
    print("=" * 60)
    print("4. Jetson Multimedia API")
    print("=" * 60)
    
    import os
    
    paths_to_check = [
        "/usr/lib/aarch64-linux-gnu/libnvv4l2.so",
        "/usr/lib/aarch64-linux-gnu/libnvbufsurface.so",
        "/usr/lib/aarch64-linux-gnu/libnvdsbufferpool.so",
    ]
    
    found = False
    for path in paths_to_check:
        if os.path.exists(path):
            print(f"  ✅ {path}")
            found = True
        else:
            print(f"  ❌ {path}")
    
    if found:
        print("\n  ℹ️  Jetson Multimedia API 库已安装")
    else:
        print("\n  ⚠️  Jetson Multimedia API 库未找到")
    
    print()

def check_jetson_ffmpeg():
    """检查 Jetson-FFmpeg 安装"""
    print("=" * 60)
    print("5. Jetson-FFmpeg 状态")
    print("=" * 60)
    
    try:
        result = subprocess.run(
            ['pkg-config', '--modversion', 'jetson-ffmpeg'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            version = result.stdout.strip()
            print(f"  ✅ Jetson-FFmpeg 已安装 (版本: {version})")
        else:
            print("  ❌ Jetson-FFmpeg 未安装")
            print("\n  安装方法:")
            print("    git clone https://github.com/jocover/jetson-ffmpeg.git")
            print("    cd jetson-ffmpeg")
            print("    mkdir build && cd build")
            print("    cmake ..")
            print("    make")
            print("    sudo make install")
            print("    sudo ldconfig")
            
    except FileNotFoundError:
        print("  ⚠️  pkg-config 未安装")
    except Exception as e:
        print(f"  ❌ 检查失败: {e}")
    
    print()

def provide_recommendations(pyav_encoders):
    """提供建议"""
    print("=" * 60)
    print("推荐配置")
    print("=" * 60)
    
    if not pyav_encoders:
        print("  ❌ PyAV 没有可用的硬件编码器")
        print("\n  建议:")
        print("  1. 重新编译 FFmpeg 并启用 Jetson 支持")
        print("  2. 重新安装 PyAV: pip install av --no-binary av")
        print("  3. 或考虑使用 GStreamer 替代方案")
        return
    
    # 硬件编码器优先级
    hardware_encoders = [enc for enc in pyav_encoders if enc != 'libx264' and enc != 'libx264rgb']
    
    if hardware_encoders:
        print(f"  ✅ 找到 {len(hardware_encoders)} 个硬件编码器")
        print("\n  推荐使用（按优先级）:")
        for i, enc in enumerate(hardware_encoders, 1):
            print(f"    {i}. {enc}")
        
        print(f"\n  在 realsenseD435.py 中配置:")
        print(f"    HARDWARE_ENCODERS = {hardware_encoders}")
    else:
        print("  ⚠️  仅软件编码器可用")
        print(f"    可用: {', '.join(pyav_encoders)}")

def main():
    print("\n" + "=" * 60)
    print("Jetson 硬件编码器检测工具")
    print("=" * 60 + "\n")
    
    # 检查系统信息
    try:
        with open('/etc/nv_tegra_release', 'r') as f:
            print(f"Jetson 信息: {f.read().strip()}\n")
    except:
        print("⚠️  警告: 这可能不是 Jetson 设备\n")
    
    # 执行各项检查
    ffmpeg_encoders = check_ffmpeg_encoders()
    pyav_encoders = check_pyav_encoders()
    check_gstreamer()
    check_jetson_multimedia_api()
    check_jetson_ffmpeg()
    
    # 提供建议
    provide_recommendations(pyav_encoders)
    
    print("=" * 60)
    print("检测完成")
    print("=" * 60 + "\n")

if __name__ == "__main__":
    main()

