#!/bin/bash
echo "========================================"
echo "检查 NVENC 硬件编码器使用情况"
echo "========================================"
echo ""
echo "方法1: 使用 tegrastats 查看 NVENC 引擎使用率"
echo "运行以下命令实时监控（按 Ctrl+C 停止）："
echo "  sudo tegrastats"
echo ""
echo "查找输出中的 NVENC 字段，如果有数值说明硬件编码器在使用"
echo "示例输出: NVENC 450 (表示 NVENC 在工作)"
echo ""
echo "========================================"
echo ""
echo "方法2: 查看视频编码进程"
ps aux | grep -E "realsense|python.*D435" | grep -v grep
echo ""
echo "========================================"
echo ""
echo "方法3: 检查 NVENC 库是否被加载"
if [ -n "$(pgrep -f realsenseD435)" ]; then
    PID=$(pgrep -f realsenseD435)
    echo "找到进程 PID: $PID"
    echo "检查加载的 NVIDIA 库:"
    sudo lsof -p $PID 2>/dev/null | grep -E "nvidia-encode|nvcuvid|nvenc" || echo "  未检测到 NVENC 库（可能使用软件编码）"
else
    echo "未找到 realsenseD435 进程"
fi
echo ""
echo "========================================"
echo ""
echo "方法4: 快速测试当前是否使用硬件编码"
echo "运行完整测试脚本:"
echo "  python3 test_hardware_encoder.py"
