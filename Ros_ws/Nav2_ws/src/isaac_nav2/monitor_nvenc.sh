#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Jetson NVENC 硬件编码实时监控${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查是否为 root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${YELLOW}提示: 某些监控功能需要 root 权限${NC}"
    echo -e "${YELLOW}建议使用: sudo $0${NC}"
    echo ""
fi

# 函数：检查 realsenseD435 进程
check_process() {
    PID=$(pgrep -f "realsenseD435" | head -1)
    if [ -n "$PID" ]; then
        echo -e "${GREEN}✅ 找到 realsenseD435 进程 (PID: $PID)${NC}"
        return 0
    else
        echo -e "${RED}❌ realsenseD435 进程未运行${NC}"
        return 1
    fi
}

# 函数：检查 NVENC 库加载
check_nvenc_libs() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}1. 检查进程加载的 NVIDIA 库${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if ! check_process; then
        return 1
    fi
    
    PID=$(pgrep -f "realsenseD435" | head -1)
    
    # 检查是否加载了 NVENC 相关库
    if [ "$EUID" -eq 0 ]; then
        NVENC_LIBS=$(lsof -p $PID 2>/dev/null | grep -E "nvidia-encode|nvcuvid|nvenc|libnv")
        if [ -n "$NVENC_LIBS" ]; then
            echo -e "${GREEN}✅ 硬件编码器库已加载:${NC}"
            echo "$NVENC_LIBS" | while read line; do
                echo -e "   ${GREEN}→${NC} $(echo $line | awk '{print $9}')"
            done
        else
            echo -e "${RED}❌ 未检测到 NVENC 库（可能使用软件编码）${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  需要 root 权限检查库加载情况${NC}"
        echo -e "   运行: ${YELLOW}sudo lsof -p $PID | grep nvidia${NC}"
    fi
}

# 函数：监控 CPU 使用率
monitor_cpu() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}2. 进程 CPU 使用率（5秒采样）${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if ! check_process; then
        return 1
    fi
    
    PID=$(pgrep -f "realsenseD435" | head -1)
    
    echo "   采样中..."
    CPU_USAGE=$(top -b -n 2 -d 2 -p $PID | tail -1 | awk '{print $9}')
    
    if [ -n "$CPU_USAGE" ]; then
        CPU_INT=$(echo $CPU_USAGE | cut -d. -f1)
        if [ "$CPU_INT" -lt 30 ]; then
            echo -e "   CPU 使用率: ${GREEN}${CPU_USAGE}%${NC} ${GREEN}✅ (可能使用硬件编码)${NC}"
        elif [ "$CPU_INT" -lt 60 ]; then
            echo -e "   CPU 使用率: ${YELLOW}${CPU_USAGE}%${NC} ${YELLOW}⚠️  (混合或部分硬件)${NC}"
        else
            echo -e "   CPU 使用率: ${RED}${CPU_USAGE}%${NC} ${RED}❌ (可能使用软件编码)${NC}"
        fi
        
        echo ""
        echo "   参考值:"
        echo "   • 硬件编码: 通常 < 30%"
        echo "   • 软件编码: 通常 > 60%"
    fi
}

# 函数：检查 tegrastats（NVENC 引擎）
monitor_tegrastats() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}3. Jetson 硬件编码引擎状态${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if ! command -v tegrastats &> /dev/null; then
        echo -e "${RED}❌ tegrastats 命令不可用（可能不是 Jetson 设备）${NC}"
        return 1
    fi
    
    if [ "$EUID" -ne 0 ]; then
        echo -e "${YELLOW}⚠️  需要 root 权限运行 tegrastats${NC}"
        echo -e "   运行: ${YELLOW}sudo tegrastats${NC}"
        return 1
    fi
    
    echo "   采样 5 秒..."
    TEGRA_OUTPUT=$(timeout 5 tegrastats --interval 1000 2>/dev/null)
    
    # 检查 NVENC 字段
    NVENC_FOUND=$(echo "$TEGRA_OUTPUT" | grep -o "NVENC [0-9]*" | tail -1)
    
    if [ -n "$NVENC_FOUND" ]; then
        NVENC_VALUE=$(echo "$NVENC_FOUND" | awk '{print $2}')
        if [ "$NVENC_VALUE" -gt 0 ]; then
            echo -e "${GREEN}✅ NVENC 引擎正在使用: ${NVENC_VALUE}${NC}"
            echo -e "   ${GREEN}→ 硬件编码器已激活！${NC}"
        else
            echo -e "${YELLOW}⚠️  NVENC 引擎空闲: ${NVENC_VALUE}${NC}"
            echo -e "   ${YELLOW}→ 硬件编码器未在使用${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  未检测到 NVENC 字段${NC}"
        echo -e "   可能原因:"
        echo -e "   • 硬件编码器未激活"
        echo -e "   • 设备不支持 NVENC"
        echo -e "   • tegrastats 版本不支持显示 NVENC"
    fi
    
    echo ""
    echo "   完整 tegrastats 输出示例:"
    echo "$TEGRA_OUTPUT" | tail -1 | sed 's/^/   /'
}

# 函数：检查 FFmpeg NVENC 支持
check_ffmpeg_nvenc() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}4. FFmpeg NVENC 支持检查${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if ! command -v ffmpeg &> /dev/null; then
        echo -e "${RED}❌ FFmpeg 未安装${NC}"
        return 1
    fi
    
    if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q "h264_nvenc"; then
        echo -e "${GREEN}✅ FFmpeg 支持 h264_nvenc${NC}"
    else
        echo -e "${RED}❌ FFmpeg 不支持 h264_nvenc${NC}"
        echo -e "   需要安装 Jetson-FFmpeg"
    fi
    
    if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q "h264_v4l2m2m"; then
        echo -e "${GREEN}✅ FFmpeg 支持 h264_v4l2m2m (V4L2)${NC}"
    fi
}

# 主监控循环
main_monitor() {
    while true; do
        clear
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}  Jetson NVENC 硬件编码实时监控${NC}"
        echo -e "${BLUE}  $(date '+%Y-%m-%d %H:%M:%S')${NC}"
        echo -e "${BLUE}========================================${NC}"
        
        check_nvenc_libs
        monitor_cpu
        monitor_tegrastats
        
        echo ""
        echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo -e "按 ${YELLOW}Ctrl+C${NC} 停止监控，等待 ${YELLOW}10 秒${NC}后刷新..."
        echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        
        sleep 10
    done
}

# 主程序
if [ "$1" = "--loop" ]; then
    # 循环监控模式
    main_monitor
else
    # 单次检查模式
    check_nvenc_libs
    monitor_cpu
    monitor_tegrastats
    check_ffmpeg_nvenc
    
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}提示:${NC}"
    echo -e "  • 运行 ${YELLOW}$0 --loop${NC} 进入实时监控模式"
    echo -e "  • 运行 ${YELLOW}sudo tegrastats${NC} 查看详细硬件状态"
    echo -e "${BLUE}========================================${NC}"
fi

