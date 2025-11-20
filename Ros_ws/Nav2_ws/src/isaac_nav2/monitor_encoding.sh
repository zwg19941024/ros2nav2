#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  实时编码监控${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 查找进程
PID=$(pgrep -f realsenseD435 | head -1)

if [ -z "$PID" ]; then
    echo -e "${RED}❌ realsenseD435 进程未运行${NC}"
    exit 1
fi

echo -e "${GREEN}✅ 找到进程 PID: $PID${NC}"
echo ""

# 循环监控
while true; do
    clear
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  硬件编码监控 - $(date '+%H:%M:%S')${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # CPU 使用率
    echo -e "${BLUE}📊 CPU 使用率:${NC}"
    CPU=$(top -b -n 1 -p $PID | tail -1 | awk '{print $9}')
    
    if [ -n "$CPU" ]; then
        CPU_INT=$(echo $CPU | cut -d. -f1)
        
        if [ "$CPU_INT" -lt 35 ]; then
            echo -e "   ${GREEN}$CPU%${NC} ${GREEN}✅ 硬件编码工作中${NC}"
            STATUS="硬件编码 (h264_v4l2m2m)"
        elif [ "$CPU_INT" -lt 50 ]; then
            echo -e "   ${YELLOW}$CPU%${NC} ${YELLOW}⚠️  可能混合编码${NC}"
            STATUS="混合模式"
        else
            echo -e "   ${RED}$CPU%${NC} ${RED}❌ 可能使用软件编码${NC}"
            STATUS="软件编码 (libx264)"
        fi
    else
        echo -e "   ${RED}无法获取 CPU 数据${NC}"
        STATUS="未知"
    fi
    
    echo ""
    
    # 内存使用
    echo -e "${BLUE}💾 内存使用:${NC}"
    MEM=$(top -b -n 1 -p $PID | tail -1 | awk '{print $10}')
    echo -e "   $MEM%"
    echo ""
    
    # 进程状态
    echo -e "${BLUE}🔧 进程信息:${NC}"
    echo -e "   PID: $PID"
    echo -e "   编码状态: $STATUS"
    echo ""
    
    # V4L2 设备检查
    echo -e "${BLUE}📹 V4L2 设备:${NC}"
    V4L2_DEVS=$(sudo lsof -p $PID 2>/dev/null | grep -E "video[0-9]+" | wc -l)
    if [ "$V4L2_DEVS" -gt 0 ]; then
        echo -e "   ${GREEN}✅ 使用了 $V4L2_DEVS 个 video 设备${NC}"
        sudo lsof -p $PID 2>/dev/null | grep -E "video[0-9]+" | awk '{print "   → " $9}'
    else
        echo -e "   ${YELLOW}⚠️  未检测到 V4L2 设备使用${NC}"
    fi
    echo ""
    
    # 判断总结
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    if [ "$CPU_INT" -lt 35 ]; then
        echo -e "${GREEN}✅ 硬件编码正常工作！${NC}"
    elif [ "$CPU_INT" -lt 50 ]; then
        echo -e "${YELLOW}⚠️  性能中等，请检查配置${NC}"
    else
        echo -e "${RED}❌ 可能未使用硬件编码${NC}"
        echo -e "${YELLOW}   检查程序日志中是否有 '🚀 HARDWARE ENCODER ACTIVATED!'${NC}"
    fi
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    echo ""
    echo -e "按 ${YELLOW}Ctrl+C${NC} 停止监控，${YELLOW}5 秒${NC}后刷新..."
    sleep 5
done

