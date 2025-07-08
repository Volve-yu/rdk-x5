#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=" * 50
echo "启动RDK X5视觉识别系统"
echo "项目目录: $SCRIPT_DIR"
echo "=" * 50

# 检查摄像头设备
if [ ! -e /dev/video0 ]; then
    echo "警告: 未检测到摄像头设备 /dev/video0"
    echo "可用的视频设备:"
    ls /dev/video* 2>/dev/null || echo "没有找到视频设备"
fi

# 检查串口设备
if [ ! -e /dev/ttyUSB0 ]; then
    echo "警告: 未检测到串口设备 /dev/ttyUSB0"
    echo "可用的串口设备:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "没有找到串口设备"
fi

# 检查ONNX模型
if [ ! -f "models/shape_model.onnx" ]; then
    echo "警告: ONNX模型文件不存在: models/shape_model.onnx"
    echo "请将训练好的模型文件放置在 models/ 目录下"
fi

# 检查Python依赖
echo "检查Python依赖..."
python3 -c "
import cv2, numpy, serial, pyzbar
print('基础依赖检查通过')
try:
    import onnxruntime
    print('ONNX运行时检查通过')
except:
    print('警告: ONNX运行时未安装')
"

# 设置环境变量
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

# 创建日志文件
LOG_FILE="data/logs/vision_system_$(date +%Y%m%d_%H%M%S).log"
echo "日志文件: $LOG_FILE"

# 启动主程序
echo "启动视觉系统..."
python3 main.py 2>&1 | tee "$LOG_FILE"

echo "视觉系统已退出"
echo "日志已保存到: $LOG_FILE"
