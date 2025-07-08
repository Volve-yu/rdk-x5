#!/bin/bash

echo "正在安装RDK X5视觉系统依赖..."

# 更新包列表
sudo apt update

# 安装基础开发工具
sudo apt install -y python3 python3-pip python3-dev
sudo apt install -y build-essential cmake pkg-config

# 安装OpenCV依赖
sudo apt install -y libopencv-dev python3-opencv
sudo apt install -y libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y libv4l-dev libxvidcore-dev libx264-dev

# 安装串口通信
sudo apt install -y python3-serial

# 安装二维码识别库
pip3 install pyzbar

# 安装ONNX运行时
pip3 install onnxruntime

# 安装音频相关
sudo apt install -y espeak espeak-data festival
sudo apt install -y alsa-utils pulseaudio

# 安装其他Python依赖
pip3 install numpy opencv-python pyserial queue

# 设置权限
sudo usermod -a -G dialout $USER
sudo usermod -a -G audio $USER

echo "依赖安装完成！"
echo "请重新登录以使权限更改生效。"
