RDK X5 视觉识别系统
项目简介
基于D-Robotics RDK X5开发板的视觉识别系统，实现二维码识别、形状检测、语音播报等功能。

硬件要求
D-Robotics RDK X5开发板
USB摄像头
音频输出设备
串口连接（与树莓派通信）
安装步骤
克隆或下载项目到RDK开发板
运行安装脚本：./setup.sh
将训练好的ONNX模型文件放置在models/shape_model.onnx
使用方法
启动系统：./start_vision_system.sh
系统将等待串口指令
支持的指令：
SCAN_QR: 扫描二维码
CAPTURE 1/2/3: 拍摄对应图案
RESET: 重置系统
SHUTDOWN: 关闭系统
项目结构
rdk_vision_system/ ├── config.py # 配置文件 ├── main.py # 主程序 ├── *_module.py # 各功能模块 ├── models/ # 模型文件 ├── data/ # 数据存储 ├── captured_images/ # 拍摄图片 └── recordings/ # 录像文件

配置说明
主要配置在 中，包括：config.py

串口设备路径
摄像头参数
文件存储路径
音频设置
故障排除
摄像头无法使用：检查 设备权限/dev/video0
串口连接失败：确认串口设备路径和权限
音频播放失败：检查音频设备和音量设置
ONNX模型加载失败：确认模型文件存在且格式正确 完整的部署命令 在你的RDK开发板上执行：

1. 创建项目目录
cd /home/ubuntu mkdir rdk_vision_system CD rdk_vision_system

2. 下载或创建所有文件（将我提供的代码保存为对应的文件名）
3. 运行安装脚本
chmod +x setup.sh ./setup.sh

4. 放置ONNX模型文件（你需要提供）
cp your_shape_model.onnx 模型/shape_model.onnx
5. 启动系统
./start_vision_system.sh
