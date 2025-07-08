#!/bin/bash

echo "设置RDK X5视觉识别系统..."

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    echo "请不要以root用户运行此脚本"
    exit 1
fi

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "项目目录: $SCRIPT_DIR"

# 创建必要的目录
echo "创建项目目录结构..."
mkdir -p models data data/logs captured_images recordings tests

# 设置权限
chmod +x install_dependencies.sh
chmod +x start_vision_system.sh

echo "安装Python依赖..."
pip3 install -r requirements.txt

# 运行依赖安装脚本
echo "安装系统依赖..."
./install_dependencies.sh

# 创建桌面快捷方式
echo "创建启动脚本..."
cat > ~/Desktop/start_vision.sh << EOF
#!/bin/bash
cd "$SCRIPT_DIR"
./start_vision_system.sh
EOF

chmod +x ~/Desktop/start_vision.sh

echo "设置完成！"
echo "项目位置: $SCRIPT_DIR"
echo "启动方式: cd $SCRIPT_DIR && ./start_vision_system.sh"
echo "或双击桌面上的 start_vision.sh"
