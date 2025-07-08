import os

# 项目根目录
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# Serial Communication
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际情况调整
SERIAL_BAUDRATE = 115200

# Camera Settings
CAMERA_DEVICE = '/dev/video0'  # USB摄像头设备
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# File Paths (使用绝对路径)
QR_DATA_FILE = os.path.join(PROJECT_ROOT, 'data', 'qr_data.json')
CAPTURED_IMAGES_DIR = os.path.join(PROJECT_ROOT, 'captured_images')
ONNX_MODEL_PATH = os.path.join(PROJECT_ROOT, 'models', 'shape_model.onnx')
RECORDING_DIR = os.path.join(PROJECT_ROOT, 'recordings')
LOG_DIR = os.path.join(PROJECT_ROOT, 'data', 'logs')

# Audio Settings
AUDIO_VOLUME = 80
TTS_LANGUAGE = 'zh'  # 中文语音

# Shape and Color Detection
SHAPE_CLASSES = ['circle', 'square', 'triangle']
COLOR_RANGES = {
    'red': {'lower': [0, 120, 70], 'upper': [10, 255, 255]},
    'blue': {'lower': [110, 50, 50], 'upper': [130, 255, 255]},
    'green': {'lower': [50, 50, 50], 'upper': [70, 255, 255]}
}

# System Settings
DEBUG_MODE = True
ENABLE_RECORDING = False  # 根据性能需求调整
MAX_RETRIES = 3

# 确保目录存在
for directory in [os.path.dirname(QR_DATA_FILE), CAPTURED_IMAGES_DIR, 
                  os.path.dirname(ONNX_MODEL_PATH), RECORDING_DIR, LOG_DIR]:
    os.makedirs(directory, exist_ok=True)
