#!/usr/bin/env python3
import os
import sys
import time
import threading
from datetime import datetime

from config import *
from qr_module import QRModule
from serial_module import SerialModule
from camera_module import CameraModule
from shape_detection import ShapeDetector
from audio_module import AudioModule
from storage_module import StorageModule
from comparison_module import ComparisonModule

class VisionSystem:
    def __init__(self):
        self.qr_module = QRModule()
        self.serial_module = SerialModule()
        self.camera_module = CameraModule()
        self.shape_detector = ShapeDetector()
        self.audio_module = AudioModule()
        self.storage_module = StorageModule()
        self.comparison_module = ComparisonModule()
        
        self.qr_data = {}
        self.captured_data = []
        self.is_running = True
        
        self.log("视觉系统初始化完成")
    
    def log(self, message):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] {message}")
    
    def initialize_system(self):
        """初始化系统"""
        try:
            # 创建必要的目录
            os.makedirs(CAPTURED_IMAGES_DIR, exist_ok=True)
            if ENABLE_RECORDING:
                os.makedirs(RECORDING_DIR, exist_ok=True)
            
            # 初始化各模块
            self.camera_module.initialize()
            self.serial_module.initialize()
            self.audio_module.initialize()
            
            self.log("系统初始化成功")
            return True
        except Exception as e:
            self.log(f"系统初始化失败: {e}")
            return False
    
    def scan_qr_code(self):
        """扫描二维码获取图案信息"""
        self.log("开始扫描二维码...")
        
        # 拍摄图片
        frame = self.camera_module.capture_frame()
        if frame is None:
            self.log("无法获取摄像头画面")
            return False
        
        # 识别二维码
        qr_text = self.qr_module.decode_qr(frame)
        if qr_text:
            self.log(f"二维码识别成功: {qr_text}")
            
            # 解析二维码信息
            self.qr_data = self.qr_module.parse_qr_data(qr_text)
            
            # 存储二维码信息
            self.storage_module.save_qr_data(self.qr_data)
            
            # 语音播报
            self.audio_module.speak(f"二维码扫描完成，识别到{len(self.qr_data['patterns'])}个图案")
            
            return True
        else:
            self.log("未检测到二维码")
            return False
    
    def process_capture_command(self, command):
        """处理拍摄命令"""
        self.log(f"接收到拍摄命令: {command}")
        
        try:
            # 解析命令中的图案索引
            pattern_index = int(command.split()[-1]) - 1
            
            if 0 <= pattern_index < len(self.qr_data.get('patterns', [])):
                expected_pattern = self.qr_data['patterns'][pattern_index]
                self.log(f"开始拍摄第{pattern_index + 1}个图案: {expected_pattern}")
                
                # 拍摄并识别
                result = self.capture_and_analyze(expected_pattern, pattern_index)
                
                if result:
                    self.captured_data.append(result)
                    self.log(f"拍摄完成，已保存数据")
                    
                    # 如果拍摄了三次，进行比较
                    if len(self.captured_data) == 3:
                        self.perform_comparison()
                
            else:
                self.log(f"无效的图案索引: {pattern_index + 1}")
                
        except Exception as e:
            self.log(f"处理拍摄命令时出错: {e}")
    
    def capture_and_analyze(self, expected_pattern, pattern_index):
        """拍摄并分析图片"""
        # 拍摄图片
        frame = self.camera_module.capture_frame()
        if frame is None:
            self.log("无法获取摄像头画面")
            return None
        
        # 形状检测
        detected_shapes = self.shape_detector.detect_shapes(frame)
        
        if detected_shapes:
            # 选择最佳匹配的形状
            best_match = self.shape_detector.find_best_match(detected_shapes, expected_pattern)
            
            if best_match:
                # 在图片上标注
                annotated_frame = self.shape_detector.annotate_image(frame, best_match)
                
                # 保存图片
                image_path = self.storage_module.save_image(annotated_frame, pattern_index)
                
                # 语音播报
                shape_name = best_match['shape']
                color_name = best_match['color']
                self.audio_module.speak(f"识别到{color_name}{shape_name}")
                
                result = {
                    'expected': expected_pattern,
                    'detected': f"{color_name} {shape_name}",
                    'image_path': image_path,
                    'timestamp': datetime.now().isoformat(),
                    'pattern_index': pattern_index
                }
                
                return result
        
        self.log("未检测到有效图案")
        self.audio_module.speak("未检测到图案")
        return None
    
    def perform_comparison(self):
        """执行三次拍摄结果的比较"""
        self.log("开始进行三次拍摄结果比较...")
        
        comparison_result = self.comparison_module.compare_sequences(
            self.qr_data['patterns'], 
            [item['detected'] for item in self.captured_data]
        )
        
        if comparison_result['is_correct']:
            self.log("图案序列正确!")
            self.audio_module.speak("图案序列正确，检测完成")
        else:
            error_msg = comparison_result['error_message']
            self.log(f"图案序列错误: {error_msg}")
            self.audio_module.speak(f"警告！{error_msg}")
        
        # 保存比较结果
        self.storage_module.save_comparison_result(comparison_result, self.captured_data)
        
        # 重置数据准备下一轮
        self.captured_data = []
    
    def main_loop(self):
        """主循环"""
        self.log("进入主循环，等待指令...")
        
        while self.is_running:
            try:
                # 检查串口命令
                command = self.serial_module.read_command()
                
                if command:
                    self.log(f"收到串口指令: {command}")
                    
                    if command.startswith("SCAN_QR"):
                        self.scan_qr_code()
                    elif command.startswith("CAPTURE"):
                        self.process_capture_command(command)
                    elif command == "RESET":
                        self.reset_system()
                    elif command == "SHUTDOWN":
                        self.shutdown_system()
                
                time.sleep(0.1)  # 避免CPU占用过高
                
            except KeyboardInterrupt:
                self.log("接收到键盘中断信号")
                break
            except Exception as e:
                self.log(f"主循环出错: {e}")
                time.sleep(1)
    
    def reset_system(self):
        """重置系统"""
        self.log("重置系统...")
        self.qr_data = {}
        self.captured_data = []
        self.audio_module.speak("系统已重置")
    
    def shutdown_system(self):
        """关闭系统"""
        self.log("正在关闭系统...")
        self.is_running = False
        
        # 清理资源
        self.camera_module.cleanup()
        self.serial_module.cleanup()
        self.audio_module.cleanup()
        
        self.log("系统已关闭")

def main():
    print("=" * 50)
    print("RDK X5 视觉识别系统启动")
    print("=" * 50)
    
    vision_system = VisionSystem()
    
    if not vision_system.initialize_system():
        print("系统初始化失败，退出程序")
        return
    
    # 启动录像线程（如果启用）
    if ENABLE_RECORDING:
        recording_thread = threading.Thread(
            target=vision_system.camera_module.start_recording,
            daemon=True
        )
        recording_thread.start()
    
    try:
        vision_system.main_loop()
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        vision_system.shutdown_system()

if __name__ == "__main__":
    main()
