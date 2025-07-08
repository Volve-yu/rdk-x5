import cv2
import threading
import time
from datetime import datetime
from config import *

class CameraModule:
    def __init__(self):
        self.cap = None
        self.is_recording = False
        self.video_writer = None
    
    def initialize(self):
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(CAMERA_DEVICE)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            
            if not self.cap.isOpened():
                raise Exception("无法打开摄像头")
            
            print(f"摄像头初始化成功: {CAMERA_DEVICE}")
            return True
            
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            return False
    
    def capture_frame(self):
        """捕获单帧图像"""
        if not self.cap:
            return None
        
        ret, frame = self.cap.read()
        if ret:
            return frame
        return None
    
    def start_recording(self):
        """开始录像"""
        if not ENABLE_RECORDING or not self.cap:
            return
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_path = f"{RECORDING_DIR}/recording_{timestamp}.mp4"
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                video_path, fourcc, CAMERA_FPS, (CAMERA_WIDTH, CAMERA_HEIGHT)
            )
            
            self.is_recording = True
            print(f"开始录像: {video_path}")
            
            while self.is_recording:
                frame = self.capture_frame()
                if frame is not None:
                    self.video_writer.write(frame)
                time.sleep(1/CAMERA_FPS)
                
        except Exception as e:
            print(f"录像出错: {e}")
        finally:
            if self.video_writer:
                self.video_writer.release()
    
    def stop_recording(self):
        """停止录像"""
        self.is_recording = False
        print("录像已停止")
    
    def cleanup(self):
        """清理资源"""
        self.stop_recording()
        if self.cap:
            self.cap.release()
        print("摄像头模块已清理")
