import os
import threading
from queue import Queue
import subprocess
from config import AUDIO_VOLUME, TTS_LANGUAGE

class AudioModule:
    def __init__(self):
        self.tts_queue = Queue()
        self.is_running = False
        self.tts_thread = None
    
    def initialize(self):
        """初始化音频模块"""
        try:
            # 检查音频设备
            result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
            if result.returncode != 0:
                print("警告: 未检测到音频播放设备")
            
            # 设置音量
            subprocess.run(['amixer', 'set', 'Master', f'{AUDIO_VOLUME}%'], 
                         capture_output=True)
            
            # 启动TTS处理线程
            self.is_running = True
            self.tts_thread = threading.Thread(target=self._tts_worker, daemon=True)
            self.tts_thread.start()
            
            print("音频模块初始化成功")
            return True
            
        except Exception as e:
            print(f"音频模块初始化失败: {e}")
            return False
    
    def speak(self, text):
        """添加语音播报任务到队列"""
        if text:
            self.tts_queue.put(text)
            print(f"[语音] 播报: {text}")
    
    def _tts_worker(self):
        """TTS工作线程"""
        while self.is_running:
            try:
                if not self.tts_queue.empty():
                    text = self.tts_queue.get()
                    self._synthesize_speech(text)
            except Exception as e:
                print(f"TTS工作线程出错: {e}")
    
    def _synthesize_speech(self, text):
        """合成语音"""
        try:
            # 方法1: 使用espeak（轻量级TTS）
            if TTS_LANGUAGE == 'zh':
                # 中文TTS
                subprocess.run([
                    'espeak', '-v', 'zh', '-s', '150', '-a', str(AUDIO_VOLUME), text
                ], check=True)
            else:
                # 英文TTS
                subprocess.run([
                    'espeak', '-v', 'en', '-s', '150', '-a', str(AUDIO_VOLUME), text
                ], check=True)
                
        except subprocess.CalledProcessError:
            # 方法2: 使用festival（备用）
            try:
                # 创建临时文件
                temp_file = '/tmp/tts_text.txt'
                with open(temp_file, 'w', encoding='utf-8') as f:
                    f.write(text)
                
                subprocess.run(['festival', '--tts', temp_file], check=True)
                os.remove(temp_file)
                
            except:
                # 方法3: 使用系统铃声作为提示音
                subprocess.run(['speaker-test', '-t', 'sine', '-f', '1000', '-l', '1'], 
                             capture_output=True)
                print(f"语音合成失败，使用提示音代替: {text}")
        
        except Exception as e:
            print(f"语音合成出错: {e}")
    
    def play_alert_sound(self):
        """播放警报声"""
        try:
            # 播放警报音调
            for freq in [800, 1000, 800, 1000]:
                subprocess.run([
                    'speaker-test', '-t', 'sine', '-f', str(freq), '-l', '1'
                ], capture_output=True, timeout=1)
        except:
            print("警报声播放失败")
    
    def cleanup(self):
        """清理资源"""
        self.is_running = False
        print("音频模块已清理")
