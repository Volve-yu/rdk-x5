import serial
import threading
from queue import Queue
from config import SERIAL_PORT, SERIAL_BAUDRATE

class SerialModule:
    def __init__(self):
        self.serial_port = None
        self.command_queue = Queue()
        self.is_running = False
        self.read_thread = None
    
    def initialize(self):
        """初始化串口"""
        try:
            self.serial_port = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUDRATE,
                timeout=1
            )
            
            self.is_running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            print(f"串口初始化成功: {SERIAL_PORT}")
            return True
            
        except Exception as e:
            print(f"串口初始化失败: {e}")
            return False
    
    def _read_loop(self):
        """串口读取循环"""
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.command_queue.put(line)
                        print(f"[串口] 接收: {line}")
            except Exception as e:
                print(f"串口读取出错: {e}")
    
    def read_command(self):
        """读取命令"""
        if not self.command_queue.empty():
            return self.command_queue.get()
        return None
    
    def send_response(self, message):
        """发送响应"""
        try:
            if self.serial_port:
                self.serial_port.write(f"{message}\n".encode('utf-8'))
                print(f"[串口] 发送: {message}")
        except Exception as e:
            print(f"串口发送出错: {e}")
    
    def cleanup(self):
        """清理资源"""
        self.is_running = False
        if self.serial_port:
            self.serial_port.close()
        print("串口模块已清理")
