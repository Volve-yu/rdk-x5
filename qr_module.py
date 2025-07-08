import cv2
import json
from pyzbar import pyzbar

class QRModule:
    def __init__(self):
        self.qr_decoder = cv2.QRCodeDetector()
    
    def decode_qr(self, frame):
        """解码二维码"""
        try:
            # 使用opencv解码
            data, bbox, _ = self.qr_decoder.detectAndDecode(frame)
            
            if data:
                return data
            
            # 备用：使用pyzbar解码
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_codes = pyzbar.decode(gray)
            
            if qr_codes:
                return qr_codes[0].data.decode('utf-8')
                
        except Exception as e:
            print(f"二维码解码出错: {e}")
        
        return None
    
    def parse_qr_data(self, qr_text):
        """解析二维码文本数据"""
        try:
            # 假设格式: "红色圆形 蓝色正方形 绿色三角形"
            patterns = qr_text.strip().split()
            
            parsed_patterns = []
            for i in range(0, len(patterns), 2):
                if i + 1 < len(patterns):
                    color = patterns[i]
                    shape = patterns[i + 1]
                    parsed_patterns.append(f"{color} {shape}")
            
            return {
                'raw_text': qr_text,
                'patterns': parsed_patterns,
                'pattern_count': len(parsed_patterns)
            }
            
        except Exception as e:
            print(f"解析二维码数据出错: {e}")
            return None
