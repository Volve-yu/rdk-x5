import cv2
import numpy as np
import onnxruntime as ort
from config import ONNX_MODEL_PATH, COLOR_RANGES, SHAPE_CLASSES

class ShapeDetector:
    def __init__(self):
        self.onnx_session = None
        self.load_onnx_model()
    
    def load_onnx_model(self):
        """加载ONNX模型"""
        try:
            self.onnx_session = ort.InferenceSession(ONNX_MODEL_PATH)
            print(f"ONNX模型加载成功: {ONNX_MODEL_PATH}")
        except Exception as e:
            print(f"ONNX模型加载失败: {e}")
    
    def detect_colors(self, frame):
        """检测图像中的颜色"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_colors = []
        
        for color_name, color_range in COLOR_RANGES.items():
            lower = np.array(color_range['lower'])
            upper = np.array(color_range['upper'])
            
            # 创建颜色掩码
            mask = cv2.inRange(hsv, lower, upper)
            
            # 形态学操作去噪
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # 过滤小区域
                    x, y, w, h = cv2.boundingRect(contour)
                    detected_colors.append({
                        'color': color_name,
                        'contour': contour,
                        'bbox': (x, y, w, h),
                        'area': area,
                        'center': (x + w//2, y + h//2)
                    })
        
        return detected_colors
    
    def detect_shapes_with_onnx(self, roi):
        """使用ONNX模型检测形状"""
        if not self.onnx_session:
            return None
        
        try:
            # 预处理ROI
            resized_roi = cv2.resize(roi, (224, 224))  # 假设模型输入大小为224x224
            normalized_roi = resized_roi.astype(np.float32) / 255.0
            input_data = np.transpose(normalized_roi, (2, 0, 1))  # CHW格式
            input_data = np.expand_dims(input_data, axis=0)  # 添加batch维度
            
            # 推理
            input_name = self.onnx_session.get_inputs()[0].name
            output_name = self.onnx_session.get_outputs()[0].name
            
            result = self.onnx_session.run([output_name], {input_name: input_data})
            predictions = result[0][0]
            
            # 获取最高概率的类别
            predicted_class_idx = np.argmax(predictions)
            confidence = predictions[predicted_class_idx]
            
            if confidence > 0.5:  # 置信度阈值
                return SHAPE_CLASSES[predicted_class_idx]
        
        except Exception as e:
            print(f"ONNX推理出错: {e}")
        
        return None
    
    def detect_shapes_opencv(self, contour):
        """使用OpenCV检测形状（备用方法）"""
        # 计算轮廓周长
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 根据顶点数量判断形状
        vertices = len(approx)
        
        if vertices == 3:
            return "triangle"
        elif vertices == 4:
            # 检查是否为正方形
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            if 0.8 <= aspect_ratio <= 1.2:
                return "square"
            else:
                return "rectangle"
        elif vertices > 8:
            # 检查圆形
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity > 0.7:
                    return "circle"
        
        return "unknown"
    
    def detect_shapes(self, frame):
        """主要形状检测函数"""
        detected_objects = []
        
        # 首先检测颜色区域
        color_regions = self.detect_colors(frame)
        
        for color_region in color_regions:
            # 提取ROI
            x, y, w, h = color_region['bbox']
            roi = frame[y:y+h, x:x+w]
            
            # 尝试使用ONNX模型检测形状
            shape = self.detect_shapes_with_onnx(roi)
            
            # 如果ONNX失败，使用OpenCV方法
            if not shape:
                shape = self.detect_shapes_opencv(color_region['contour'])
            
            if shape and shape != "unknown":
                detected_objects.append({
                    'color': color_region['color'],
                    'shape': shape,
                    'bbox': color_region['bbox'],
                    'contour': color_region['contour'],
                    'center': color_region['center'],
                    'area': color_region['area']
                })
        
        return detected_objects
    
    def find_best_match(self, detected_shapes, expected_pattern):
        """找到最佳匹配的形状"""
        expected_parts = expected_pattern.split()
        if len(expected_parts) != 2:
            return None
        
        expected_color, expected_shape = expected_parts
        
        best_match = None
        best_score = 0
        
        for shape_obj in detected_shapes:
            # 计算匹配分数
            color_match = 1.0 if shape_obj['color'] == expected_color else 0.0
            shape_match = 1.0 if shape_obj['shape'] == expected_shape else 0.0
            
            # 综合评分（可以加入面积、位置等因素）
            score = (color_match + shape_match) / 2.0
            
            if score > best_score and score > 0.5:  # 至少50%匹配度
                best_score = score
                best_match = shape_obj
        
        return best_match
    
    def annotate_image(self, frame, shape_obj):
        """在图像上标注检测结果"""
        annotated_frame = frame.copy()
        
        # 绘制边界框
        x, y, w, h = shape_obj['bbox']
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # 绘制轮廓
        cv2.drawContours(annotated_frame, [shape_obj['contour']], -1, (255, 0, 0), 2)
        
        # 添加文字标签
        label = f"{shape_obj['color']} {shape_obj['shape']}"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.rectangle(annotated_frame, (x, y - label_size[1] - 10), 
                     (x + label_size[0], y), (0, 255, 0), -1)
        cv2.putText(annotated_frame, label, (x, y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # 添加中心点
        center = shape_obj['center']
        cv2.circle(annotated_frame, center, 5, (0, 0, 255), -1)
        
        return annotated_frame
