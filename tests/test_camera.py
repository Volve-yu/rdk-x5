import os
import json
import cv2
from datetime import datetime
from config import QR_DATA_FILE, CAPTURED_IMAGES_DIR

class StorageModule:
    def __init__(self):
        pass
    
    def save_qr_data(self, qr_data):
        """保存二维码数据"""
        try:
            data_to_save = {
                'qr_data': qr_data,
                'timestamp': datetime.now().isoformat(),
                'scan_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(QR_DATA_FILE, 'w', encoding='utf-8') as f:
                json.dump(data_to_save, f, ensure_ascii=False, indent=2)
            
            print(f"二维码数据已保存: {QR_DATA_FILE}")
            return True
            
        except Exception as e:
            print(f"保存二维码数据失败: {e}")
            return False
    
    def load_qr_data(self):
        """加载二维码数据"""
        try:
            if os.path.exists(QR_DATA_FILE):
                with open(QR_DATA_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                return data['qr_data']
        except Exception as e:
            print(f"加载二维码数据失败: {e}")
        return None
    
    def save_image(self, image, pattern_index, prefix="captured"):
        """保存图片"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{prefix}_pattern_{pattern_index + 1}_{timestamp}.jpg"
            image_path = os.path.join(CAPTURED_IMAGES_DIR, filename)
            
            # 保存图片
            cv2.imwrite(image_path, image)
            
            print(f"图片已保存: {image_path}")
            return image_path
            
        except Exception as e:
            print(f"保存图片失败: {e}")
            return None
    
    def save_comparison_result(self, comparison_result, captured_data):
        """保存比较结果"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            result_file = os.path.join(CAPTURED_IMAGES_DIR, f"comparison_result_{timestamp}.json")
            
            result_data = {
                'comparison_result': comparison_result,
                'captured_data': captured_data,
                'timestamp': datetime.now().isoformat(),
                'analysis_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(result_file, 'w', encoding='utf-8') as f:
                json.dump(result_data, f, ensure_ascii=False, indent=2)
            
            print(f"比较结果已保存: {result_file}")
            return True
            
        except Exception as e:
            print(f"保存比较结果失败: {e}")
            return False
    
    def create_summary_report(self, session_data):
        """创建总结报告"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            report_file = os.path.join(CAPTURED_IMAGES_DIR, f"session_report_{timestamp}.json")
            
            report = {
                'session_id': timestamp,
                'start_time': session_data.get('start_time'),
                'end_time': datetime.now().isoformat(),
                'qr_data': session_data.get('qr_data'),
                'total_captures': len(session_data.get('captured_data', [])),
                'comparison_results': session_data.get('comparison_results', []),
                'success_rate': self._calculate_success_rate(session_data),
                'performance_metrics': self._calculate_metrics(session_data)
            }
            
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
            
            print(f"会话报告已保存: {report_file}")
            return report_file
            
        except Exception as e:
            print(f"创建总结报告失败: {e}")
            return None
    
    def _calculate_success_rate(self, session_data):
        """计算成功率"""
        try:
            comparison_results = session_data.get('comparison_results', [])
            if not comparison_results:
                return 0.0
            
            successful = sum(1 for result in comparison_results if result.get('is_correct', False))
            return (successful / len(comparison_results)) * 100
            
        except:
            return 0.0
    
    def _calculate_metrics(self, session_data):
        """计算性能指标"""
        try:
            captured_data = session_data.get('captured_data', [])
            
            if not captured_data:
                return {}
            
            # 计算处理时间等指标
            timestamps = [item.get('timestamp') for item in captured_data if item.get('timestamp')]
            
            metrics = {
                'total_images_captured': len(captured_data),
                'average_processing_time': 'N/A',  # 需要实际测量
                'detection_accuracy': 'N/A',  # 需要实际测量
            }
            
            return metrics
            
        except:
            return {}
    
    def cleanup_old_files(self, days_to_keep=7):
        """清理旧文件"""
        try:
            import time
            current_time = time.time()
            cutoff_time = current_time - (days_to_keep * 24 * 60 * 60)
            
            for filename in os.listdir(CAPTURED_IMAGES_DIR):
                file_path = os.path.join(CAPTURED_IMAGES_DIR, filename)
                
                if os.path.isfile(file_path):
                    file_time = os.path.getmtime(file_path)
                    if file_time < cutoff_time:
                        os.remove(file_path)
                        print(f"已删除旧文件: {filename}")
            
            print(f"文件清理完成，保留{days_to_keep}天内的文件")
            
        except Exception as e:
            print(f"清理旧文件失败: {e}")
