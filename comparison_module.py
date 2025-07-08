class ComparisonModule:
    def __init__(self):
        pass
    
    def compare_sequences(self, expected_patterns, detected_patterns):
        """比较期望序列和检测序列"""
        try:
            if len(expected_patterns) != len(detected_patterns):
                return {
                    'is_correct': False,
                    'error_message': f"序列长度不匹配: 期望{len(expected_patterns)}个，检测到{len(detected_patterns)}个",
                    'expected_sequence': expected_patterns,
                    'detected_sequence': detected_patterns,
                    'detailed_comparison': []
                }
            
            detailed_comparison = []
            errors = []
            
            for i, (expected, detected) in enumerate(zip(expected_patterns, detected_patterns)):
                is_match = expected.lower() == detected.lower()
                
                comparison_item = {
                    'position': i + 1,
                    'expected': expected,
                    'detected': detected,
                    'is_correct': is_match
                }
                
                detailed_comparison.append(comparison_item)
                
                if not is_match:
                    errors.append(f"第{i + 1}个位置错误: 期望'{expected}'，检测到'{detected}'")
            
            is_all_correct = len(errors) == 0
            
            if is_all_correct:
                message = "所有图案序列正确匹配"
            else:
                message = "发现序列错误: " + "; ".join(errors)
            
            return {
                'is_correct': is_all_correct,
                'error_message': message,
                'expected_sequence': expected_patterns,
                'detected_sequence': detected_patterns,
                'detailed_comparison': detailed_comparison,
                'error_count': len(errors),
                'accuracy_percentage': ((len(expected_patterns) - len(errors)) / len(expected_patterns)) * 100
            }
            
        except Exception as e:
            return {
                'is_correct': False,
                'error_message': f"比较过程出错: {e}",
                'expected_sequence': expected_patterns,
                'detected_sequence': detected_patterns,
                'detailed_comparison': []
            }
    
    def analyze_error_patterns(self, comparison_results):
        """分析错误模式"""
        try:
            error_analysis = {
                'total_comparisons': len(comparison_results),
                'successful_comparisons': 0,
                'failed_comparisons': 0,
                'common_errors': {},
                'accuracy_trend': []
            }
            
            for result in comparison_results:
                if result['is_correct']:
                    error_analysis['successful_comparisons'] += 1
                else:
                    error_analysis['failed_comparisons'] += 1
                    
                    # 统计常见错误
                    for detail in result.get('detailed_comparison', []):
                        if not detail['is_correct']:
                            error_key = f"{detail['expected']} -> {detail['detected']}"
                            error_analysis['common_errors'][error_key] = \
                                error_analysis['common_errors'].get(error_key, 0) + 1
                
                # 记录准确率趋势
                error_analysis['accuracy_trend'].append(result.get('accuracy_percentage', 0))
            
            return error_analysis
            
        except Exception as e:
            print(f"错误模式分析失败: {e}")
            return {}
