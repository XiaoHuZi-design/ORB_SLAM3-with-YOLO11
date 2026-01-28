/**
 * @file YoloFilter.h
 * @brief 基于YOLO的动态物体检测过滤器
 * @author ht & Claude Code
 * @date 2025-01-16
 *
 * 包装现有的YoloDetection实现，使其符合统一接口
 */

#ifndef YOLO_FILTER_H
#define YOLO_FILTER_H

#include "DynamicFilterBase.h"
#include "YoloDetect.h"  // 你现有的YOLO实现
#include <memory>

namespace ORB_SLAM3 {

class YoloFilter : public DynamicFilterBase {
public:
    YoloFilter();
    YoloFilter(YoloDetection* detector);  // 新增：接受外部检测器的构造函数
    ~YoloFilter();

    bool Initialize(const std::string& config_file = "") override;

    std::vector<cv::Rect> DetectDynamicAreas(
        const cv::Mat& image,
        double timestamp
    ) override;

    std::string GetName() const override { return "YOLO-v11"; }

    cv::Mat GetVisualization() const override;

    // 设置YOLO检测器实例（如果外部已创建）
    void SetDetector(YoloDetection* detector) { detector_ = detector; external_detector_ = true; }

    // 获取检测到的类别统计
    std::map<std::string, int> GetDetectedClasses() const { return class_count_; }

private:
    YoloDetection* detector_;
    bool external_detector_;  // 是否使用外部创建的检测器

    // 配置
    std::vector<std::string> dynamic_classes_;  // 动态物体类别列表
    int bbox_expansion_;  // 边界框扩展像素

    // 统计
    std::map<std::string, int> class_count_;

    // 可视化
    mutable cv::Mat visualization_;

    /**
     * @brief 检查类别是否为动态
     */
    bool IsDynamicClass(const std::string& class_name) const;

    /**
     * @brief 扩展边界框（像素方式）
     * @param bbox 原始边界框
     * @param image_size 图像尺寸
     * @param expand 向四周扩展的像素数
     * @note 当前未使用，YoloDetect.cpp中使用比例系数方式扩展
     */
    cv::Rect ExpandBBox(const cv::Rect& bbox, const cv::Size& image_size, int expand) const;
};

} // namespace ORB_SLAM3

#endif // YOLO_FILTER_H
