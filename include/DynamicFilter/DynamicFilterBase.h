/**
 * @file DynamicFilterBase.h
 * @brief 动态特征点过滤器基类
 * @author ht & Claude Code
 * @date 2025-01-16
 *
 * 所有动态过滤方法（YOLO、光流、几何约束等）的统一接口
 */

#ifndef DYNAMIC_FILTER_BASE_H
#define DYNAMIC_FILTER_BASE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>

namespace ORB_SLAM3 {

class Frame;  // 前向声明

/**
 * @brief 动态特征点过滤器基类
 *
 * 设计理念：
 * - 每个过滤器独立工作
 * - 提供统一的接口
 * - 支持可视化调试
 * - 易于扩展新方法
 */
class DynamicFilterBase {
public:
    virtual ~DynamicFilterBase() {}

    /**
     * @brief 初始化过滤器
     * @param config_file 配置文件路径（可选）
     * @return 是否成功初始化
     */
    virtual bool Initialize(const std::string& config_file = "") { return true; }

    /**
     * @brief 检测动态区域
     *
     * 核心功能：给定一帧图像，返回可能包含动态物体的矩形区域
     *
     * @param image 输入图像（RGB或灰度）
     * @param timestamp 时间戳（某些方法需要时间信息）
     * @return 动态区域列表 (Rect)
     */
    virtual std::vector<cv::Rect> DetectDynamicAreas(
        const cv::Mat& image,
        double timestamp
    ) = 0;

    /**
     * @brief 过滤动态特征点（可选，基类提供默认实现）
     *
     * 根据动态区域，过滤掉落在这些区域内的特征点
     *
     * @param keypoints 输入特征点
     * @param dynamic_areas 动态区域
     * @return 过滤后的特征点索引 (保留的点的索引)
     */
    virtual std::vector<int> FilterKeypoints(
        const std::vector<cv::KeyPoint>& keypoints,
        const std::vector<cv::Rect>& dynamic_areas
    );

    /**
     * @brief 获取过滤器名称
     */
    virtual std::string GetName() const = 0;

    /**
     * @brief 获取可视化图像（用于调试）
     * @return 可视化图像，可以叠加在原图上显示
     */
    virtual cv::Mat GetVisualization() const { return cv::Mat(); }

    /**
     * @brief 是否启用此过滤器
     */
    virtual bool IsEnabled() const { return enabled_; }
    virtual void SetEnabled(bool enabled) { enabled_ = enabled; }

    /**
     * @brief 获取统计信息
     */
    virtual int GetDetectedRegionCount() const { return detected_region_count_; }
    virtual int GetFilteredPointCount() const { return filtered_point_count_; }

protected:
    bool enabled_ = true;
    std::string name_;

    // 统计信息
    int detected_region_count_ = 0;
    int filtered_point_count_ = 0;
};

/**
 * @brief 基类的默认实现：过滤特征点
 */
inline std::vector<int> DynamicFilterBase::FilterKeypoints(
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<cv::Rect>& dynamic_areas
) {
    std::vector<int> valid_indices;
    valid_indices.reserve(keypoints.size());

    for(size_t i = 0; i < keypoints.size(); i++) {
        const cv::Point2f& pt = keypoints[i].pt;
        bool is_dynamic = false;

        // 检查点是否落在任何动态区域内
        for(const auto& area : dynamic_areas) {
            if(area.contains(pt)) {
                is_dynamic = true;
                break;
            }
        }

        if(!is_dynamic) {
            valid_indices.push_back(i);
        }
    }

    filtered_point_count_ = keypoints.size() - valid_indices.size();
    return valid_indices;
}

} // namespace ORB_SLAM3

#endif // DYNAMIC_FILTER_BASE_H
