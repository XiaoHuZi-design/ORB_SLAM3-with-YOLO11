/**
 * @file FilterManager.h
 * @brief 动态过滤器管理器（简化版）
 * @author ht & Claude Code
 * @date 2025-01-16
 *
 * 管理多个过滤器，支持灵活组合
 */

#ifndef FILTER_MANAGER_H
#define FILTER_MANAGER_H

#include "DynamicFilterBase.h"
#include <memory>
#include <map>
#include <vector>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

/**
 * @brief 动态过滤器管理器
 */
class FilterManager {
public:
    enum FilterStrategy {
        UNION,          // 并集：任一过滤器检测到即认为是动态
        INTERSECTION,   // 交集：所有过滤器都检测到才认为是动态
        CASCADE         // 级联：按顺序逐个过滤
    };

    FilterManager();
    ~FilterManager();

    /**
     * @brief 注册过滤器
     */
    void RegisterFilter(
        const std::string& name,
        std::shared_ptr<DynamicFilterBase> filter
    );

    /**
     * @brief 移除过滤器
     */
    void RemoveFilter(const std::string& name);

    /**
     * @brief 执行过滤（v3.0兼容接口 - 不带相机运动补偿）
     */
    std::vector<cv::Rect> FilterFrame(
        const cv::Mat& image,
        double timestamp
    );

    /**
     * @brief 执行过滤（v4.0新增 - 带相机运动补偿）
     * @param image 当前帧图像
     * @param timestamp 时间戳
     * @param depth_map 深度图
     * @param camera_motion 相机运动（T_curr * T_prev^-1）
     * @param fx, fy, cx, cy 相机内参
     * @return 动态区域列表
     */
    std::vector<cv::Rect> FilterFrameWithMotionCompensation(
        const cv::Mat& image,
        double timestamp,
        const cv::Mat& depth_map,
        const Sophus::SE3f& camera_motion,
        float fx, float fy, float cx, float cy
    );

    /**
     * @brief 设置过滤策略
     */
    void SetStrategy(FilterStrategy strategy) { strategy_ = strategy; }

    /**
     * @brief 启用/禁用特定过滤器
     */
    void EnableFilter(const std::string& name, bool enabled);

    /**
     * @brief 获取过滤器
     */
    std::shared_ptr<DynamicFilterBase> GetFilter(const std::string& name);

    /**
     * @brief 获取统计信息
     */
    std::map<std::string, int> GetStatistics() const;

    /**
     * @brief 获取所有可视化图像
     */
    std::map<std::string, cv::Mat> GetVisualizations() const;

    /**
     * @brief 获取每个过滤器的检测结果（用于可视化对比）
     */
    std::map<std::string, std::vector<cv::Rect>> GetIndividualResults() const;

private:
    std::map<std::string, std::shared_ptr<DynamicFilterBase>> filters_;
    FilterStrategy strategy_;

    // ========== 新增：存储每个过滤器的检测结果（用于可视化对比）==========
    mutable std::map<std::string, std::vector<cv::Rect>> individual_results_;

    /**
     * @brief 合并多个过滤器的结果（并集）
     */
    std::vector<cv::Rect> MergeUnion(
        const std::vector<std::vector<cv::Rect>>& all_areas
    );

    /**
     * @brief 合并多个过滤器的结果（交集）
     */
    std::vector<cv::Rect> MergeIntersection(
        const std::vector<std::vector<cv::Rect>>& all_areas
    );
};

} // namespace ORB_SLAM3

#endif // FILTER_MANAGER_H
