/**
 * @file OpticalFlowFilter.h
 * @brief 基于光流的动态物体检测过滤器（v4.0 - 相机运动补偿版本）
 * @author ht & Claude Code
 * @date 2025-01-17
 *
 * 原理：
 * 1. 提取网格点进行LK光流跟踪
 * 2. 使用SLAM位姿估计进行相机运动补偿
 * 3. 计算残差光流（真实物体运动）
 * 4. 结合YOLO框进行加权判断
 * 5. 深度一致性验证
 * 6. 聚类异常运动区域为动态区域
 */

#ifndef OPTICAL_FLOW_FILTER_H
#define OPTICAL_FLOW_FILTER_H

#include "DynamicFilterBase.h"
#include <opencv2/video/tracking.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <deque>

namespace ORB_SLAM3 {

class OpticalFlowFilter : public DynamicFilterBase {
public:
    OpticalFlowFilter();
    ~OpticalFlowFilter();

    bool Initialize(const std::string& config_file = "") override;

    // ========== v4.0新增：支持相机运动补偿的检测接口 ==========
    std::vector<cv::Rect> DetectDynamicAreas(
        const cv::Mat& image,
        double timestamp
    ) override;

    /**
     * @brief v4.0核心接口：带相机运动补偿的动态检测
     * @param image 当前帧图像
     * @param timestamp 时间戳
     * @param yolo_boxes YOLO检测框（用于加权判断）
     * @param depth_map 深度图
     * @param camera_motion 相机运动（T_curr * T_prev^-1）
     * @param fx, fy, cx, cy 相机内参
     * @return 动态区域列表
     */
    std::vector<cv::Rect> DetectDynamicAreasWithMotionCompensation(
        const cv::Mat& image,
        double timestamp,
        const std::vector<cv::Rect>& yolo_boxes,
        const cv::Mat& depth_map,
        const Sophus::SE3f& camera_motion,
        float fx, float fy, float cx, float cy
    );

    std::string GetName() const override { return "OpticalFlow-MotionCompensated"; }

    cv::Mat GetVisualization() const override;

    // 设置参数的接口
    void SetMotionThreshold(float threshold) { motion_threshold_ = threshold; }
    void SetGridSize(int size) { grid_size_ = size; }
    void SetMinPointsPerRegion(int count) { min_points_per_region_ = count; }

private:
    // 上一帧数据
    cv::Mat prev_gray_;
    cv::Mat prev_depth_;               // v4.0新增：上一帧深度图
    std::vector<cv::Point2f> prev_points_;
    double prev_timestamp_;
    bool first_frame_;

    // v4.0新增：相机内参（用于相机运动补偿）
    float fx_, fy_, cx_, cy_;
    bool camera_params_initialized_;

    // 光流跟踪的点
    std::vector<cv::Point2f> tracked_points_;
    std::vector<cv::Point2f> flow_vectors_;
    std::vector<bool> is_dynamic_;

    // 可视化
    mutable cv::Mat visualization_;

    // 配置参数
    float motion_threshold_;           // 运动阈值（像素）
    int grid_size_;                    // 网格大小
    int min_points_per_region_;        // 每个动态区域最小点数
    float clustering_distance_;        // 聚类距离阈值
    float depth_consistency_threshold_; // v4.0新增：深度一致性阈值（米）

    // LK光流参数
    cv::Size lk_window_size_;
    int lk_max_level_;
    cv::TermCriteria lk_criteria_;

    /**
     * @brief v4.0核心：计算期望光流（相机运动导致的光流）
     * @param pt 像素坐标
     * @param depth 深度值
     * @param camera_motion 相机运动
     * @return 期望光流向量
     */
    cv::Point2f ComputeExpectedFlow(
        const cv::Point2f& pt,
        float depth,
        const Sophus::SE3f& camera_motion
    ) const;

    /**
     * @brief v4.0新增：深度一致性验证
     * @param pt_prev 上一帧点坐标
     * @param pt_curr 当前帧点坐标
     * @param depth_prev 上一帧深度图
     * @param depth_curr 当前帧深度图
     * @return true=深度一致，false=深度突变（遮挡或错误）
     */
    bool VerifyDepthConsistency(
        const cv::Point2f& pt_prev,
        const cv::Point2f& pt_curr,
        const cv::Mat& depth_prev,
        const cv::Mat& depth_curr
    ) const;

    /**
     * @brief 在图像上生成网格点
     */
    std::vector<cv::Point2f> GenerateGridPoints(
        const cv::Size& image_size,
        int grid_size
    );

    /**
     * @brief 计算光流
     */
    bool ComputeOpticalFlow(
        const cv::Mat& prev_gray,
        const cv::Mat& curr_gray,
        const std::vector<cv::Point2f>& prev_pts,
        std::vector<cv::Point2f>& curr_pts,
        std::vector<uchar>& status
    );

    /**
     * @brief 检测异常运动点（相对于相机运动）
     */
    std::vector<bool> DetectAbnormalMotion(
        const std::vector<cv::Point2f>& prev_pts,
        const std::vector<cv::Point2f>& curr_pts,
        const std::vector<uchar>& status
    );

    /**
     * @brief 聚类动态点为矩形区域
     */
    std::vector<cv::Rect> ClusterDynamicRegions(
        const std::vector<cv::Point2f>& points,
        const std::vector<bool>& is_dynamic,
        const cv::Size& image_size
    );

    /**
     * @brief 生成可视化图像
     */
    void GenerateVisualization(
        const cv::Mat& image,
        const std::vector<cv::Point2f>& prev_pts,
        const std::vector<cv::Point2f>& curr_pts,
        const std::vector<bool>& is_dynamic,
        const std::vector<cv::Rect>& dynamic_areas
    );
};

} // namespace ORB_SLAM3

#endif // OPTICAL_FLOW_FILTER_H
