/**
 * @file OpticalFlowFilter.cpp
 * @brief 基于光流的动态物体检测过滤器实现
 * @author ht & Claude Code
 * @date 2025-01-16
 */

#include "DynamicFilter/OpticalFlowFilter.h"
#include <algorithm>
#include <iostream>

using namespace std;
using namespace cv;

namespace ORB_SLAM3 {

// OpticalFlowFilter::OpticalFlowFilter()
//     : first_frame_(true),
//       prev_timestamp_(0.0),
//       motion_threshold_(2.5),        // 默认2.5像素
//       grid_size_(20),                 // 默认20像素网格
//       min_points_per_region_(5),      // 最少5个点
//       clustering_distance_(50.0),     // 聚类距离50像素
//       lk_window_size_(21, 21),
//       lk_max_level_(3),
//       lk_criteria_(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
//       camera_params_initialized_(false),  // v4.0新增
//       depth_consistency_threshold_(0.5)   // v4.0新增：深度一致性阈值0.5m
// {
//     name_ = "OpticalFlow-LK";
// }

OpticalFlowFilter::OpticalFlowFilter()
    : first_frame_(true),
      prev_timestamp_(0.0),
      motion_threshold_(1.2),        // ✅ 从2.5降到1.2（更敏感）
      grid_size_(15),                // ✅ 从20降到15（更密集）
      min_points_per_region_(3),     // ✅ 从5降到3（更宽松）
      clustering_distance_(25.0),    // ✅ 从50降到25（更精确）
      lk_window_size_(21, 21),
      lk_max_level_(3),
      lk_criteria_(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
      camera_params_initialized_(false),
      depth_consistency_threshold_(0.5)
{
    name_ = "OpticalFlow-LK";
}

OpticalFlowFilter::~OpticalFlowFilter() {}

bool OpticalFlowFilter::Initialize(const string& config_file) {
    cout << "[OpticalFlowFilter] Initializing..." << endl;

    // TODO: 如果提供了配置文件，从文件读取参数
    // 现在使用默认参数

    cout << "[OpticalFlowFilter] Parameters:" << endl;
    cout << "  - Motion threshold: " << motion_threshold_ << " pixels" << endl;
    cout << "  - Grid size: " << grid_size_ << " pixels" << endl;
    cout << "  - Min points per region: " << min_points_per_region_ << endl;
    cout << "  - LK window size: " << lk_window_size_.width << "x" << lk_window_size_.height << endl;

    return true;
}

vector<Rect> OpticalFlowFilter::DetectDynamicAreas(
    const Mat& image,
    double timestamp
) {
    if(!enabled_) {
        return vector<Rect>();
    }

    // 转换为灰度图
    Mat gray;
    if(image.channels() == 3) {
        cvtColor(image, gray, COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    // 第一帧：初始化
    if(first_frame_) {
        prev_gray_ = gray.clone();
        prev_points_ = GenerateGridPoints(gray.size(), grid_size_);
        prev_timestamp_ = timestamp;
        first_frame_ = false;

        visualization_ = image.clone();
        detected_region_count_ = 0;
        return vector<Rect>();
    }

    // 计算光流
    vector<Point2f> curr_points;
    vector<uchar> status;

    bool success = ComputeOpticalFlow(prev_gray_, gray, prev_points_, curr_points, status);

    if(!success) {
        cout << "[OpticalFlowFilter] Failed to compute optical flow" << endl;
        prev_gray_ = gray.clone();
        prev_points_ = GenerateGridPoints(gray.size(), grid_size_);
        prev_timestamp_ = timestamp;
        return vector<Rect>();
    }

    // 检测异常运动
    vector<bool> is_dynamic = DetectAbnormalMotion(prev_points_, curr_points, status);

    // 聚类动态区域
    vector<Rect> dynamic_areas = ClusterDynamicRegions(curr_points, is_dynamic, gray.size());

    // 生成可视化
    GenerateVisualization(image, prev_points_, curr_points, is_dynamic, dynamic_areas);

    // 更新历史
    prev_gray_ = gray.clone();
    prev_points_ = GenerateGridPoints(gray.size(), grid_size_);
    prev_timestamp_ = timestamp;

    detected_region_count_ = dynamic_areas.size();

    return dynamic_areas;
}

vector<Point2f> OpticalFlowFilter::GenerateGridPoints(
    const Size& image_size,
    int grid_size
) {
    vector<Point2f> points;

    for(int y = grid_size; y < image_size.height - grid_size; y += grid_size) {
        for(int x = grid_size; x < image_size.width - grid_size; x += grid_size) {
            points.push_back(Point2f(x, y));
        }
    }

    return points;
}

bool OpticalFlowFilter::ComputeOpticalFlow(
    const Mat& prev_gray,
    const Mat& curr_gray,
    const vector<Point2f>& prev_pts,
    vector<Point2f>& curr_pts,
    vector<uchar>& status
) {
    if(prev_pts.empty()) {
        return false;
    }

    vector<float> err;

    try {
        calcOpticalFlowPyrLK(
            prev_gray, curr_gray,
            prev_pts, curr_pts,
            status, err,
            lk_window_size_,
            lk_max_level_,
            lk_criteria_
        );
    } catch(cv::Exception& e) {
        cerr << "[OpticalFlowFilter] OpenCV exception: " << e.what() << endl;
        return false;
    }

    return true;
}

vector<bool> OpticalFlowFilter::DetectAbnormalMotion(
    const vector<Point2f>& prev_pts,
    const vector<Point2f>& curr_pts,
    const vector<uchar>& status
) {
    vector<bool> is_dynamic(prev_pts.size(), false);

    if(prev_pts.size() != curr_pts.size() || prev_pts.size() != status.size()) {
        return is_dynamic;
    }

    // 计算所有成功跟踪点的运动向量
    vector<Point2f> motion_vectors;
    vector<float> motion_magnitudes;

    for(size_t i = 0; i < prev_pts.size(); i++) {
        if(status[i]) {
            Point2f motion = curr_pts[i] - prev_pts[i];
            motion_vectors.push_back(motion);
            motion_magnitudes.push_back(norm(motion));
        } else {
            motion_vectors.push_back(Point2f(0, 0));
            motion_magnitudes.push_back(0.0f);
        }
    }

    // 计算运动的中位数（假设大部分点属于静态背景）
    vector<float> sorted_magnitudes = motion_magnitudes;
    sort(sorted_magnitudes.begin(), sorted_magnitudes.end());

    float median_motion = 0.0f;
    if(!sorted_magnitudes.empty()) {
        size_t mid = sorted_magnitudes.size() / 2;
        median_motion = sorted_magnitudes[mid];
    }

    // 标记异常运动的点
    for(size_t i = 0; i < prev_pts.size(); i++) {
        if(!status[i]) {
            is_dynamic[i] = false;
            continue;
        }

        float motion_mag = motion_magnitudes[i];

        // 如果运动显著大于中位数，认为是动态
        if(motion_mag > median_motion + motion_threshold_) {
            is_dynamic[i] = true;
        }
    }

    return is_dynamic;
}

vector<Rect> OpticalFlowFilter::ClusterDynamicRegions(
    const vector<Point2f>& points,
    const vector<bool>& is_dynamic,
    const Size& image_size
) {
    vector<Rect> regions;

    // 收集所有动态点
    vector<Point2f> dynamic_points;
    for(size_t i = 0; i < points.size(); i++) {
        if(is_dynamic[i]) {
            dynamic_points.push_back(points[i]);
        }
    }

    if(dynamic_points.size() < (size_t)min_points_per_region_) {
        return regions;  // 动态点太少，不认为有动态物体
    }

    // 简单的聚类：使用boundingRect
    // TODO: 可以使用更复杂的聚类算法（DBSCAN等）

    // 方法：将动态点分组，距离近的归为一组
    vector<vector<Point2f>> clusters;
    vector<bool> visited(dynamic_points.size(), false);

    for(size_t i = 0; i < dynamic_points.size(); i++) {
        if(visited[i]) continue;

        vector<Point2f> cluster;
        cluster.push_back(dynamic_points[i]);
        visited[i] = true;

        // 找到所有距离小于阈值的点
        for(size_t j = 0; j < dynamic_points.size(); j++) {
            if(visited[j]) continue;

            float dist = norm(dynamic_points[i] - dynamic_points[j]);
            if(dist < clustering_distance_) {
                cluster.push_back(dynamic_points[j]);
                visited[j] = true;
            }
        }

        if(cluster.size() >= (size_t)min_points_per_region_) {
            clusters.push_back(cluster);
        }
    }

    // 为每个簇创建包围盒
    for(const auto& cluster : clusters) {
        Rect bbox = boundingRect(cluster);

        // 扩展边界框
        int expand = 20;  // 扩展20像素
        bbox.x = max(0, bbox.x - expand);
        bbox.y = max(0, bbox.y - expand);
        bbox.width = min(image_size.width - bbox.x, bbox.width + 2 * expand);
        bbox.height = min(image_size.height - bbox.y, bbox.height + 2 * expand);

        regions.push_back(bbox);
    }

    return regions;
}

void OpticalFlowFilter::GenerateVisualization(
    const Mat& image,
    const vector<Point2f>& prev_pts,
    const vector<Point2f>& curr_pts,
    const vector<bool>& is_dynamic,
    const vector<Rect>& dynamic_areas
) {
    visualization_ = image.clone();

    if(visualization_.channels() == 1) {
        cvtColor(visualization_, visualization_, COLOR_GRAY2BGR);
    }

    // 绘制光流向量
    for(size_t i = 0; i < prev_pts.size() && i < curr_pts.size(); i++) {
        if(is_dynamic[i]) {
            // 动态点：红色箭头
            line(visualization_, prev_pts[i], curr_pts[i], Scalar(0, 0, 255), 2);
            circle(visualization_, curr_pts[i], 3, Scalar(0, 0, 255), -1);
        } else {
            // 静态点：绿色小点
            circle(visualization_, curr_pts[i], 1, Scalar(0, 255, 0), -1);
        }
    }

    // 绘制动态区域边界框
    for(const auto& area : dynamic_areas) {
        rectangle(visualization_, area, Scalar(255, 0, 0), 2);

        // 添加标签
        string label = "Dynamic";
        putText(visualization_, label, Point(area.x, area.y - 5),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
    }

    // 添加信息文字
    string info = "LK Flow | Regions: " + to_string(dynamic_areas.size());
    putText(visualization_, info, Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
}

Mat OpticalFlowFilter::GetVisualization() const {
    return visualization_;
}

// ========== v4.0新增：带相机运动补偿的动态检测 ==========

vector<Rect> OpticalFlowFilter::DetectDynamicAreasWithMotionCompensation(
    const Mat& image,
    double timestamp,
    const vector<Rect>& yolo_boxes,
    const Mat& depth_map,
    const Sophus::SE3f& camera_motion,
    float fx, float fy, float cx, float cy
) {
    if(!enabled_) {
        return vector<Rect>();
    }

    // 保存相机内参
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    camera_params_initialized_ = true;

    // 转换为灰度图
    Mat gray;
    if(image.channels() == 3) {
        cvtColor(image, gray, COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    // 第一帧：初始化
    if(first_frame_) {
        prev_gray_ = gray.clone();
        prev_depth_ = depth_map.clone();
        prev_points_ = GenerateGridPoints(gray.size(), grid_size_);
        prev_timestamp_ = timestamp;
        first_frame_ = false;

        visualization_ = image.clone();
        detected_region_count_ = 0;
        return yolo_boxes;  // 第一帧返回YOLO框
    }

    // ========== 步骤1：相机运动有效性检查 ==========
    Eigen::Vector3f translation = camera_motion.translation();
    float motion_magnitude = translation.norm();

    // 如果相机运动过大（跟踪丢失）或位姿无效，回退到YOLO
    if(motion_magnitude > 1.0 || motion_magnitude < 0.0001) {
        cout << "[OpticalFlowFilter] Camera motion异常 (" << motion_magnitude
             << "m), 回退到YOLO模式" << endl;
        prev_gray_ = gray.clone();
        prev_depth_ = depth_map.clone();
        return yolo_boxes;
    }

    // ========== 步骤2：计算LK光流 ==========
    vector<Point2f> curr_points;
    vector<uchar> status;
    vector<float> err;

    calcOpticalFlowPyrLK(
        prev_gray_, gray,
        prev_points_, curr_points,
        status, err,
        lk_window_size_, lk_max_level_,
        lk_criteria_
    );

    if(curr_points.empty()) {
        prev_gray_ = gray.clone();
        prev_depth_ = depth_map.clone();
        return yolo_boxes;
    }

    // ========== 步骤3：相机运动补偿 + 深度验证 ==========
    vector<Point2f> real_moving_points;  // 真实运动的点

    for(size_t i = 0; i < prev_points_.size(); i++) {
        // 跳过跟踪失败的点
        if(!status[i]) continue;

        Point2f& pt_prev = prev_points_[i];
        Point2f& pt_curr = curr_points[i];

        // 检查边界
        if(pt_prev.x < 0 || pt_prev.x >= depth_map.cols ||
           pt_prev.y < 0 || pt_prev.y >= depth_map.rows ||
           pt_curr.x < 0 || pt_curr.x >= depth_map.cols ||
           pt_curr.y < 0 || pt_curr.y >= depth_map.rows) {
            continue;
        }

        // 获取深度
        float depth_prev = prev_depth_.at<float>((int)pt_prev.y, (int)pt_prev.x);
        float depth_curr = depth_map.at<float>((int)pt_curr.y, (int)pt_curr.x);

        // 深度有效性检查
        if(depth_prev < 0.1 || depth_prev > 10.0 ||
           depth_curr < 0.1 || depth_curr > 10.0) {
            continue;
        }

        // 深度一致性验证
        if(!VerifyDepthConsistency(pt_prev, pt_curr, prev_depth_, depth_map)) {
            continue;  // 深度突变，跳过（遮挡或错误）
        }

        // 计算期望光流（相机运动导致的光流）
        Point2f expected_flow = ComputeExpectedFlow(pt_prev, depth_prev, camera_motion);

        // 计算实际光流
        Point2f actual_flow = pt_curr - pt_prev;

        // 计算残差光流（物体真实运动）
        Point2f residual_flow = actual_flow - expected_flow;
        float residual_magnitude = norm(residual_flow);

        // 检查是否在YOLO框内（框内点权重更高）
        bool in_yolo_box = false;
        for(const auto& box : yolo_boxes) {
            if(box.contains(pt_prev)) {
                in_yolo_box = true;
                break;
            }
        }

        // 判断是否为真实动态点
        float threshold = motion_threshold_;
        if(!in_yolo_box) {
            threshold *= 1.5;  // 框外点阈值提高50%
        }

        if(residual_magnitude > threshold) {
            real_moving_points.push_back(pt_prev);
        }
    }

    cout << "[OpticalFlowFilter] 相机运动补偿: 检测到 " << real_moving_points.size()
         << " 个真实运动点" << endl;

    // ========== 步骤4：聚类生成动态区域 ==========
    vector<Rect> optical_flow_regions;
    if(!real_moving_points.empty()) {
        optical_flow_regions = ClusterDynamicRegions(
            real_moving_points,
            vector<bool>(real_moving_points.size(), true),
            gray.size()
        );
    }

    // ========== 步骤5：与YOLO框合并（并集）==========
    vector<Rect> merged_regions = yolo_boxes;
    for(const auto& flow_region : optical_flow_regions) {
        merged_regions.push_back(flow_region);
    }

    // 更新上一帧数据
    prev_gray_ = gray.clone();
    prev_depth_ = depth_map.clone();
    prev_points_ = GenerateGridPoints(gray.size(), grid_size_);

    detected_region_count_ = merged_regions.size();

    cout << "[OpticalFlowFilter] YOLO:" << yolo_boxes.size()
         << " + 光流:" << optical_flow_regions.size()
         << " = 合并:" << merged_regions.size() << endl;

    return merged_regions;
}

// ========== v4.0核心：计算期望光流（相机运动导致的光流）==========
Point2f OpticalFlowFilter::ComputeExpectedFlow(
    const Point2f& pt,
    float depth,
    const Sophus::SE3f& camera_motion
) const {
    // 1. 像素坐标 → 归一化坐标
    Eigen::Vector3f p_normalized(
        (pt.x - cx_) / fx_,
        (pt.y - cy_) / fy_,
        1.0f
    );

    // 2. 归一化坐标 → 3D点（相机坐标系）
    Eigen::Vector3f P_prev = depth * p_normalized;

    // 3. 相机运动变换：P_curr = T_motion * P_prev
    Eigen::Vector3f P_curr = camera_motion * P_prev;

    // 4. 投影回当前帧像素坐标
    if(P_curr.z() < 0.01) {
        // 点在相机后面，返回零光流
        return Point2f(0, 0);
    }

    float x_curr = fx_ * P_curr.x() / P_curr.z() + cx_;
    float y_curr = fy_ * P_curr.y() / P_curr.z() + cy_;

    Point2f pt_curr(x_curr, y_curr);

    // 5. 期望光流 = 当前位置 - 之前位置
    return pt_curr - pt;
}

// ========== v4.0新增：深度一致性验证 ==========
bool OpticalFlowFilter::VerifyDepthConsistency(
    const Point2f& pt_prev,
    const Point2f& pt_curr,
    const Mat& depth_prev,
    const Mat& depth_curr
) const {
    int x_prev = (int)pt_prev.x;
    int y_prev = (int)pt_prev.y;
    int x_curr = (int)pt_curr.x;
    int y_curr = (int)pt_curr.y;

    // 边界检查
    if(x_prev < 0 || x_prev >= depth_prev.cols ||
       y_prev < 0 || y_prev >= depth_prev.rows ||
       x_curr < 0 || x_curr >= depth_curr.cols ||
       y_curr < 0 || y_curr >= depth_curr.rows) {
        return false;
    }

    float d_prev = depth_prev.at<float>(y_prev, x_prev);
    float d_curr = depth_curr.at<float>(y_curr, x_curr);

    // 深度有效性
    if(d_prev < 0.1 || d_curr < 0.1) {
        return false;
    }

    // 深度突变检测（>50cm = 遮挡或错误）
    float depth_diff = fabs(d_curr - d_prev);
    if(depth_diff > depth_consistency_threshold_) {
        return false;
    }

    return true;
}

} // namespace ORB_SLAM3
