/**
 * @file FilterManager.cpp
 * @brief 动态过滤器管理器实现
 * @author ht & Claude Code
 * @date 2025-01-16
 */

#include "DynamicFilter/FilterManager.h"
#include "DynamicFilter/OpticalFlowFilter.h"
#include <iostream>
#include <algorithm>

using namespace std;
using namespace cv;

namespace ORB_SLAM3 {

FilterManager::FilterManager()
    : strategy_(UNION)   //当前使用的是并集策略 UNION、 INTERSECTION、 CASCADE
{
    cout << "[FilterManager] Initialized with UNION strategy" << endl;
}

FilterManager::~FilterManager() {
    filters_.clear();
}

void FilterManager::RegisterFilter(
    const string& name,
    shared_ptr<DynamicFilterBase> filter
) {
    if(!filter) {
        cerr << "[FilterManager] Cannot register null filter: " << name << endl;
        return;
    }

    filters_[name] = filter;
    cout << "[FilterManager] Registered filter: " << name << endl;
}

void FilterManager::RemoveFilter(const string& name) {
    auto it = filters_.find(name);
    if(it != filters_.end()) {
        filters_.erase(it);
        cout << "[FilterManager] Removed filter: " << name << endl;
    }
}

vector<Rect> FilterManager::FilterFrame(
    const Mat& image,
    double timestamp
) {
    // ========== 清空上一帧的结果 ==========
    individual_results_.clear();

    if(filters_.empty()) {
        return vector<Rect>();
    }

    // 收集所有启用的过滤器的结果
    vector<vector<Rect>> all_areas;
    vector<string> active_filter_names;

    // ========== 遍历所有注册的过滤器（YOLO、光流等）==========
    // 每个过滤器的 DetectDynamicAreas() 会执行实际检测
    // 例如：YoloFilter::DetectDynamicAreas() → detector_->Detect()（YOLO推理）
    //       OpticalFlowFilter::DetectDynamicAreas() → 计算光流
    for(auto& pair : filters_) {
        const string& name = pair.first;
        auto& filter = pair.second;

        if(!filter->IsEnabled()) {
            continue;
        }

        vector<Rect> areas = filter->DetectDynamicAreas(image, timestamp);

        // ========== 新增：存储每个过滤器的结果（用于可视化对比）==========
        individual_results_[name] = areas;

        if(!areas.empty()) {
            all_areas.push_back(areas);
            active_filter_names.push_back(name);
        }
    }

    if(all_areas.empty()) {
        return vector<Rect>();
    }

    // 根据策略合并结果
    vector<Rect> result;

    switch(strategy_) {
        case UNION:
            result = MergeUnion(all_areas); //所有过滤器检测结果的并集
            break;

        case INTERSECTION:
            result = MergeIntersection(all_areas); //所有过滤器检测结果的交集
            break;

        case CASCADE:
            // 级联：使用第一个过滤器的结果
            result = all_areas[0];
            break;

        default:
            result = MergeUnion(all_areas);  //所有过滤器检测结果的并集
    }

    return result;
}

// ========== v4.0新增：带相机运动补偿的过滤接口 ==========
vector<Rect> FilterManager::FilterFrameWithMotionCompensation(
    const Mat& image,
    double timestamp,
    const Mat& depth_map,
    const Sophus::SE3f& camera_motion,
    float fx, float fy, float cx, float cy
) {
    // 清空上一帧的结果
    individual_results_.clear();

    if(filters_.empty()) {
        return vector<Rect>();
    }

    // 收集所有启用的过滤器的结果
    vector<vector<Rect>> all_areas;
    vector<string> active_filter_names;

    for(auto& pair : filters_) {
        const string& name = pair.first;
        auto& filter = pair.second;

        if(!filter->IsEnabled()) {
            continue;
        }

        vector<Rect> areas;

        // 检查是否为OpticalFlowFilter（支持相机运动补偿）
        auto optical_flow_filter = dynamic_pointer_cast<OpticalFlowFilter>(filter);

        if(optical_flow_filter) {
            // 使用相机运动补偿版本
            // 首先获取YOLO框（如果存在）
            vector<Rect> yolo_boxes;
            auto yolo_it = individual_results_.find("yolo");
            if(yolo_it != individual_results_.end()) {
                yolo_boxes = yolo_it->second;
            }

            areas = optical_flow_filter->DetectDynamicAreasWithMotionCompensation(
                image, timestamp, yolo_boxes, depth_map, camera_motion, fx, fy, cx, cy
            );

            cout << "[FilterManager] OpticalFlow with motion compensation: "
                 << areas.size() << " regions" << endl;
        } else {
            // 普通检测方法（如YOLO）
            areas = filter->DetectDynamicAreas(image, timestamp);
        }

        // 存储每个过滤器的结果
        individual_results_[name] = areas;

        if(!areas.empty()) {
            all_areas.push_back(areas);
            active_filter_names.push_back(name);
        }
    }

    if(all_areas.empty()) {
        return vector<Rect>();
    }

    // 根据策略合并结果
    vector<Rect> result;

    switch(strategy_) {
        case UNION:
            result = MergeUnion(all_areas);
            break;

        case INTERSECTION:
            result = MergeIntersection(all_areas);
            break;

        case CASCADE:
            // 级联：使用第一个过滤器的结果
            result = all_areas[0];
            break;

        default:
            result = MergeUnion(all_areas);
    }

    cout << "[FilterManager] Motion compensation mode: Merged " << result.size()
         << " regions from " << active_filter_names.size() << " filters" << endl;

    return result;
}

void FilterManager::EnableFilter(const string& name, bool enabled) {
    auto it = filters_.find(name);
    if(it != filters_.end()) {
        it->second->SetEnabled(enabled);
        cout << "[FilterManager] Filter '" << name << "' "
             << (enabled ? "enabled" : "disabled") << endl;
    } else {
        cerr << "[FilterManager] Filter not found: " << name << endl;
    }
}

shared_ptr<DynamicFilterBase> FilterManager::GetFilter(const string& name) {
    auto it = filters_.find(name);
    if(it != filters_.end()) {
        return it->second;
    }
    return nullptr;
}

map<string, int> FilterManager::GetStatistics() const {
    map<string, int> stats;

    for(const auto& pair : filters_) {
        stats[pair.first] = pair.second->GetDetectedRegionCount();
    }

    return stats;
}

map<string, Mat> FilterManager::GetVisualizations() const {
    map<string, Mat> visualizations;

    for(const auto& pair : filters_) {
        Mat vis = pair.second->GetVisualization();
        if(!vis.empty()) {
            visualizations[pair.first] = vis;
        }
    }

    return visualizations;
}

// ========== 新增：获取每个过滤器的检测结果（用于可视化对比）==========
map<string, vector<Rect>> FilterManager::GetIndividualResults() const {
    return individual_results_;
}

vector<Rect> FilterManager::MergeUnion(
    const vector<vector<Rect>>& all_areas
) {
    // 简单实现：直接合并所有矩形
    vector<Rect> merged;

    for(const auto& areas : all_areas) {
        merged.insert(merged.end(), areas.begin(), areas.end());
    }

    // TODO: 可以合并重叠的矩形以减少数量

    return merged;
}

vector<Rect> FilterManager::MergeIntersection(
    const vector<vector<Rect>>& all_areas
) {
    if(all_areas.empty()) {
        return vector<Rect>();
    }

    if(all_areas.size() == 1) {
        return all_areas[0];
    }

    // 交集：找到所有过滤器都检测到的区域
    // 简化实现：如果矩形在所有过滤器中都有重叠区域，保留该重叠
    vector<Rect> result;

    // 以第一个过滤器的结果为基准
    for(const auto& rect1 : all_areas[0]) {
        bool found_in_all = true;

        // 检查是否在其他所有过滤器中都有重叠
        for(size_t i = 1; i < all_areas.size(); i++) {
            bool has_overlap = false;

            for(const auto& rect2 : all_areas[i]) {
                // 检查两个矩形是否有交集
                Rect intersection = rect1 & rect2;
                if(intersection.area() > 0) {
                    has_overlap = true;
                    break;
                }
            }

            if(!has_overlap) {
                found_in_all = false;
                break;
            }
        }

        if(found_in_all) {
            result.push_back(rect1);
        }
    }

    return result;
}

} // namespace ORB_SLAM3
