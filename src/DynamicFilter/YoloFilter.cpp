/**
 * @file YoloFilter.cpp
 * @brief 基于YOLO的动态物体检测过滤器实现
 * @author ht & Claude Code
 * @date 2025-01-16
 */

#include "DynamicFilter/YoloFilter.h"
#include <iostream>

using namespace std;
using namespace cv;

namespace ORB_SLAM3 {

YoloFilter::YoloFilter()
    : detector_(nullptr),
      external_detector_(false),
      bbox_expansion_(10)
{
    name_ = "YOLO-v11";

    // 默认动态物体类别（COCO数据集）
    dynamic_classes_ = {
        "person", "bicycle", "car", "motorcycle",
        "bus", "truck", "bird", "cat", "dog"
    };
}

YoloFilter::YoloFilter(YoloDetection* detector)
    : detector_(detector),
      external_detector_(true),
      bbox_expansion_(20)  // v4.0优化：从10增加到20像素，提高覆盖范围
{
    name_ = "YOLO-v11";

    // 默认动态物体类别（COCO数据集）
    dynamic_classes_ = {
        "person", "bicycle", "car", "motorcycle",
        "bus", "truck", "bird", "cat", "dog"
    };
}

YoloFilter::~YoloFilter() {
    // 只有非外部创建的检测器才需要删除
    if(!external_detector_ && detector_) {
        delete detector_;
    }
}

bool YoloFilter::Initialize(const string& config_file) {
    cout << "[YoloFilter] Initializing..." << endl;

    // 如果外部已经设置了检测器，直接使用
    if(external_detector_ && detector_) {
        cout << "[YoloFilter] Using external YOLO detector" << endl;
        return true;
    }

    // 否则创建新的检测器
    detector_ = new YoloDetection();

    if(!detector_) {
        cerr << "[YoloFilter] Failed to create YOLO detector" << endl;
        return false;
    }

    cout << "[YoloFilter] YOLO detector created" << endl;
    cout << "[YoloFilter] Dynamic classes: ";
    for(const auto& cls : dynamic_classes_) {
        cout << cls << " ";
    }
    cout << endl;
    cout << "[YoloFilter] BBox expansion: " << bbox_expansion_ << " pixels" << endl;

    return true;
}

vector<Rect> YoloFilter::DetectDynamicAreas(
    const Mat& image,
    double timestamp
) {
    if(!enabled_ || !detector_) {
        return vector<Rect>();
    }

    // ========== 调用YOLO检测 ==========
    // 注意：此处会调用 detector_->Detect()，是实际的YOLO推理
    // 如果外部已经调用了 Detect()，会导致重复检测浪费资源
    // FilterManager会通过此函数调用YOLO，外部Tracking不应再调用
    Mat input_image = image.clone();
    detector_->GetImage(input_image);

    bool success = detector_->Detect();  // ✅ YOLO推理在此执行

    if(!success) {
        cerr << "[YoloFilter] YOLO detection failed" << endl;
        return vector<Rect>();
    }

    // 获取检测结果
    vector<Rect> dynamic_areas;
    class_count_.clear();

    // 从detector的mmDetectMap中提取动态物体区域
    // mmDetectMap: map<string, vector<Rect>>，存储每个类别名对应的所有检测框
    for(const auto& pair : detector_->mmDetectMap) {
        // pair.first: 类别名称，如"person", "car"等（COCO数据集的80类）
        const string& class_name = pair.first;
        // pair.second: 该类别所有检测到的边界框列表
        // 每个Rect包含: x(左上角x坐标), y(左上角y坐标), width(宽度), height(高度)
        const vector<Rect>& boxes = pair.second;

        // 检查当前类别是否属于预定义的动态物体类别
        // dynamic_classes_默认包含: "person", "car", "bus", "truck", "bicycle", "motorcycle"
        if(!IsDynamicClass(class_name)) {
            continue;  // 如果不是动态类别，跳过
        }

        // 统计该动态类别检测到的数量
        // class_count_: map<string, int>，用于记录每个动态类别的检测数量
        class_count_[class_name] = boxes.size();

        // 遍历该类别的所有检测框
        for(const auto& box : boxes) {
            // box: cv::Rect类型，表示YOLO检测到的原始边界框
            // box.x: 边界框左上角的x坐标（像素）
            // box.y: 边界框左上角的y坐标（像素）
            // box.width: 边界框的宽度（像素）
            // box.height: 边界框的高度（像素）

            // 方法1：使用YoloDetect.cpp中的比例系数方式扩展
            // 直接使用原始边界框，扩展由YoloDetect.cpp:177-178的系数控制
            // 优点：比例自适应，对不同大小的物体扩展比例一致
            // dynamic_areas.push_back(box);

            // 方法2：使用像素方式扩展
            // ExpandBBox参数说明：
            //   - box: 原始边界框
            //   - image.size(): 图像尺寸Size(width, height)，用于边界检查
            //   - bbox_expansion_: 向四周扩展的像素数，默认20像素
            // 返回值：扩展后的边界框，确保不超出图像边界
            // 优点：固定像素扩展，对小目标扩展比例大，大目标扩展比例小
            Rect expanded_box = ExpandBBox(box, image.size(), bbox_expansion_);
            dynamic_areas.push_back(expanded_box);
        }
    }

    detected_region_count_ = dynamic_areas.size();

    // 生成可视化（可选）
    visualization_ = image.clone();
    if(!dynamic_areas.empty()) {
        for(const auto& area : dynamic_areas) {
            rectangle(visualization_, area, Scalar(0, 255, 0), 2);
        }

        // 添加统计信息
        int y_offset = 30;
        for(const auto& pair : class_count_) {
            string text = pair.first + ": " + to_string(pair.second);
            putText(visualization_, text, Point(10, y_offset),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
            y_offset += 25;
        }
    }

    return dynamic_areas;
}

bool YoloFilter::IsDynamicClass(const string& class_name) const {
    return find(dynamic_classes_.begin(), dynamic_classes_.end(), class_name)
           != dynamic_classes_.end();
}

Rect YoloFilter::ExpandBBox(const Rect& bbox, const Size& image_size, int expand) const {
    Rect expanded = bbox;

    expanded.x = max(0, expanded.x - expand);
    expanded.y = max(0, expanded.y - expand);
    expanded.width = min(image_size.width - expanded.x, expanded.width + 2 * expand);
    expanded.height = min(image_size.height - expanded.y, expanded.height + 2 * expand);

    return expanded;
}

Mat YoloFilter::GetVisualization() const {
    return visualization_;
}

} // namespace ORB_SLAM3
