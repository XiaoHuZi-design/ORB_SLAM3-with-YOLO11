# 参数传递与数据流架构详解

**作者**: ht & Claude Code
**日期**: 2025-11-17
**目标**: 从光流+YOLO检测 → 动态剔除 → 稠密点云生成的完整数据流

---

## 🎯 整体数据流图

```
┌────────────────────────────────────────────────────────────────────────┐
│                         1. 图像输入                                     │
│  rgbd_tum.cc:104  SLAM.TrackRGBD(imRGB, imD, timestamp)               │
└────────────────────────────┬───────────────────────────────────────────┘
                             │ cv::Mat imRGB, imD
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    2. 动态检测（光流+YOLO）                             │
│  Tracking.cc:1747  dynamicAreas = FilterManager::FilterFrame(...)     │
│                                                                         │
│  ┌─────────────────────────┐    ┌─────────────────────────┐          │
│  │ YoloFilter              │    │ OpticalFlowFilter       │          │
│  │ YoloDetect.cpp:Detect() │    │ calcOpticalFlowPyrLK()  │          │
│  │ → vector<Rect>          │    │ → vector<Rect>          │          │
│  └───────────┬─────────────┘    └───────────┬─────────────┘          │
│              └──────────┬──────────────────┘                          │
│                         │ FilterManager::MergeBBoxes()                │
│                         ▼                                              │
│              std::vector<cv::Rect> dynamicAreas                       │
└────────────────────────────┬───────────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    3. 传递到 ORB 特征提取                               │
│  Tracking.cc:1759  mpORBextractorLeft->mvDynamicArea = dynamicAreas   │
│                                                                         │
│  ORBextractor.h:115  vector<cv::Rect> mvDynamicArea;                  │
└────────────────────────────┬───────────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    4. 传递到 Frame                                      │
│  Tracking.cc:1802  mCurrentFrame.mvDynamicArea = dynamicAreas         │
│                                                                         │
│  Frame.h:约200行  std::vector<cv::Rect> mvDynamicArea;                │
└────────────────────────────┬───────────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    5. 传递到 KeyFrame                                   │
│  Tracking.cc:CreateNewKeyFrame()                                       │
│  → KeyFrame构造函数自动继承 Frame.mvDynamicArea                         │
│                                                                         │
│  KeyFrame.h:约150行  std::vector<cv::Rect> mvDynamicArea;             │
└────────────────────────────┬───────────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    6. 点云生成时动态剔除                                │
│  PointCloudMapper.cpp:62  for (auto area : kf->mvDynamicArea)         │
│                           if (area.contains(pt)) IsDynamic = true;     │
│                                                                         │
│  结果: 动态区域内的像素不生成点云                                       │
└────────────────────────────┬───────────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                    7. 点云过滤与融合                                    │
│  PointCloudMapper.cpp:110  *mpGlobalMap += *pointCloud_new            │
│  PointCloudMapper.cpp:120  mpVoxel->filter(*mpGlobalMap)              │
│  PointCloudMapper.cpp:117  mpStatisticalFilter->filter(*temp) [可选]  │
│                                                                         │
│  输出: vslam_final.pcd (干净的稠密点云地图)                            │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 📋 关键参数传递路径

### 路径1: 图像输入 → 动态检测

| 步骤 | 文件 | 行号 | 参数名 | 类型 | 说明 |
|------|------|------|--------|------|------|
| 1 | `Examples/RGB-D/rgbd_tum.cc` | 104 | `imRGB, imD` | `cv::Mat` | 输入RGB和深度图 |
| 2 | `src/System.cc` | TrackRGBD() | 传递到Tracking | `cv::Mat` | 系统层转发 |
| 3 | `src/Tracking.cc` | 1751 | `imRGB` → `InputImage` | `cv::Mat` | 克隆用于检测 |

**代码片段**:
```cpp
// rgbd_tum.cc:104
SLAM.TrackRGBD(imRGB, imD, tframe);

// Tracking.cc:1751
Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, ...) {
    cv::Mat InputImage = imRGB.clone();  // 行1766
```

---

### 路径2: 动态检测执行

| 步骤 | 文件 | 行号 | 函数/变量 | 输入 | 输出 |
|------|------|------|-----------|------|------|
| **2.1 判断模式** | `src/Tracking.cc` | 1747-1756 | `if (mbUseFilterManager)` | `mbUseFilterManager` | 选择检测方式 |
| **2.2a 联合检测** | `src/Tracking.cc` | 1749 | `FilterManager::FilterFrame()` | `InputImage, timestamp` | `vector<Rect>` |
| **2.2b 纯YOLO** | `src/Tracking.cc` | 1753-1755 | `mpDetector->Detect()` | `InputImage` | `mvDynamicArea` |

**代码片段**:
```cpp
// Tracking.cc:1747-1756
std::vector<cv::Rect> dynamicAreas;

if (mpFilterManager && mbUseFilterManager) {
    // 使用 FilterManager（YOLO + 光流联合检测）
    dynamicAreas = mpFilterManager->FilterFrame(InputImage, timestamp);  // 行1749
    std::cout << "[Tracking] FilterManager detected " << dynamicAreas.size() << " dynamic areas" << std::endl;
} else {
    // 回退到纯YOLO检测（原有逻辑）
    mpDetector->GetImage(InputImage);
    mpDetector->Detect();
    dynamicAreas = mpDetector->mvDynamicArea;  // 行1755
}
```

---

### 路径3: FilterManager 内部流程（联合检测）

| 步骤 | 文件 | 行号 | 函数 | 说明 |
|------|------|------|------|------|
| 3.1 | `src/DynamicFilter/FilterManager.cpp` | 42-82 | `FilterFrame()` | 调度所有过滤器 |
| 3.2 | `src/DynamicFilter/YoloFilter.cpp` | 79-101 | `DetectDynamicAreas()` | 调用YOLO检测 |
| 3.3 | `src/YoloDetect.cpp` | ~50 | `Detect()` | YOLO推理 |
| 3.4 | `src/DynamicFilter/OpticalFlowFilter.cpp` | 101-278 | `DetectDynamicAreas()` | 光流检测 |
| 3.5 | `src/DynamicFilter/FilterManager.cpp` | 87-120 | `MergeBBoxes()` | 融合结果（并集） |

**详细调用链**:
```cpp
// FilterManager.cpp:42-82
std::vector<cv::Rect> FilterManager::FilterFrame(const cv::Mat& image, double timestamp) {
    std::vector<cv::Rect> merged_result;

    for (auto& [name, filter] : filters_) {  // 遍历所有过滤器
        if (!filter_enabled_[name]) continue;

        // 调用每个过滤器的检测
        auto result = filter->DetectDynamicAreas(image, timestamp);  // 行63

        if (merged_result.empty()) {
            merged_result = result;
        } else {
            // 融合策略
            merged_result = MergeBBoxes(merged_result, result, strategy_);  // 行71
        }
    }
    return merged_result;
}
```

---

### 路径4: 光流检测详细流程

| 步骤 | 文件 | 行号 | 操作 | 输入 | 输出 |
|------|------|------|------|------|------|
| 4.1 | `OpticalFlowFilter.cpp` | 120-135 | 网格采样 | `gray` (灰度图) | `grid_points` (采样点) |
| 4.2 | `OpticalFlowFilter.cpp` | 139-146 | LK光流跟踪 | `prev_gray_, gray, grid_points` | `next_points, status` |
| 4.3 | `OpticalFlowFilter.cpp` | 150-162 | 运动分析 | `next_points - grid_points` | `moving_points` |
| 4.4 | `OpticalFlowFilter.cpp` | 165-214 | 区域聚类 | `moving_points` | `clusters` |
| 4.5 | `OpticalFlowFilter.cpp` | 218-272 | 生成边界框 | `clusters` | `vector<Rect> regions` |

**核心代码**:
```cpp
// OpticalFlowFilter.cpp:139-146 (LK光流)
std::vector<cv::Point2f> next_points;
std::vector<uchar> status;
std::vector<float> err;

cv::calcOpticalFlowPyrLK(
    prev_gray_, gray,              // 前后两帧
    grid_points, next_points,      // 输入/输出点
    status, err,                   // 跟踪状态
    lk_win_size_, lk_max_level_    // 参数: 21x21窗口, 3层金字塔
);

// OpticalFlowFilter.cpp:150-162 (运动分析)
for (size_t i = 0; i < grid_points.size(); i++) {
    if (!status[i]) continue;

    cv::Point2f flow = next_points[i] - grid_points[i];
    float magnitude = cv::norm(flow);

    if (magnitude > motion_threshold_) {  // 阈值: 2.5像素
        moving_points.push_back(grid_points[i]);
    }
}
```

---

### 路径5: 检测结果传递到 ORB 和 Frame

| 步骤 | 文件 | 行号 | 变量/函数 | 说明 |
|------|------|------|-----------|------|
| 5.1 | `src/Tracking.cc` | 1759 | `mpORBextractorLeft->mvDynamicArea = dynamicAreas` | 传递给ORB提取器 |
| 5.2 | `include/ORBextractor.h` | 115 | `vector<cv::Rect> mvDynamicArea` | ORB提取器成员变量 |
| 5.3 | `src/Tracking.cc` | 1802 | `mCurrentFrame.mvDynamicArea = dynamicAreas` | 传递给当前帧 |
| 5.4 | `include/Frame.h` | ~200 | `std::vector<cv::Rect> mvDynamicArea` | Frame成员变量 |

**代码片段**:
```cpp
// Tracking.cc:1759 - 传递给ORB提取器
mpORBextractorLeft->mvDynamicArea = dynamicAreas;

// Tracking.cc:1802 - 传递给当前帧
mCurrentFrame.mvDynamicArea = dynamicAreas;
```

---

### 路径6: Frame → KeyFrame 自动继承

| 步骤 | 文件 | 行号 | 函数 | 说明 |
|------|------|------|------|------|
| 6.1 | `src/Tracking.cc` | ~1300 | `CreateNewKeyFrame()` | 创建关键帧 |
| 6.2 | `src/KeyFrame.cc` | 构造函数 | `KeyFrame(Frame &F, ...)` | 从Frame复制所有成员 |
| 6.3 | `include/KeyFrame.h` | ~150 | `std::vector<cv::Rect> mvDynamicArea` | KeyFrame成员变量 |

**代码逻辑**:
```cpp
// KeyFrame构造函数会自动继承Frame的所有成员变量
KeyFrame::KeyFrame(Frame &F, ...) :
    ...,
    mvDynamicArea(F.mvDynamicArea),  // 复制动态区域
    ...
{
}
```

---

### 路径7: KeyFrame → 点云生成（动态剔除）

| 步骤 | 文件 | 行号 | 操作 | 说明 |
|------|------|------|------|------|
| 7.1 | `src/Tracking.cc` | ~1350 | `mpPointCloudMapper->InsertKeyFrame(...)` | 插入关键帧到点云线程 |
| 7.2 | `src/PointCloudMapper.cpp` | 44-50 | `InsertKeyFrame()` | 加入队列 |
| 7.3 | `src/PointCloudMapper.cpp` | 103 | `GeneratePointCloud()` | 生成点云 |
| 7.4 | `src/PointCloudMapper.cpp` | 62-63 | `for (auto area : kf->mvDynamicArea)` | 遍历动态区域 |
| 7.5 | `src/PointCloudMapper.cpp` | 63 | `if (area.contains(pt))` | 判断像素是否在动态区域 |

**核心剔除逻辑**:
```cpp
// PointCloudMapper.cpp:56-78 (GeneratePointCloud函数)
for (int v=0; v<imRGB.rows; v++) {
    for (int u=0; u<imRGB.cols; u++) {
        cv::Point2i pt(u, v);

        // ========== 关键：动态剔除逻辑 ==========
        bool IsDynamic = false;
        for (auto area : kf->mvDynamicArea) {           // 行62: 遍历所有动态区域
            if (area.contains(pt)) IsDynamic = true;    // 行63: 检查像素是否在区域内
        }

        if (!IsDynamic) {                                // 行64: 只处理非动态像素
            float d = imDepth.ptr<float>(v)[u];
            if (d<0.01 || d>10) continue;                // 行67: 深度范围过滤

            // 生成点云点
            PointT p;
            p.z = d;
            p.x = (u - kf->cx) * p.z / kf->fx;          // 行70: 像素→相机坐标
            p.y = (v - kf->cy) * p.z / kf->fy;

            p.b = imRGB.ptr<cv::Vec3b>(v)[u][0];        // 行73-75: RGB颜色
            p.g = imRGB.ptr<cv::Vec3b>(v)[u][1];
            p.r = imRGB.ptr<cv::Vec3b>(v)[u][2];
            pointCloud_temp->push_back(p);
        }
    }
}
```

---

### 路径8: 点云过滤与全局融合

| 步骤 | 文件 | 行号 | 操作 | 参数 | 说明 |
|------|------|------|------|------|------|
| 8.1 | `PointCloudMapper.cpp` | 103 | `pointCloud_new = GeneratePointCloud()` | - | 生成单帧点云 |
| 8.2 | `PointCloudMapper.cpp` | 110 | `*mpGlobalMap += *pointCloud_new` | - | 累加到全局地图 |
| 8.3 | `PointCloudMapper.cpp` | 114 | `pcl::copyPointCloud(*mpGlobalMap, *temp)` | - | 深拷贝 |
| 8.4 | `PointCloudMapper.cpp` | 117-118 | `mpStatisticalFilter->filter(*temp)` | MeanK=50, Thresh=1.0 | 统计滤波（**已注释**） |
| 8.5 | `PointCloudMapper.cpp` | 120-121 | `mpVoxel->filter(*mpGlobalMap)` | LeafSize=0.008 | 体素下采样 |
| 8.6 | `PointCloudMapper.cpp` | 130 | `savePCDFileBinary("vslam_final.pcd", ...)` | - | 保存最终地图 |

**过滤流程**:
```cpp
// PointCloudMapper.cpp:100-122 (run函数主循环)
while (1) {
    if (mKeyFrameSize != 0) {
        // 8.1 生成单帧点云
        PointCloud::Ptr pointCloud_new(new PointCloud);
        pointCloud_new = GeneratePointCloud(mqKeyFrame.front(), mqRGB.front(), mqDepth.front());

        mqKeyFrame.pop();
        mqRGB.pop();
        mqDepth.pop();

        // 8.2 累加到全局地图
        *mpGlobalMap += *pointCloud_new;                    // 行110

        // 8.3 深拷贝用于过滤
        PointCloud::Ptr temp(new PointCloud);
        pcl::copyPointCloud(*mpGlobalMap, *temp);           // 行114

        // 8.4 统计滤波（当前已注释）
        // mpStatisticalFilter->setInputCloud(mpGlobalMap); // 行117
        // mpStatisticalFilter->filter(*temp);              // 行118

        // 8.5 体素滤波（下采样）
        mpVoxel->setInputCloud(temp);                       // 行120
        mpVoxel->filter(*mpGlobalMap);                      // 行121
    }
}
```

---

## 🔧 关键参数汇总

### 动态检测参数

| 参数名 | 文件位置 | 行号 | 默认值 | 说明 |
|--------|---------|------|--------|------|
| **YOLO检测** |
| `dynamic_classes_` | `YoloFilter.cpp` | 24-27 | person, car, ... | 动态物体类别 |
| `bbox_expansion_` | `YoloFilter.cpp` | 19, 33 | 10 | YOLO框扩展（像素）|
| **光流检测** |
| `motion_threshold_` | `OpticalFlowFilter.cpp` | 40 | 2.5 | 运动阈值（像素）|
| `grid_size_` | `OpticalFlowFilter.cpp` | 41 | 20 | 网格采样间隔 |
| `min_points_per_region_` | `OpticalFlowFilter.cpp` | 42 | 5 | 最小聚类点数 |
| `lk_win_size_` | `OpticalFlowFilter.cpp` | 43 | 21x21 | LK窗口大小 |
| `lk_max_level_` | `OpticalFlowFilter.cpp` | 44 | 3 | 金字塔层数 |
| `clustering_distance_` | `OpticalFlowFilter.cpp` | 45 | 50 | 聚类距离阈值 |
| `expand` | `OpticalFlowFilter.cpp` | 262 | 20 | 边界框扩展 |
| **融合策略** |
| `strategy_` | `FilterManager.cpp` | 26 | UNION | 融合策略（并集）|

### 点云过滤参数

| 参数名 | 文件位置 | 行号 | 默认值 | 说明 |
|--------|---------|------|--------|------|
| **深度过滤** |
| 最小深度 | `PointCloudMapper.cpp` | 67 | 0.01 | 深度下限（米）|
| 最大深度 | `PointCloudMapper.cpp` | 67 | 10 | 深度上限（米）|
| **体素滤波** |
| `LeafSize` | `PointCloudMapper.cpp` | 16 | 0.008 | 体素大小（米）|
| **统计滤波（已注释）** |
| `MeanK` | `PointCloudMapper.cpp` | 22 | 50 | 邻域点数 |
| `StddevMulThresh` | `PointCloudMapper.cpp` | 23 | 1.0 | 标准差倍数 |

---

## 📊 数据结构传递

### 核心数据类型

```cpp
// 1. 动态区域边界框
std::vector<cv::Rect> dynamicAreas;
// 传递路径: FilterManager → Tracking → ORBextractor → Frame → KeyFrame → PointCloudMapper

// 2. 点云点
pcl::PointXYZRGB {
    float x, y, z;      // 3D坐标
    uint8_t r, g, b;    // RGB颜色
}

// 3. 点云对象
pcl::PointCloud<PointT>::Ptr pointCloud;
```

### 成员变量位置

| 变量名 | 类 | 文件 | 行号 | 说明 |
|--------|---|------|------|------|
| `mvDynamicArea` | ORBextractor | `include/ORBextractor.h` | 115 | ORB提取器的动态区域 |
| `mvDynamicArea` | Frame | `include/Frame.h` | ~200 | 帧的动态区域 |
| `mvDynamicArea` | KeyFrame | `include/KeyFrame.h` | ~150 | 关键帧的动态区域 |
| `mpGlobalMap` | PointCloudMapper | `include/PointCloudMapper.h` | ~50 | 全局点云地图 |
| `mpVoxel` | PointCloudMapper | `include/PointCloudMapper.h` | ~51 | 体素滤波器 |
| `mpStatisticalFilter` | PointCloudMapper | `include/PointCloudMapper.h` | ~52 | 统计滤波器（已注释）|

---

## 🎯 优化建议

### 当前瓶颈

1. **统计滤波被注释掉** - 导致飘点残留
2. **深度范围过大** (0.01-10m) - 远距离噪点多
3. **无深度梯度检查** - 边缘噪点未过滤

### 优化路径

1. **启用统计滤波** → 见下节修改代码
2. **收紧深度范围** → `0.3-5m`
3. **增加边界框扩展** → `expand = 50`

---

## 🔍 调试技巧

### 1. 查看每步检测结果

```cpp
// Tracking.cc:1749 后添加
std::cout << "[DEBUG] YOLO detected: " << yolo_areas.size() << " areas" << std::endl;
std::cout << "[DEBUG] OpticalFlow detected: " << optical_areas.size() << " areas" << std::endl;
std::cout << "[DEBUG] Merged: " << dynamicAreas.size() << " areas" << std::endl;
```

### 2. 可视化动态区域

```cpp
// Tracking.cc:1810 后添加
cv::Mat debug_img = imRGB.clone();
for (auto& rect : dynamicAreas) {
    cv::rectangle(debug_img, rect, cv::Scalar(0, 255, 0), 2);
}
cv::imshow("Dynamic Areas", debug_img);
cv::waitKey(1);
```

### 3. 统计点云数量

```cpp
// PointCloudMapper.cpp:110 后添加
std::cout << "[DEBUG] Before filter: " << mpGlobalMap->size() << " points" << std::endl;
// 过滤后
std::cout << "[DEBUG] After filter: " << mpGlobalMap->size() << " points" << std::endl;
```

---

**文档结束** 🎉

下一步: 修改 `PointCloudMapper.cpp` 启用统计滤波
