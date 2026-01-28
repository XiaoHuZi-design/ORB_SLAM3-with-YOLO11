# 稠密点云地图质量优化指南

**目标**: 消除"飘在空中"的噪点，生成适合栅格地图的高质量点云

---

## 🎯 问题分析

你遇到的"飘点"问题有3个主要原因：

### 1. 动态剔除不完全
- **现象**: 检测框没完全覆盖动态物体边缘
- **后果**: 手臂、头发等边缘点云残留

### 2. 深度噪声
- **现象**: RGB-D相机深度测量抖动
- **后果**: 同一位置的点在时间上前后漂移

### 3. 相机运动伪动态
- **现象**: 相机快速移动时，静态物体也产生光流
- **后果**: 误删静态点或误保留动态点

---

## 🔧 优化方案

### 方案1: 增大动态检测框扩展（最简单）

**修改位置**: `src/DynamicFilter/OpticalFlowFilter.cpp:262`

```cpp
// 原代码
int expand = 20;  // 扩展20像素

// 优化后
int expand = 50;  // 扩展50像素（更激进）
```

**效果**: 更彻底地剔除动态物体边缘

**副作用**: 可能过度删除（把静态背景也删了）

**适用场景**: 导航用栅格地图（宁可少点，不要错点）

---

### 方案2: 点云统计滤波（推荐）

**原理**: 删除"孤立点"（周围没有邻居的点）

**修改位置**: `src/PointCloudMapper.cpp` (在体素滤波后添加)

#### 步骤1: 添加头文件

```cpp
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
```

#### 步骤2: 在 `insertKeyFrame` 或 `generatePointCloud` 函数中添加

找到类似这样的代码：
```cpp
// 体素滤波
pcl::VoxelGrid<PointT> voxel;
voxel.setLeafSize(0.01, 0.01, 0.01);
voxel.setInputCloud(cloud);
voxel.filter(*cloud);
```

**在体素滤波后添加**：

```cpp
// ========== 新增：统计滤波 ==========
pcl::StatisticalOutlierRemoval<PointT> sor;
sor.setInputCloud(cloud);
sor.setMeanK(50);              // 每个点考察50个邻居
sor.setStddevMulThresh(1.0);   // 标准差倍数（1.0=严格，2.0=宽松）
sor.filter(*cloud);

std::cout << "[PointCloudMapper] Filtered cloud: "
          << cloud->points.size() << " points" << std::endl;
```

**参数调优**：
- `MeanK`: 50-100 (越大越慢，但更准确)
- `StddevMulThresh`:
  - `0.5`: 非常严格（删除更多点）
  - `1.0`: 推荐值
  - `2.0`: 宽松（保留更多点）

---

### 方案3: 半径滤波（针对飘点）

**原理**: 删除半径内邻居太少的点

**在统计滤波后继续添加**：

```cpp
// ========== 新增：半径滤波 ==========
pcl::RadiusOutlierRemoval<PointT> ror;
ror.setInputCloud(cloud);
ror.setRadiusSearch(0.05);      // 半径5cm
ror.setMinNeighborsInRadius(5); // 半径内至少5个点
ror.filter(*cloud);

std::cout << "[PointCloudMapper] After radius filter: "
          << cloud->points.size() << " points" << std::endl;
```

**参数调优**：
- `RadiusSearch`:
  - `0.02`: 2cm（严格，适合精细场景）
  - `0.05`: 5cm（推荐）
  - `0.10`: 10cm（宽松）
- `MinNeighborsInRadius`: 5-10

---

### 方案4: 深度范围限制（基础过滤）

**原理**: 只保留合理深度范围的点

**修改位置**: 生成点云的循环（`PointCloudMapper.cpp`）

找到类似这样的代码：
```cpp
for (int m = 0; m < depth.rows; m += 3) {
    for (int n = 0; n < depth.cols; n += 3) {
        float d = depth.ptr<float>(m)[n];

        // 原代码可能只检查 d > 0
        if (d > 0 && d < 10.0) {
            // 生成点云
        }
    }
}
```

**优化为**：

```cpp
// ========== 深度过滤参数 ==========
const float MIN_DEPTH = 0.3;   // 最小深度30cm（避免相机前噪点）
const float MAX_DEPTH = 5.0;   // 最大深度5m（避免远距离噪点）

for (int m = 0; m < depth.rows; m += 3) {
    for (int n = 0; n < depth.cols; n += 3) {
        float d = depth.ptr<float>(m)[n];

        // 深度范围检查
        if (d < MIN_DEPTH || d > MAX_DEPTH) continue;

        // 跳过动态区域（原有逻辑）
        if (isInDynamicArea(m, n)) continue;

        // 生成点云
        PointT p;
        p.z = d;
        p.x = (n - cx) * p.z / fx;
        p.y = (m - cy) * p.z / fy;

        // ========== 新增：深度梯度检查 ==========
        // 如果相邻像素深度差异太大，可能是边缘噪点
        if (n > 0) {
            float d_left = depth.ptr<float>(m)[n-1];
            if (abs(d - d_left) > 0.3) continue;  // 深度跳变>30cm，跳过
        }

        tmp->points.push_back(p);
    }
}
```

---

### 方案5: 平面分割（高级，去除飘点）

**原理**: 只保留属于平面的点（墙、地面）

```cpp
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// ========== 平面分割 ==========
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

pcl::SACSegmentation<PointT> seg;
seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.02);  // 平面距离阈值2cm

seg.setInputCloud(cloud);
seg.segment(*inliers, *coefficients);

// 提取平面点
pcl::ExtractIndices<PointT> extract;
extract.setInputCloud(cloud);
extract.setIndices(inliers);
extract.setNegative(false);  // false=保留平面点
extract.filter(*cloud);
```

**适用场景**: 室内导航（只关心地面和墙壁）

---

## 📊 推荐组合方案

### 组合A: 快速优化（5分钟实现）
```
1. 增大检测框扩展 (expand = 50)
2. 深度范围限制 (0.3m - 5m)
```
**效果**: 消除70%的飘点
**性能**: 无明显影响

---

### 组合B: 标准优化（推荐）
```
1. 深度范围限制 (0.3m - 5m)
2. 深度梯度检查 (跳变>30cm)
3. 体素滤波 (原有)
4. 统计滤波 (MeanK=50, Thresh=1.0)
```
**效果**: 消除90%的飘点
**性能**: 增加约20%计算时间

---

### 组合C: 导航专用（最严格）
```
1. 深度范围限制 (0.5m - 4m)
2. 检测框扩展 (expand = 50)
3. 体素滤波 (原有)
4. 统计滤波 (MeanK=50, Thresh=0.5) ← 更严格
5. 半径滤波 (Radius=0.05, MinNeighbors=8)
6. 平面分割 (只保留地面、墙壁)
```
**效果**: 接近完美的干净点云
**性能**: 增加约50%计算时间

---

## 🔍 调试技巧

### 1. 可视化对比

```bash
# 生成两个点云
DynamicFilter.UseFilterManager: 0
运行 -> 保存 map_before.pcd

# 修改代码，添加滤波
运行 -> 保存 map_after.pcd

# 对比
pcl_viewer map_before.pcd map_after.pcd
```

### 2. 统计信息输出

在每次滤波后添加：
```cpp
std::cout << "[Filter] Before: " << cloud_before->size()
          << " After: " << cloud->size()
          << " Removed: " << (cloud_before->size() - cloud->size())
          << " (" << (100.0 * (cloud_before->size() - cloud->size()) / cloud_before->size())
          << "%)" << std::endl;
```

### 3. 分帧检查

```cpp
// 保存每一帧的点云，检查哪一帧产生飘点
char filename[100];
sprintf(filename, "frame_%04d.pcd", frame_id);
pcl::io::savePCDFileBinary(filename, *cloud);
```

---

## 🎯 栅格地图转换建议

生成干净点云后，转栅格地图的步骤：

### 1. 投影到2D

```cpp
// 遍历点云，投影到XY平面
std::vector<std::vector<float>> grid(grid_size_x, std::vector<float>(grid_size_y, 0));

for (auto& p : cloud->points) {
    int x = (p.x - origin_x) / resolution;
    int y = (p.y - origin_y) / resolution;

    if (x >= 0 && x < grid_size_x && y >= 0 && y < grid_size_y) {
        // 记录该格子的最小高度（地面）
        if (grid[x][y] == 0 || p.z < grid[x][y]) {
            grid[x][y] = p.z;
        }
    }
}
```

### 2. 障碍物检测

```cpp
const float GROUND_HEIGHT = 0.1;   // 地面阈值10cm
const float OBSTACLE_HEIGHT = 0.3; // 障碍物阈值30cm

for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
        if (grid[x][y] == 0) {
            occupancy_grid[x][y] = UNKNOWN;
        } else if (grid[x][y] < GROUND_HEIGHT) {
            occupancy_grid[x][y] = FREE;
        } else if (grid[x][y] > OBSTACLE_HEIGHT) {
            occupancy_grid[x][y] = OCCUPIED;
        }
    }
}
```

### 3. 形态学优化

```cpp
// 膨胀操作：扩大障碍物（安全边界）
cv::Mat grid_img = ...; // 转为OpenCV Mat
cv::dilate(grid_img, grid_img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

// 开运算：去除小噪点
cv::morphologyEx(grid_img, grid_img, cv::MORPH_OPEN,
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
```

---

## 📝 总结

| 问题 | 解决方案 | 优先级 |
|------|---------|--------|
| 动态边缘残留 | 增大检测框扩展 | ⭐⭐⭐ 高 |
| 深度噪声飘点 | 统计滤波 + 半径滤波 | ⭐⭐⭐ 高 |
| 远距离噪点 | 深度范围限制 | ⭐⭐⭐ 高 |
| 边缘噪点 | 深度梯度检查 | ⭐⭐ 中 |
| 非平面飘点 | 平面分割 | ⭐ 低（可选）|

**建议从"组合B"开始**，如果还有飘点再升级到"组合C"。

---

**优化完成后的效果预期**：
- ✅ 点云数量减少20-40%（删除噪点）
- ✅ 飘点基本消除
- ✅ 栅格地图质量显著提升
- ✅ 路径规划成功率提高

祝优化顺利！🎉
