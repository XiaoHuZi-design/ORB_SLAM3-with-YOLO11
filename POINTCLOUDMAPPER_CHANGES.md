# PointCloudMapper 优化修改总结

**修改日期**: 2025-11-17
**修改文件**: `src/PointCloudMapper.cpp`
**目标**: 启用统计滤波，消除飘点，优化点云质量

---

## 📝 修改清单

### 修改1: 启用统计滤波器（构造函数）

**位置**: `PointCloudMapper.cpp` 行19-33

**原代码**（已注释掉）:
```cpp
// // 使用 make_shared 创建统计滤波器
// mpStatisticalFilter = pcl::make_shared<pcl::StatisticalOutlierRemoval<PointT>>();
// // 设置统计滤波器参数
// mpStatisticalFilter->setMeanK(50);
// mpStatisticalFilter->setStddevMulThresh(1.0);
// cout << "Statistical filter initialized" << endl;
```

**新代码**（已启用）:
```cpp
// ========== 优化修改：启用统计滤波器（原代码已注释，现在启用）==========
// 使用 make_shared 创建统计滤波器
mpStatisticalFilter = pcl::make_shared<pcl::StatisticalOutlierRemoval<PointT>>();
// 设置统计滤波器参数（优化后）
mpStatisticalFilter->setMeanK(50);           // 设置邻近点数量  典型值50-100
mpStatisticalFilter->setStddevMulThresh(1.0); // 设置标准差倍数阈值  典型值1.0-2.0
cout << "Statistical filter initialized" << endl;

// // ========== 原代码（已注释）==========
// [原代码保留在注释中]
```

**效果**: 初始化统计滤波器，用于删除孤立点（飘点）

---

### 修改2: 收紧深度范围（GeneratePointCloud函数）

**位置**: `PointCloudMapper.cpp` 行65-86

**原代码**:
```cpp
float d = imDepth.ptr<float>(v)[u];
if (d<0.01 || d>10) continue;  // 0.01-10米
```

**新代码**:
```cpp
// ========== 优化修改：添加深度范围常量 ==========
const float MIN_DEPTH = 0.3f;   // 最小深度30cm（避免相机前噪点）
const float MAX_DEPTH = 5.0f;   // 最大深度5m（避免远距离噪点）

float d = imDepth.ptr<float>(v)[u];

// ========== 优化修改：收紧深度范围（原代码：0.01-10m）==========
// if (d<0.01 || d>10) continue;  // 原代码
if (d < MIN_DEPTH || d > MAX_DEPTH) continue;  // 优化后：0.3-5m
```

**效果**:
- 过滤相机前30cm的噪点（反光、手指等）
- 过滤5米外的远距离噪点（精度低）
- 保留室内导航有用的距离范围

---

### 修改3: 启用统计滤波流程（run函数）

**位置**: `PointCloudMapper.cpp` 行130-165

**原代码**:
```cpp
// 添加到全局地图
*mpGlobalMap += *pointCloud_new;
PointCloud::Ptr temp(new PointCloud);
pcl::copyPointCloud(*mpGlobalMap, *temp);

// 先进行统计滤波去除噪声
// mpStatisticalFilter->setInputCloud(mpGlobalMap); //设置输入为全局地图
// mpStatisticalFilter->filter(*temp);  //输出到临时点云（保留滤波结果）

// 再进行体素滤波
mpVoxel->setInputCloud(temp);
mpVoxel->filter(*mpGlobalMap);
```

**新代码**:
```cpp
// ========== 点云过滤流程 ==========
// 1. 添加到全局地图
*mpGlobalMap += *pointCloud_new;
size_t points_before_filter = mpGlobalMap->size();

// 2. 创建临时点云用于过滤
PointCloud::Ptr temp(new PointCloud);
pcl::copyPointCloud(*mpGlobalMap, *temp);

// ========== 优化修改：启用统计滤波（原代码已注释，现在启用）==========
// 3. 先进行统计滤波去除噪声（删除孤立点）
mpStatisticalFilter->setInputCloud(temp);  // 输入：全局地图副本
mpStatisticalFilter->filter(*temp);        // 输出：过滤后的点云
size_t points_after_statistical = temp->size();

// 4. 再进行体素滤波（下采样）
mpVoxel->setInputCloud(temp);
mpVoxel->filter(*mpGlobalMap);
size_t points_after_voxel = mpGlobalMap->size();

// ========== 优化修改：添加过滤统计信息 ==========
cout << "[PointCloudMapper] Points: Before=" << points_before_filter
     << " | After StatisticalFilter=" << points_after_statistical
     << " (removed " << (points_before_filter - points_after_statistical) << ", "
     << (100.0 * (points_before_filter - points_after_statistical) / points_before_filter) << "%)"
     << " | After VoxelFilter=" << points_after_voxel
     << " (removed " << (points_after_statistical - points_after_voxel) << ", "
     << (100.0 * (points_after_statistical - points_after_voxel) / points_after_statistical) << "%)" << endl;

// // ========== 原代码（已注释）==========
// [原代码保留在注释中]
```

**效果**:
- 启用两阶段过滤：统计滤波（删除飘点）→ 体素滤波（下采样）
- 实时显示过滤效果统计（删除了多少点，百分比）
- 方便调试和评估

---

## 🎯 优化效果预期

### 深度范围收紧（0.3-5m）
- ✅ 消除相机前反光噪点
- ✅ 消除远距离低精度点
- ⚠️ 对5米外的物体不建图（室内导航够用）

### 统计滤波启用
- ✅ 删除孤立飘点（周围没有邻居的点）
- ✅ 保留真实表面点（有足够多邻居）
- 📊 预期删除10-30%的点（飘点为主）

### 总体效果
- ✅ 点云密度降低（但质量提升）
- ✅ 栅格地图更干净
- ✅ 路径规划成功率提高

---

## 🔧 参数调优指南

### 如果飘点还很多
```cpp
// 构造函数中调整统计滤波参数
mpStatisticalFilter->setMeanK(50);           // 保持不变
mpStatisticalFilter->setStddevMulThresh(0.5); // 从1.0改为0.5（更严格）
```

### 如果删除过度（有用的点也被删了）
```cpp
// 构造函数中调整统计滤波参数
mpStatisticalFilter->setMeanK(50);           // 保持不变
mpStatisticalFilter->setStddevMulThresh(2.0); // 从1.0改为2.0（更宽松）
```

### 如果需要更远的建图范围
```cpp
// GeneratePointCloud函数中调整深度范围
const float MIN_DEPTH = 0.3f;   // 保持不变
const float MAX_DEPTH = 8.0f;   // 从5.0改为8.0（更远）
```

### 如果近距离也有用（比如手臂操作）
```cpp
// GeneratePointCloud函数中调整深度范围
const float MIN_DEPTH = 0.1f;   // 从0.3改为0.1（更近）
const float MAX_DEPTH = 5.0f;   // 保持不变
```

---

## 📊 运行后查看统计

运行SLAM后，终端会实时输出：
```
[PointCloudMapper] Points: Before=1250000 | After StatisticalFilter=1050000 (removed 200000, 16%) | After VoxelFilter=850000 (removed 200000, 19%)
```

**解读**:
- Before: 累积的原始点数
- After StatisticalFilter: 统计滤波后（删除了16%的飘点）
- After VoxelFilter: 体素滤波后（又删除了19%的冗余点）

---

## ⚠️ 注意事项

### 编译
```bash
cd build
make PointCloudMapper -j4  # 只编译点云映射模块
# 或
make rgbd_tum -j4          # 编译整个程序
```

### 性能影响
- 统计滤波会增加约20-30%的计算时间
- 大地图（>100万点）可能较慢
- 可以通过降低 MeanK 来加速（如改为30）

### 兼容性
- ✅ 保留了所有原代码（在注释中）
- ✅ 可以随时回退（注释掉新代码，取消注释原代码）
- ✅ 不影响其他模块

---

## 🔄 回退方法

如果优化后效果不好，可以轻松回退：

1. **禁用统计滤波**:
   - 注释掉行19-25（构造函数）
   - 注释掉行140-157（run函数）
   - 取消注释原代码

2. **恢复深度范围**:
   - 删除行65-67
   - 改回 `if (d<0.01 || d>10) continue;`

---

## 📖 相关文档

- 完整架构: `PARAMETER_FLOW_ARCHITECTURE.md`
- 优化指南: `POINT_CLOUD_OPTIMIZATION_GUIDE.md`
- 集成日志: `INTEGRATION_LOG.md`

---

**修改完成** ✅
现在编译运行，查看点云质量提升！🎉
