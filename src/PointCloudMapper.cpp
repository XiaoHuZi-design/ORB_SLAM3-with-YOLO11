//
// Created by yuwenlu on 2022/7/2.
//
#include "PointCloudMapper.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloudMapper::PointCloudMapper() //double resolution = 0.04  double meank = 50   double thresh = 1
{
    // 使用 make_shared 创建全局地图
    mpGlobalMap = pcl::make_shared<PointCloud>();
    cout << "Global map initialized" << endl;
    // 使用 make_shared 创建体素滤波器
    mpVoxel = pcl::make_shared<pcl::VoxelGrid<PointT>>();
    mpVoxel->setLeafSize(0.01, 0.01, 0.01); //0.01
    cout << "Voxel filter initialized" << endl;

    // ========== 初始化动态检测标志位 ==========
    mHasDetectedDynamic = false;  // 初始状态：未检测到动态物体（假设为静态场景）

    // ========== 暂时禁用统计滤波器（避免段错误）==========
    // // 使用 make_shared 创建统计滤波器
    // mpStatisticalFilter = pcl::make_shared<pcl::StatisticalOutlierRemoval<PointT>>();
    // // 设置统计滤波器参数
    // mpStatisticalFilter->setMeanK(50);           // 设置邻近点数量  典型值50-100
    // mpStatisticalFilter->setStddevMulThresh(1.0); // 设置标准差倍数阈值  典型值1.0-2.0
    // cout << "Statistical filter initialized" << endl;

//----------------------------------------------------------------------------------------------//
    // cout << "PointCloudMapper constructor start" << endl;
    
    // // 使用PCL的智能指针创建方式
    // mpGlobalMap = PointCloud::Ptr(new PointCloud());
    
    // mpVoxel = pcl::VoxelGrid<PointT>::Ptr(new pcl::VoxelGrid<PointT>());
    // mpVoxel->setLeafSize(0.04, 0.04, 0.04);
    
    // mpStatisticalFilter = pcl::StatisticalOutlierRemoval<PointT>::Ptr(
    //     new pcl::StatisticalOutlierRemoval<PointT>());
    // mpStatisticalFilter->setMeanK(50);
    // mpStatisticalFilter->setStddevMulThresh(1.0);
    
    // cout << "All filters initialized successfully" << endl;
    
}

void PointCloudMapper::InsertKeyFrame(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
{
    std::lock_guard<std::mutex> lck_loadKF(mmLoadKFMutex);
    mqKeyFrame.push(kf);
    mqRGB.push(imRGB.clone());
    mqDepth.push(imDepth.clone());
}

PointCloud::Ptr PointCloudMapper::GeneratePointCloud(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
{
    // ========== 修复动态残影问题 v3：智能场景识别 ==========
    // 原问题：默认区域 (1,1,1,1) 太小导致漏检时人体残影
    // v1问题：跳过整帧导致静态背景丢失
    // v2问题：无法区分纯静态场景和动态场景漏检
    // v3方案：使用 mHasDetectedDynamic 标志位区分场景
    //
    // 逻辑：
    // - 纯静态场景（从未检测到人）：正常生成点云，不做动态剔除
    // - 动态场景（曾经检测到过人）：
    //   - 检测到人：正常剔除动态区域
    //   - 漏检：返回空点云（避免残影）

    bool hasValidDynamicArea = !kf->mvDynamicArea.empty();
    bool isDefaultArea = false;

    // 检查是否是默认的无意义区域（回退兼容）
    if (hasValidDynamicArea && kf->mvDynamicArea.size() == 1) {
        const cv::Rect& area = kf->mvDynamicArea[0];
        if (area.width <= 2 && area.height <= 2) {
            isDefaultArea = true;
        }
    }

    // 判断是否有有效的动态区域
    bool hasDynamic = hasValidDynamicArea && !isDefaultArea;

    if (hasDynamic) {
        // 检测到动态物体 → 标记为动态场景
        if (!mHasDetectedDynamic) {
            mHasDetectedDynamic = true;
            cout << "[PointCloudMapper] Dynamic scene detected! Switching to dynamic mapping mode." << endl;
        }
    }

    // 场景判断和处理逻辑
    if (!hasDynamic) {
        // 没有检测到动态区域
        if (mHasDetectedDynamic) {
            // 曾经检测到过人 → 现在空了是漏检 → 返回空点云（保守策略）
            cout << "[PointCloudMapper] Dynamic scene: no detection this frame (likely miss), using conservative strategy" << endl;
            return pcl::make_shared<PointCloud>();
        } else {
            // 从未检测到人 → 纯静态场景 → 正常生成点云，不做动态剔除
            // cout << "[PointCloudMapper] Static scene detected, generating point cloud without dynamic filtering" << endl;
        }
    }
    // 如果 hasDynamic 为 true，继续执行后续的动态剔除逻辑

    PointCloud::Ptr pointCloud_temp(new PointCloud);

    // ========== 优化修改：添加深度范围常量 ==========
    const float MIN_DEPTH = 0.3f;   // 最小深度30cm（避免相机前噪点）
    const float MAX_DEPTH = 5.0f;   // 最大深度5m（避免远距离噪点）

    for (int v=0; v<imRGB.rows; v++)
    {
        for (int u=0; u<imRGB.cols; u++)
        {
            cv::Point2i pt(u, v);

            // ========== 动态剔除逻辑（保持原样）==========
            bool IsDynamic = false;
            for (auto area : kf->mvDynamicArea)
                if (area.contains(pt)) IsDynamic = true;

            if (!IsDynamic)
            {
                float d = imDepth.ptr<float>(v)[u];

                // ========== 优化修改：收紧深度范围（原代码：0.01-10m）==========
                // if (d<0.01 || d>10) continue;  // 原代码
                if (d < MIN_DEPTH || d > MAX_DEPTH) continue;  // 优化后：0.3-5m

                PointT p;
                p.z = d;
                p.x = ( u - kf->cx) * p.z / kf->fx;
                p.y = ( v - kf->cy) * p.z / kf->fy;

                p.b = imRGB.ptr<cv::Vec3b>(v)[u][0];
                p.g = imRGB.ptr<cv::Vec3b>(v)[u][1];
                p.r = imRGB.ptr<cv::Vec3b>(v)[u][2];
                pointCloud_temp->push_back(p);
            }
        }
    }

    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr pointCloud(new PointCloud);
    pcl::transformPointCloud(*pointCloud_temp, *pointCloud, T.inverse().matrix());
    pointCloud->is_dense = false;
    return pointCloud;
}


void PointCloudMapper::run()
{
    pcl::visualization::CloudViewer Viewer("Viewer");
    cout << endl << "PointCloudMapping thread start!" << endl;
    int ID = 0;
    while (1)
    {
        {
            std::lock_guard<std::mutex> lck_loadKFSize(mmLoadKFMutex);
            mKeyFrameSize= mqKeyFrame.size();
        }
        if (mKeyFrameSize != 0)
        {
            PointCloud::Ptr pointCloud_new(new PointCloud);
            pointCloud_new = GeneratePointCloud(mqKeyFrame.front(), mqRGB.front(), mqDepth.front());
            mqKeyFrame.pop();
            mqRGB.pop();
            mqDepth.pop();
            cout << "==============Insert No. " << ID << "KeyFrame ================" << endl;
            ID++;

            // ========== 修复动态残影问题 v2：检查点云是否有效 ==========
            // 如果点云为空（YOLO漏检时的保守策略），只输出日志但继续处理
            if (pointCloud_new->empty()) {
                cout << "[PointCloudMapper] Frame has no valid dynamic area, using conservative strategy (empty point cloud)" << endl;
                // 不 continue，而是继续处理（虽然该帧为空，但不会打断地图构建流程）
            }

            // ========== 简化版：只保留原有逻辑 + 深度优化 ==========
            // 添加到全局地图
            *mpGlobalMap += *pointCloud_new;

            // ========== 临时：保存滤波前的点云 ==========
            pcl::io::savePCDFileBinary("/home/tianbot/ht/PaperCode/ORB_SLAM3-with-YOLO11/before_voxel_filter.pcd", *mpGlobalMap);
            cout << "[PointCloudMapper] Before filter: " << mpGlobalMap->size() << " points saved" << endl;

            // 创建临时点云用于体素滤波
            PointCloud::Ptr temp(new PointCloud);
            pcl::copyPointCloud(*mpGlobalMap, *temp);

            // 只进行体素滤波（与原代码一致）
            mpVoxel->setInputCloud(temp);
            mpVoxel->filter(*mpGlobalMap);

            cout << "[PointCloudMapper] Current global map size: " << mpGlobalMap->size() << " points" << endl;
        }

        // Viewer.showCloud(mpGlobalMap);
        // 显示点云时设置点大小
        if (!mpGlobalMap->empty()) {
            Viewer.showCloud(mpGlobalMap);
            // 设置点云显示属性（需要pcl::visualization::PCLVisualizer）
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            pcl::io::savePCDFileBinary("/home/tianbot/ht/PaperCode/ORB_SLAM3-with-YOLO11/vslam_final.pcd", *mpGlobalMap);
        }
    }
    //pcl::io::savePCDFileBinary("/home/teng/Dy_Nav/ORB_SLAM3-with-YOLO11/vslam_final.pcd", *mpGlobalMap);
    // save();

}



// 保存地图的函数，需要的自行调用~
void PointCloudMapper::save()
{
    std::unique_lock<std::mutex> lck(mmLoadKFMutex);
    pcl::io::savePCDFile("/home/tianbot/ht/PaperCode/ORB_SLAM3-with-YOLO11/result.pcd", *mpGlobalMap);
    cout << "globalMap save finished" << endl;
}














// PointCloud::Ptr PointCloudMapper::GeneratePointCloud(KeyFrame* kf, cv::Mat& imRGB, cv::Mat& imDepth) {
//     // === 输入验证 ===
//     CV_Assert(!imRGB.empty() && imRGB.type() == CV_8UC3);
//     CV_Assert(!imDepth.empty() && imDepth.type() == CV_32F);
//     CV_Assert(imRGB.size() == imDepth.size());
    
//     // === 安全预处理 ===
//     const cv::Mat rgb = imRGB.isContinuous() ? imRGB : imRGB.clone();
//     const cv::Mat depth = imDepth.isContinuous() ? imDepth : imDepth.clone();
    
//     PointCloud::Ptr pointCloud_temp(new PointCloud);
//     pointCloud_temp->reserve(imRGB.rows * imRGB.cols / 2);  // 预分配内存

//     // === 核心处理 ===
//     #pragma omp parallel for collapse(2)  // 并行优化
//     for (int v = 0; v < rgb.rows; v++) {
//         for (int u = 0; u < rgb.cols; u++) {
//             // 动态物体检测安全版
//             bool isDynamic = false;
//             const cv::Point2i pt(u, v);
//             for (const auto& area : kf->mvDynamicArea) {
//                 if (area.x >= 0 && area.y >= 0 && 
//                     area.x + area.width <= rgb.cols &&
//                     area.y + area.height <= rgb.rows) {
//                     isDynamic |= area.contains(pt);
//                 }
//             }
            
//             if (!isDynamic) {
//                 const float d = depth.at<float>(v, u);
//                 if (d > 0.01f && d < 10.0f && 
//                     kf->fx != 0 && kf->fy != 0) {  // 参数校验
//                     PointT p;
//                     p.z = d;
//                     p.x = (u - kf->cx) * p.z / kf->fx;
//                     p.y = (v - kf->cy) * p.z / kf->fy;
                    
//                     const auto& color = rgb.at<cv::Vec3b>(v, u);
//                     p.b = color[0]; p.g = color[1]; p.r = color[2];
                    
//                     #pragma omp critical
//                     pointCloud_temp->push_back(p);
//                 }
//             }
//         }
//     }

//     // === 位姿变换 ===
//     if (!pointCloud_temp->empty()) {
//         Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
//         PointCloud::Ptr pointCloud(new PointCloud);
//         pcl::transformPointCloud(*pointCloud_temp, *pointCloud, T.inverse().matrix());
//         pointCloud->is_dense = false;
//         return pointCloud;
//     }
//     return nullptr;  // 返回空指针而非空点云
// }