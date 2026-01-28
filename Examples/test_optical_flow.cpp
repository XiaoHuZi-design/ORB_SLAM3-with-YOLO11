/**
 * @file test_optical_flow.cpp
 * @brief 测试光流过滤器的独立程序
 * @author ht & Claude Code
 * @date 2025-01-16
 */

#include "DynamicFilter/OpticalFlowFilter.h"
#include "DynamicFilter/FilterManager.h"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace ORB_SLAM3;

int main(int argc, char** argv) {
    if(argc != 2) {
        cerr << "Usage: " << argv[0] << " <video_file_or_camera_id>" << endl;
        return -1;
    }

    // 打开视频或相机
    VideoCapture cap;

    if(string(argv[1]).length() == 1 && isdigit(argv[1][0])) {
        // 相机ID
        int cam_id = atoi(argv[1]);
        cap.open(cam_id);
    } else {
        // 视频文件
        cap.open(argv[1]);
    }

    if(!cap.isOpened()) {
        cerr << "Failed to open video source: " << argv[1] << endl;
        return -1;
    }

    // 创建光流过滤器
    auto of_filter = make_shared<OpticalFlowFilter>();
    of_filter->Initialize();

    // 创建管理器
    FilterManager manager;
    manager.RegisterFilter("optical_flow", of_filter);

    cout << "=== Optical Flow Filter Test ===" << endl;
    cout << "Press 'q' to quit, 'space' to pause" << endl;

    Mat frame;
    int frame_count = 0;
    bool paused = false;

    while(true) {
        if(!paused) {
            cap >> frame;

            if(frame.empty()) {
                cout << "End of video" << endl;
                break;
            }

            frame_count++;
            double timestamp = frame_count * 0.033;  // 假设30fps

            // 执行过滤
            auto dynamic_areas = manager.FilterFrame(frame, timestamp);

            // 获取可视化
            Mat vis = of_filter->GetVisualization();

            if(!vis.empty()) {
                // 显示信息
                string info = "Frame: " + to_string(frame_count);
                info += " | Dynamic regions: " + to_string(dynamic_areas.size());
                putText(vis, info, Point(10, vis.rows - 20),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);

                imshow("Optical Flow Filter", vis);
            } else {
                imshow("Optical Flow Filter", frame);
            }
        }

        int key = waitKey(paused ? 0 : 30);

        if(key == 'q' || key == 27) {  // q or ESC
            break;
        } else if(key == ' ') {  // space
            paused = !paused;
            cout << (paused ? "Paused" : "Resumed") << endl;
        }
    }

    // 输出统计
    auto stats = manager.GetStatistics();
    cout << "\n=== Statistics ===" << endl;
    for(const auto& pair : stats) {
        cout << pair.first << ": " << pair.second << " regions detected in last frame" << endl;
    }

    return 0;
}
