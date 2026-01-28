/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("YOLO-ORB-SLAM3-improved: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStop("menu.Stop",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);

    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    cv::namedWindow("YOLO-ORB-SLAM3-improved: Current Frame");

    // ========== 新增：独立对比窗口（只保留YOLO和光流，主窗口已显示融合结果）==========
    // cv::namedWindow("1. YOLO Detection");
    // cv::namedWindow("2. Optical Flow Detection");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    float trackedImageScale = mpTracker->GetImageScale();

    cout << "Starting the Viewer" << endl;
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);

        if(mbStopTrack)
        {
            menuStepByStep = true;
            mbStopTrack = false;
        }

        if(menuFollowCamera && bFollow)
        {
            if(bCameraView)
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if(bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
        }

        if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
            s_cam.Follow(Ow);
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(menuStepByStep && !bStepByStep)
        {
            //cout << "Viewer: step by step" << endl;
            mpTracker->SetStepByStep(true);
            bStepByStep = true;
        }
        else if(!menuStepByStep && bStepByStep)
        {
            mpTracker->SetStepByStep(false);
            bStepByStep = false;
        }

        if(menuStep)
        {
            mpTracker->mbStep = true;
            menuStep = false;
        }


        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph, menuShowOptLba);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat toShow;
        cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }

        if(mImageViewerScale != 1.f)
        {
            int width = toShow.cols * mImageViewerScale;
            int height = toShow.rows * mImageViewerScale;
            cv::resize(toShow, toShow, cv::Size(width, height));
        }

        // ========== 主窗口：根据模式显示不同内容 ==========
        {
            std::unique_lock<std::mutex> lock(mMutexPAFinsh);

            bool useFilterManager = (!mvYoloAreas.empty() || !mvOpticalAreas.empty() || !mvMergedAreas.empty());

            if (useFilterManager) {
                // FilterManager模式：显示融合结果（黄色框）
                if (!mvMergedAreas.empty()) {
                    for (const auto& area : mvMergedAreas) {
                        cv::Rect adjustedArea(area.x / trackedImageScale, area.y / trackedImageScale,
                                            area.width / trackedImageScale, area.height / trackedImageScale);
                        cv::rectangle(toShow, adjustedArea, cv::Scalar(0, 255, 255), 2); // 黄色框
                    }
                }
                // 显示统计信息
                std::string stats = "YOLO:" + std::to_string(mvYoloAreas.size()) +
                                   " OF:" + std::to_string(mvOpticalAreas.size()) +
                                   " Merged:" + std::to_string(mvMergedAreas.size());
                cv::putText(toShow, stats, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            } else {
                // 纯YOLO模式：显示YOLO检测框（蓝色框+类别）
                for (auto vit = mmDetectMap.begin(); vit != mmDetectMap.end(); vit++) {
                    if (vit->second.size() != 0) {
                        for (auto area : vit->second) {
                            cv::Rect adjustedArea(area.x / trackedImageScale, area.y / trackedImageScale,
                                                area.width / trackedImageScale, area.height / trackedImageScale);
                            cv::rectangle(toShow, adjustedArea, cv::Scalar(255, 0, 0), 2); // 蓝色框
                            cv::putText(toShow, vit->first,
                                       cv::Point(adjustedArea.x, adjustedArea.y - 5),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
                        }
                    }
                }
            }
        }

        cv::imshow("YOLO-ORB-SLAM3-improved: Current Frame",toShow);

        // ========== 保存截图（每100帧）==========
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 100 == 0) {
            std::string save_path = "/home/tianbot/ht/PaperCode/ORB_SLAM3-with-YOLO11/Results/";
            // cv::imwrite(save_path + "latest_main.jpg", toShow); //覆盖只保存最新帧
            cv::imwrite(save_path + "main_" + std::to_string(frame_count) + ".jpg", toShow); //保存所有符合帧
            std::cout << "[Viewer] Saved frame " << frame_count << " screenshots" << std::endl;
        }

        // ========== 独立可视化窗口：仅在FilterManager模式下显示 ==========
        {
            std::unique_lock<std::mutex> lock(mMutexPAFinsh);

            // 检查是否为FilterManager模式
            bool useFilterManager = (!mvYoloAreas.empty() || !mvOpticalAreas.empty() || !mvMergedAreas.empty());

            if (useFilterManager) {
                //新增加窗口
                cv::namedWindow("1. YOLO Detection");
                cv::namedWindow("2. Optical Flow Detection");
                // 使用原始图像作为基础
                cv::Mat im_base = mpFrameDrawer->DrawFrame(trackedImageScale);
                if(mImageViewerScale != 1.f) {
                    int width = im_base.cols * mImageViewerScale;
                    int height = im_base.rows * mImageViewerScale;
                    cv::resize(im_base, im_base, cv::Size(width, height));
                }

                cv::Mat yoloVis = im_base.clone();
                cv::Mat opticalVis = im_base.clone();

                // 1. YOLO检测结果（绿色框 + 类别标签）
                int yolo_count = 0;
                for (const auto& area : mvYoloAreas) {
                    cv::Rect adjustedArea(area.x / trackedImageScale, area.y / trackedImageScale,
                                        area.width / trackedImageScale, area.height / trackedImageScale);
                    cv::rectangle(yoloVis, adjustedArea, cv::Scalar(0, 255, 0), 2);
                    yolo_count++;
                }
                // 添加类别标签
                for (auto vit = mmDetectMap.begin(); vit != mmDetectMap.end(); vit++) {
                    if (vit->second.size() != 0) {
                        for (auto area : vit->second) {
                            cv::Rect adjustedArea(area.x / trackedImageScale, area.y / trackedImageScale,
                                                area.width / trackedImageScale, area.height / trackedImageScale);
                            cv::putText(yoloVis, vit->first,
                                       cv::Point(adjustedArea.x, adjustedArea.y - 5),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                        }
                    }
                }
                cv::putText(yoloVis, "YOLO: " + std::to_string(yolo_count) + " boxes", cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

                // 2. 光流检测结果（红色框）
                int optical_count = 0;
                for (const auto& area : mvOpticalAreas) {
                    cv::Rect adjustedArea(area.x / trackedImageScale, area.y / trackedImageScale,
                                        area.width / trackedImageScale, area.height / trackedImageScale);
                    cv::rectangle(opticalVis, adjustedArea, cv::Scalar(0, 0, 255), 2);
                    optical_count++;
                }
                cv::putText(opticalVis, "Optical Flow: " + std::to_string(optical_count) + " boxes", cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

                // 显示2个独立窗口
                cv::imshow("1. YOLO Detection", yoloVis);
                cv::imshow("2. Optical Flow Detection", opticalVis);

                // ========== 保存截图（每100帧）==========
                // static int frame_count = 0;
                // frame_count++;

                if (frame_count % 100 == 0) {
                    std::string save_path = "/home/tianbot/ht/PaperCode/ORB_SLAM3-with-YOLO11/Results/";
                    // cv::imwrite(save_path + "latest_yolo.jpg", yoloVis);
                    cv::imwrite(save_path + "yolo_" + std::to_string(frame_count) + ".jpg", yoloVis); 
                    // cv::imwrite(save_path + "latest_optical.jpg", opticalVis);
                    cv::imwrite(save_path + "optical_" + std::to_string(frame_count) + ".jpg", opticalVis); 
                    std::cout << "[Viewer] Saved frame " << frame_count << " screenshots" << std::endl;
                }
            }
            // 纯YOLO模式：不显示窗口1和窗口2（已存在的窗口会保持上一帧内容）
        }

        cv::waitKey(mT);


        // cv::Mat im = mpFrameDrawer->DrawFrame();
        // cv::imshow("ORB-SLAM3: Current Frame",im);
        // cv::waitKey(mT);

        // cv::Mat imd = mpFrameDrawer->DrawDepth();
        // if(!imd.empty()) {  // 添加检查
        //     cv::imshow("ORB-SLAM3: Current Depth",imd);
        //     cv::waitKey(mT);
        // }

        // cv::Mat LKcolor = mpFrameDrawer->LK;  // FrameDrawer.h
        // if(!LKcolor.empty())
        // {
        //     cv::imshow("LK OpticalFlow",LKcolor);
        //     cv::waitKey(mT);
        // }

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->ResetActiveMap();
            menuReset = false;
        }

        if(menuStop)
        {
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();

            // Stop all threads
            mpSystem->Shutdown();

            // Save camera trajectory
            mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
            mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
            menuStop = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
