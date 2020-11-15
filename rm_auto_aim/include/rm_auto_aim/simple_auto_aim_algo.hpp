/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#ifndef RM_AUTO_AIM_SIMPLE_AUTO_AIM_ALGO_HPP
#define RM_AUTO_AIM_SIMPLE_AUTO_AIM_ALGO_HPP
#include <string>
#include <opencv2/opencv.hpp>
#include "rm_auto_aim/armor_detector.hpp"
#include "rm_common/mono_measure_tool.hpp"

namespace rm_auto_aim{
   //自瞄算法封装。 
    typedef struct _ArmorTarget{
        ArmorDescriptor armorDescriptor;
        cv::Point3f postion;
        bool isBigArmor=false;
        float distance=0;//单位cm
        float move2dCast=0; //距离中心点的代价
        float track2dCast=0; //与上次目标的2D距离代价
        float track3dCast=0; //与上次目标的3D距离代价
    } ArmorTarget;

    class SimpleAutoAimAlgo{
    public:
	SimpleAutoAimAlgo();
	~SimpleAutoAimAlgo();
    public:
       int init(std::vector<double> camera_intrinsic,std::vector<double> camera_distortion);
       void setTargetColor(bool is_red);
       int process(cv::Mat img,float current_pitch);
       ArmorTarget getTarget();
       void setTrack(bool is_track);
    private:
       //工具类
       ArmorDetector armor_detector_; 
       rm_common::MonoMeasureTool mono_location_tool_;
       //固定参数
       std::vector<cv::Point3f> mSmallArmorPoints; //小装甲板三维点
       std::vector<cv::Point3f> mBigArmorPoints; //大装甲板三维点
       //最终目标
       ArmorTarget mTarget;
       //其他参数
       bool mIsTrack;
       cv::Point2f mLastPoint2;
       cv::Point3f mLastPoint3;
   };
}

#endif//RM_AUTO_AIM_SIMPLE_AUTO_AIM_ALGO_HPP
