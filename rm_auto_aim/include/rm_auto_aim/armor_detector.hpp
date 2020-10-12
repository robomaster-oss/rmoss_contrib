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

#ifndef RM_AUTO_AIM_ARMOR_DETECTOR_HPP
#define RM_AUTO_AIM_ARMOR_DETECTOR_HPP

#include<opencv2/opencv.hpp>

//装甲板识别检测算法部分，提取相关特征点。

/* 本代码参考量大量robomaster2018各大学开源代码，主要有如下：
 *   大连交大robomaster 
 *    https://github.com/Ponkux/RM2018-DJTU-VisionOpenSource
 *   东南大学robomaster :
 *    https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource
 *   东林robomaster :
 *    https://github.com/moxiaochong/NEFU-ARES-Vison-Robomaster2018
*/

namespace rm_auto_aim{
  //参考东南大学，构建灯条，装甲板描述结构体
  typedef struct _LightDescriptor{
        int id;
	cv::RotatedRect r;
        cv::Point2f endPointYMin,endPointYMax;//灯条两端点
        cv::Point2f centerPoint;//中心点
     	//light info
        float lightHight;//灯条高度
        float lightWidth;//灯条宽度
	float lightRatioWH;// 灯条宽高比,为灯条宽/高
	float lightArea; //灯条区域大小
	float lightAngle; //灯条倾斜角度

  }LightDescriptor;

  typedef struct _ArmorDescriptor{
        cv::Point2f points[4];
        //info
	LightDescriptor lightR,lightL;//装甲板两灯条
        cv::Point2f centerPoint;//中心点
        float armorHight;//灯条高度
        float armorWidth;//灯条宽度
        float armorRatioWH;//装甲板宽高比，宽/高
	float parallelErr;//灯条平行角度误差
        float innerAngleErr;//装甲板内角误差
        float horizonLineAngle; //装甲板水平角度

  }ArmorDescriptor;



  class ArmorDetector{
  public:
     ArmorDetector();
     ~ArmorDetector();
  public:
     void setTargetColor(bool is_red);
     int process(cv::Mat img);
     std::vector<ArmorDescriptor> getArmorVector();
  private:
     int preImg(cv::Mat src,cv::Mat &dst);
     int getLightDescriptor(cv::RotatedRect r,LightDescriptor &light);//灯条参数计算
     void lightHightLong(LightDescriptor &light,float height);//灯条高度补偿
     int lightsMatch(LightDescriptor l1,LightDescriptor l2,ArmorDescriptor &armor);//灯调匹配
     //调试
     void  printArmorDescriptor(ArmorDescriptor armor);
  private:
       bool target_is_red_;
       std::vector<LightDescriptor> mLights;
       std::vector<ArmorDescriptor> mArmors;

  };

}

#endif //RM_AUTO_AIM_ARMOR_DETECTOR_HPP
