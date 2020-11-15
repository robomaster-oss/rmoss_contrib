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
#ifndef RM_POWER_RUNE_2019_RUNE_DEF_H
#define RM_POWER_RUNE_2019_RUNE_DEF_H

#include "opencv2/opencv.hpp"

  typedef struct _ArmorDescriptor{
      int id;
      cv::RotatedRect r;
      cv::Point2f points[4];
        //info
      cv::Point2f center;//中心点
      float armor_hight;//灯条高度
      float armor_width;//灯条宽度
      float armor_area;
      float armor_ratio_wh;//装甲板宽高比，宽/高
  }ArmorDescriptor;


#endif //RM_POWER_RUNE_2019_RUNE_DEF_H
