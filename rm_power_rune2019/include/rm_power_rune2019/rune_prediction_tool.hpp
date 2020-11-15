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
#ifndef RM_POWER_RUNE_2019_RUNE_PREDICTION_TOOL_H
#define RM_POWER_RUNE_2019_RUNE_PREDICTION_TOOL_H

#include <opencv2/opencv.hpp>

namespace rm_power_rune2019 {

class RunePredictionTool {
   public:
    static bool fitCircle(const std::vector<cv::Point2f> &points, cv::Point2f &centre, float &radius);
    static cv::Point2f rotatePoint2D(cv::Point2f center, cv::Point2f postion, float angle);
};

}  // namespace rm_power_rune2019

#endif  //RM_POWER_RUNE_2019_RUNE_PREDICTION_TOOL_H
