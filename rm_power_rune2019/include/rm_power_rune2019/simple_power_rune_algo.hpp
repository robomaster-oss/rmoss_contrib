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
#ifndef TASK_POWER_RUNE_2019_SIMPLE_POWER_RUNE_ALGO_HPP
#define TASK_POWER_RUNE_2019_SIMPLE_POWER_RUNE_ALGO_HPP

#include <string>

#include "opencv2/opencv.hpp"
#include "rm_power_rune2019/rune_detector.hpp"
#include "rm_power_rune2019/rune_prediction_tool.hpp"
#include "rm_power_rune2019/rune_state_machine.hpp"
#include "rm_util/mono_measure_tool.hpp"

namespace rm_power_rune2019 {

class SimplePowerRuneAlgo {
   public:
    SimplePowerRuneAlgo(std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool);
    ~SimplePowerRuneAlgo();

   public:
    //set target
    void setTargetColor(ArmorColor color);
    //algorithm for small power rune
    int process4small(cv::Mat &img);
    void clear4small();
    //algorithm for large power rune
    int process4large(cv::Mat &img);
    void clear4large();
    //shoot
    cv::Point3f getShootTarget();
    void setReShoot();

   private:
    int processNoraml(cv::Mat &img);
    int processStaticDebug(cv::Mat &img);

   private:
    // tool
    RuneDetector rune_detector_;
    RuneStateMachine rune_state_machine_;
    std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool_;
    // const data
    cv::Rect roi_;
    cv::Point2f roi_offset_;
    std::vector<cv::Point3f> realpoints_;
    // rune circle data
    cv::Point2f rune_center_;
    float rune_radius_;
    std::vector<cv::Point2f> stored_points_;
    bool points_are_enough_;
    // task data
    cv::Point3f target_;
    bool is_reshoot_;
};

}  //namespace rm_power_rune2019

#endif  //TASK_POWER_RUNE_2019_SIMPLE_POWER_RUNE_ALGO_HPP
