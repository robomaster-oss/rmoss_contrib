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
#ifndef RM_POWER_RUNE_2019_RUNE_DETECTOR_H
#define RM_POWER_RUNE_2019_RUNE_DETECTOR_H

#include <string>
#include "opencv2/opencv.hpp"
#include "rm_power_rune2019/rune_types.hpp"

namespace rm_power_rune2019
{

    enum class ArmorColor {red, blue};
    class RuneDetector
    {
    public:
        RuneDetector();
        ~RuneDetector();

    public:
        void setTargetColor(ArmorColor color);
        int process(cv::Mat &img);
        std::vector<ArmorDescriptor> getAllArmors();
        //调试
        void printArmorDescriptor(ArmorDescriptor &armor);

    private:
        // armor
        int preImg(cv::Mat &src, cv::Mat &dst);
        int getArmorDescriptor(cv::RotatedRect r, ArmorDescriptor &armor);
        int armorsDetect(cv::Mat &src_img, cv::Mat &thresh_img);

    private:
        bool target_is_red_;
        std::vector<ArmorDescriptor> armors_;
    };

} // namespace rm_power_rune2019

#endif // RM_POWER_RUNE_2019_RUNE_DETECTOR_H
