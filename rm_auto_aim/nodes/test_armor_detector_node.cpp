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
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "rm_auto_aim/armor_detector.hpp"
#include "rm_util/debug.hpp"

using namespace rm_auto_aim;
using namespace cv;

int main(int argc, char *argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_armor_detector");
    node->declare_parameter("image_path");
    rclcpp::Parameter str_param = node->get_parameter("image_path");
    std::string image_path= str_param.as_string();
    // open image
    Mat img = imread(image_path);
    // create ArmorDetector
    ArmorDetector armor_detector;
    armor_detector.setTargetColor(true);
    armor_detector.process(img);
    auto results = armor_detector.getArmorVector();
    rm_util::setDebug(true);;
    if (results.size() != 0)
    {
        for(auto result : results){
            RM_DEBUG(rm_util::draw4Point4f(img, result.points));
        }
        imshow("result", img);
        //wait to exit
        waitKey(0);
    }
    else
    {
        std::cout << "not find" << std::endl;
    }
    //clean up
    rclcpp::shutdown();
    return 0;
}