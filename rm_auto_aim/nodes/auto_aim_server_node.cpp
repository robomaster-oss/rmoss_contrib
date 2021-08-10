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
#include "rm_auto_aim/auto_aim_server.hpp"
#include "rm_util/debug.hpp"


using namespace rm_auto_aim;

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("auto_aim_server");
    //set debug
    auto get_debug = node->declare_parameter("get_debug", false);
    rm_util::set_debug(get_debug);
    // create task
    auto task = std::make_shared<AutoAimServer>(node);
    // run node until it's exited
    rclcpp::spin(node);
    //clean up
    rclcpp::shutdown();
    return 0;
}