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
#include "rm_power_rune2019/power_rune_server.hpp"
#include "rm_projectile_motion/gravity_projectile_solver.hpp"

using namespace cv;
using namespace std;
using namespace rm_power_rune2019;

using std::placeholders::_1;

//显式调用基类带参数构造函数
PowerRuneServer::PowerRuneServer(rclcpp::Node::SharedPtr& node){
    node_ = node;
    // init and get params
    node_->declare_parameter("camera_intrinsic");
    node_->declare_parameter("camera_distortion");
    auto camera_intrinsic_param = node_->get_parameter("camera_intrinsic");
    std::vector<double> camera_intrinsic = camera_intrinsic_param.as_double_array();
    auto camera_distortion_param = node_->get_parameter("camera_distortion");
    std::vector<double> camera_distortion = camera_distortion_param.as_double_array();
    // create pub,sub,srv
    gimbal_ctrl_pub_ = node_->create_publisher<rmoss_interfaces::msg::GimbalCmd>("robot_base/gimbal_cmd", 10);
    shoot_pub_ = node_->create_publisher<rmoss_interfaces::msg::ShootCmd>("robot_base/shoot_cmd", 10);
    gimbal_state_sub_ = node_->create_subscription<rmoss_interfaces::msg::Gimbal>("robot_base/gimbal_state", 10,
        std::bind(&PowerRuneServer::gimbalStateCallback, this, std::placeholders::_1));
    set_mode_srv_ = node_->create_service<rmoss_interfaces::srv::SetMode>("task_power_rune/set_mode",
        std::bind(&PowerRuneServer::setModeCallBack, this, std::placeholders::_1, std::placeholders::_2));
    // init algo tool
    auto mono_location_tool = std::make_shared<rm_util::MonoMeasureTool>();
    mono_location_tool->set_camera_info(camera_intrinsic, camera_distortion);
    power_rune_algo_ = std::make_shared<SimplePowerRuneAlgo>(mono_location_tool);
    power_rune_algo_->setTargetColor(ArmorColor::red);
    // init projectile tool
    auto projectile_model = std::make_shared<rm_projectile_motion::GravityProjectileSolver>(25);
    projectile_tansform_tool_ = std::make_shared<rm_projectile_motion::GimbalTransformTool>();
    projectile_tansform_tool_->set_projectile_solver(projectile_model);
    //start task
    std::string topic_name = node_->declare_parameter("cam_topic_name", "camera/image_raw");
    using namespace std::placeholders;
    image_task_server_ = std::make_shared<rm_util::ImageTaskServer>(
        node_,topic_name,std::bind(&PowerRuneServer::process_image, this,_1,_2));
    image_task_server_->start();
    RCLCPP_INFO(node_->get_logger(), "init successfully!");
}

void PowerRuneServer::process_image(cv::Mat& img, double /*img_stamp*/)
{
    // RCLCPP_INFO(node_->get_logger(), "stamp:%f",img_stamp);
    if (is_need_reshoot_) {
        power_rune_algo_->setReShoot();
        is_need_reshoot_ = false;
    }
    if (current_mode_ == TaskMode::small) {
        if (is_need_clear_) {
            power_rune_algo_->clear4small();
            is_need_clear_ = false;
        }
        int ret = power_rune_algo_->process4small(img);
        if (ret == 0) {
            //射击
            Point3f position = power_rune_algo_->getShootTarget();
            std::cout << "small shoot:" << position << std::endl;
        }
    } else if (current_mode_ == TaskMode::large) {
        if (is_need_clear_) {
            power_rune_algo_->clear4large();
            is_need_clear_ = false;
        }
        int ret = power_rune_algo_->process4large(img);
        if (ret == 0) {
            //射击
            Point3f position = power_rune_algo_->getShootTarget();
            std::cout << "large shoot:" << position << std::endl;
        }
    }
}

bool PowerRuneServer::setModeCallBack(
    const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response)
{
    // 0x00 停止，0x01,运行小能量机关，0x02运行大能量机关,0x03重置发射

    response->success = true;
    if (request->mode == 0x00) {
        image_task_server_->stop();
        current_mode_ = TaskMode::idle;
    } else if (request->mode == 0x01) {
        image_task_server_->start();
        is_need_clear_ = true;
        current_mode_ = TaskMode::small;
    } else if (request->mode == 0x02) {
        image_task_server_->start();
        is_need_clear_ = true;
        current_mode_ = TaskMode::large;
    } else if (request->mode == 0x03) {
        is_need_reshoot_ = true;
    } else if (request->mode == 0x10) {
        //red color config
        if (current_mode_ == TaskMode::idle) {
            power_rune_algo_->setTargetColor(ArmorColor::red);
        } else {
            response->success = false;
        }
    } else if (request->mode == 0x11) {
        //blue color config,
        if (current_mode_ == TaskMode::idle) {
            power_rune_algo_->setTargetColor(ArmorColor::blue);
        } else {
            response->success = false;
        }
    } else {
        response->success = false;
    }
    return true;
}

void PowerRuneServer::gimbalStateCallback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg)
{
    current_pitch_ = msg->pitch;
}
