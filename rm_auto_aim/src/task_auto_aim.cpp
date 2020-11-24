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
#include "rm_auto_aim/task_auto_aim.hpp"

using namespace cv;
using namespace std;
using namespace rm_auto_aim;

using std::placeholders::_1;

TaskAutoAim::TaskAutoAim(rclcpp::Node::SharedPtr &nh):
                    TaskImageProc(nh) {
    gimbal_ctrl_flag_ = true;
    shoot_ctrl_flag_ = true;
    nh_ = nh;
    //init params
    nh_->declare_parameter("camera_intrinsic");
    nh_->declare_parameter("camera_distortion");
    //create pub,sub,srv
    cmd_gimbal_pub_ =  nh_->create_publisher<rm_interfaces::msg::GimbalCmd>("cmd_gimbal", 10); 
    cmd_shoot_pub_ = nh_->create_publisher<rm_interfaces::msg::ShootCmd>("cmd_shoot", 10); 
    robot_state_sub_ =nh_->create_subscription<rm_interfaces::msg::StdRobotState>(          // CHANGE
          "robot_state", 10, std::bind(&TaskAutoAim::robotStateCallback, this, std::placeholders::_1));
    set_mode_srv_ = nh_->create_service<rm_interfaces::srv::SetMode>("task_auto_aim/set_mode",
                         std::bind(&TaskAutoAim::setModeCallBack, this, std::placeholders::_1,std::placeholders::_2));
    //get param
    auto camera_intrinsic_param =nh_->get_parameter("camera_intrinsic");
    std::vector<double> camera_intrinsic = camera_intrinsic_param.as_double_array();
    auto camera_distortion_param =nh_->get_parameter("camera_distortion");
    std::vector<double> camera_distortion = camera_distortion_param.as_double_array();
    // init tool class
    auto_aim_algo_.init(camera_intrinsic,camera_distortion);
    //set target blue
    auto_aim_algo_.setTargetColor(false);
    projectile_tansform_tool_.setProjectileModel(NULL);
    RCLCPP_INFO(nh_->get_logger(), "init!");
}

TaskAutoAim::~TaskAutoAim() {}

void TaskAutoAim::taskImageProcess(cv::Mat &img, double /*img_stamp*/) {
    int ret;
    ret = auto_aim_algo_.process(img,current_pitch_);
    if (ret == 0) {
        ArmorTarget target = auto_aim_algo_.getTarget();
        Point3f position = target.postion / 100;
        //transform
        float pitch,yaw;
        if(projectile_tansform_tool_.transform(position,pitch,yaw)!=0){
            return;
        }
        //发布云台控制topic
        if (gimbal_ctrl_flag_) {
            rm_interfaces::msg::GimbalCmd gimbal_cmd;
            gimbal_cmd.type = 0x00;
            gimbal_cmd.position.pitch = pitch;
            gimbal_cmd.position.yaw = yaw;
            cmd_gimbal_pub_->publish(gimbal_cmd);
        }
        //发射子弹
        if (shoot_ctrl_flag_) {
            rm_interfaces::msg::ShootCmd shoot_cmd;
            shoot_cmd.projectile_num = 3;
            cmd_shoot_pub_->publish(shoot_cmd);
        }
        RCLCPP_INFO(nh_->get_logger(), "find target!" );
        cout << position << endl;
    }else{
        RCLCPP_INFO(nh_->get_logger(), "not find target!");//表示没有发现目标
    }
}

bool TaskAutoAim::setModeCallBack(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
    // 0x00,休眠模式，0x01:自动射击模式，0x02：自动瞄准模式（不发子弹）,0x03,测试模式,不控制.
    // 0x10,设置目标为红色，0x11,设置目标为蓝色
    response->success=true;
    if (request->mode == 0x00) {
        stopTask();
    } else if (request->mode == 0x01) {
        startTask();
        gimbal_ctrl_flag_ = true;
        shoot_ctrl_flag_ = true;
    } else if (request->mode == 0x02) {
        startTask();
        gimbal_ctrl_flag_ = true;
        shoot_ctrl_flag_ = false;
    } else if (request->mode == 0x03) {
        startTask();
        gimbal_ctrl_flag_ = false;
        shoot_ctrl_flag_ = false;
    } else if (request->mode == 0x10) {             
        //red color config 
        auto_aim_algo_.setTargetColor(true);  
        RCLCPP_INFO(nh_->get_logger(), "set target color red" );
    } else if (request->mode == 0x11) {
        //blue color config
        auto_aim_algo_.setTargetColor(false);    
        RCLCPP_INFO(nh_->get_logger(), "set target color blue" );
    }else{
        response->success = false;
    }
    return true;
}

void TaskAutoAim::robotStateCallback(const rm_interfaces::msg::StdRobotState::SharedPtr msg){
    current_pitch_ = msg->gimbal_pos.pitch;
}
