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
#include "rm_auto_aim/task_auto_aim_node.h"

using namespace cv;
using namespace std;
using namespace rm_auto_aim;

using std::placeholders::_1;

TaskAutoAimNode::TaskAutoAimNode(rclcpp::Node::SharedPtr &nh):
                    TaskImageProcNode(nh) {
    gimbal_ctrl_flag_ = true;
    shoot_ctrl_flag_ = true;
    nh_ = nh;
    //
    /*************create pub,sub,srv************************/
    gimbal_ctrl_pub_ =  nh_->create_publisher<rm_interfaces::msg::GimbalControl>("gimbal_control", 10); 
    shoot_pub_ = nh_->create_publisher<rm_interfaces::msg::ShootControl>("shoot_control", 10); 
    state_info_sub_ =nh_->create_subscription<rm_interfaces::msg::StdBotState>(          // CHANGE
          "state_info", 10, std::bind(&TaskAutoAimNode::robotStateCallback, this, std::placeholders::_1));
    set_mode_srv_ = nh_->create_service<rm_interfaces::srv::SetMode>("task_auto_aim_set_mode",
                         std::bind(&TaskAutoAimNode::setModeCallBack, this, std::placeholders::_1,std::placeholders::_2));
    // init tool class
    int ret;
    ret = auto_aim_algo_.init("");
    projectile_tansform_tool_.setModel(NULL);
    setRunFlag(true);
}

TaskAutoAimNode::~TaskAutoAimNode() {}

void TaskAutoAimNode::taskImageProcess(cv::Mat &img, double img_stamp) {
    int ret;
    //robot_msgs::TaskAutoAimInfo auto_aim_info;
    //auto_aim_info.task_mode = task_mode_;
    auto_aim_algo_.setData(pitch_info_);
    ret = auto_aim_algo_.process(img);
    if (ret == 0) {
        // ROS_INFO("find target!!!!!!");
        ArmorTarget target = auto_aim_algo_.getTarget();
        Point3f position = target.postion / 100;
        // //transformation
        // float pitch, yaw;
        // Point3f position_tran, poistion_in_world;
        // position_tran=position+offset_trans_;
        // float cosa, sina;
        // sina = sin(pitch_info_ );
        // cosa = cos(pitch_info_ );
        // poistion_in_world.z = position_tran.z * cosa - position_tran.y * sina;
        // poistion_in_world.y = position_tran.z * sina + position_tran.y * cosa;
        // poistion_in_world.x=position_tran.x;
        // projectile_tansform_tool_.transform(poistion_in_world, pitch, yaw);
        // pitch=pitch-pitch_info_;
        // pitch = pitch + offset_pitch_ * CV_PI / 180;
        // yaw = yaw + offset_yaw_ * CV_PI / 180;
        // double cast = sqrt(yaw * yaw + pitch * pitch)*180/CV_PI;
        // //发布云台控制topic
        // if (gimbal_ctrl_flag_) {
        //     robot_msgs::GimbalInfo gimbal_info;
        //     gimbal_info.type = 0x00;
        //     gimbal_info.pitch = pitch*pitch_coeff_;
        //     gimbal_info.yaw = yaw*yaw_coeff_;
        //     gimbal_ctrl_pub_.publish(gimbal_info);
        // }
        // //发射子弹
        // if (cast < 1.2 && target.postion.z < 500) {
        //     if (launch_ctrl_flag_) {
        //         robot_msgs::LaunchInfo launch_info;
        //         launch_info.projectile_num = 3;
        //         launch_pub_.publish(launch_info);
                
        //     }
        // }
        //info
        //auto_aim_info.result_code = 0x02;
        //auto_aim_info.x = target.postion.x;
        //auto_aim_info.y = target.postion.y;
        //auto_aim_info.z = target.postion.z;
        //auto_aim_info.pitch = pitch;
        //auto_aim_info.yaw = yaw;
 
        //auto_aim_info.cast = sqrt(yaw * yaw + pitch * pitch)*180/CV_PI;
        RCLCPP_INFO(nh_->get_logger(), "taskImageProcess():find.");
    } else {
       //auto_aim_info.result_code = 0x00;
        RCLCPP_INFO(nh_->get_logger(), "not find target!!!!!!");//表示没有发现目标
        // ROS_INFO("");
    }
    //autoaim_info_pub_.publish(auto_aim_info);
}

bool TaskAutoAimNode::setModeCallBack(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
    // 0x00,启动，正常控制，0x01,启动,不发子弹,0x02,启动，不控制,
    // 0x03 暂停/休眠。
    response->success = false;
    if (1) {
        response->success = true;
        if (request->mode == 0x00) {
            setRunFlag(true);
            gimbal_ctrl_flag_ = true;
            shoot_ctrl_flag_ = true;
        } else if (request->mode == 0x01) {
            setRunFlag(true);
            gimbal_ctrl_flag_ = true;
            shoot_ctrl_flag_ = false;
        } else if (request->mode == 0x02) {
            setRunFlag(true);
            gimbal_ctrl_flag_ = false;
            shoot_ctrl_flag_ = false;
        } else if (request->mode == 0x03) {
            setRunFlag(false);
        } else if (request->mode == 0x10) {             
            // color config
            //auto_aim_algo_.setTargetColor(true);   // target is red
            //ROS_INFO("[task_auto_aim]change target color <red>");
        }else{
            response->success = false;
        }
        //update mode
        if (response->success) {
            task_mode_ = request->mode;
        }
    } else {
        response->success = false;
    }
    return true;
}

void TaskAutoAimNode::robotStateCallback(const rm_interfaces::msg::StdBotState::SharedPtr msg){
    pitch_info_ = msg->current_pitch;
}
