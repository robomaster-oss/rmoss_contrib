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
#ifndef RM_AUTO_AIM_TASK_AUTO_AIM_NODE_H
#define RM_AUTO_AIM_TASK_AUTO_AIM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "rm_task/task_image_proc_node.h"
#include "rm_auto_aim/simple_auto_aim_algo.h"
#include "rm_projectile_motion/projectile_transform_tool.h"

#include "rm_msgs/msg/gimbal_control.hpp" 
#include "rm_msgs/msg/shoot_control.hpp" 
#include "rm_msgs/msg/std_bot_state.hpp" 
#include "rm_msgs/srv/set_mode.hpp" 

namespace rm_auto_aim {
class TaskAutoAimNode : public rm_task::TaskImageProcNode {
    public:
        TaskAutoAimNode(rclcpp::Node::SharedPtr &nh);
        ~TaskAutoAimNode();
    private:
        bool setModeCallBack(const std::shared_ptr<rm_msgs::srv::SetMode::Request> request,
                std::shared_ptr<rm_msgs::srv::SetMode::Response> response) ;
        void robotStateCallback(const rm_msgs::msg::StdBotState::SharedPtr msg);
        void taskImageProcess(cv::Mat& img,double img_stamp);
    private:
        //
        rclcpp::Node::SharedPtr nh_;
        //algo tool//自瞄算法类
        SimpleAutoAimAlgo auto_aim_algo_; 
        //坐标变换工具类
        rm_projectile_motion::ProjectileTransformTool projectile_tansform_tool_;
        // ros pub
        rclcpp::Publisher<rm_msgs::msg::GimbalControl>::SharedPtr gimbal_ctrl_pub_;
        rclcpp::Publisher<rm_msgs::msg::ShootControl>::SharedPtr shoot_pub_;
        // ros sub
        rclcpp::Subscription<rm_msgs::msg::StdBotState>::SharedPtr state_info_sub_;
        // ros srv
        // 0x00暂停，0x01,正常，0x02,不控制，0x03,控制不发子弹,0x10,更新颜色配置
        rclcpp::Service<rm_msgs::srv::SetMode>::SharedPtr set_mode_srv_;
        //data
        // offset of cam-gimbal
        cv::Point3f offset_trans_; //云台相对于相机的坐标（直接测量获得），单位为cm
        float offset_pitch_;       // pitch轴偏移量
        float offset_yaw_;         // yaw轴偏移量
        // tmp data
        bool gimbal_ctrl_flag_;
        bool shoot_ctrl_flag_;
        unsigned char task_mode_ = 0x03;
        float pitch_info_;
        float pitch_coeff_;
        float yaw_coeff_;
};
}  // namespace rm_auto_aim

#endif  // RM_AUTO_AIM_TASK_AUTO_AIM_NODE_H
