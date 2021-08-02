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
#ifndef RM_AUTO_AIM_TASK_AUTO_AIM_HPP
#define RM_AUTO_AIM_TASK_AUTO_AIM_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "rm_util/image_task_server.hpp"
#include "rm_auto_aim/simple_auto_aim_algo.hpp"
#include "rm_projectile_motion/gimbal_transform_tool.hpp"

#include "rmoss_interfaces/msg/gimbal.hpp" 
#include "rmoss_interfaces/msg/gimbal_cmd.hpp" 
#include "rmoss_interfaces/msg/shoot_cmd.hpp" 
#include "rmoss_interfaces/srv/set_mode.hpp" 

namespace rm_auto_aim {
class AutoAimServer{
    public:
        AutoAimServer(rclcpp::Node::SharedPtr &nh);
        ~AutoAimServer();
    private:
        void process_image(cv::Mat& img, double img_stamp);
        bool setModeCallBack(const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
                std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response) ;
        void gimbalStateCallback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg);
        void taskImageProcess(cv::Mat& img,double img_stamp);
    private:
        //
        rclcpp::Node::SharedPtr nh_;
        //
        std::shared_ptr<rm_util::ImageTaskServer> image_task_server_;
        //自瞄算法类
        SimpleAutoAimAlgo auto_aim_algo_; 
        //坐标变换工具类
        rm_projectile_motion::GimbalTransformTool projectile_tansform_tool_;
        // ros pub
        rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
        rclcpp::Publisher<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_cmd_pub_;
        // ros sub
        rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
        // ros srv
        rclcpp::Service<rmoss_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
        //data
        // offset of cam-gimbal
        cv::Point3f offset_trans_; //云台相对于相机的坐标（直接测量获得），单位为cm
        float offset_pitch_;       // pitch轴偏移量
        float offset_yaw_;         // yaw轴偏移量
        // tmp data
        bool gimbal_ctrl_flag_;
        bool shoot_ctrl_flag_;
        float current_pitch_;
};
}  // namespace rm_auto_aim

#endif  // RM_AUTO_AIM_TASK_AUTO_AIM_HPP
