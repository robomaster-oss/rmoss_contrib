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
#ifndef RM_POWER_RUNE_2019_TASK_POWER_RUNE_HPP
#define RM_POWER_RUNE_2019_TASK_POWER_RUNE_HPP

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "rmoss_interfaces/srv/set_mode.hpp"
#include "rm_power_rune2019/simple_power_rune_algo.hpp"
#include "rm_projectile_motion/gimbal_transform_tool.hpp"
#include "rm_task/task_image_proc.hpp"


namespace rm_power_rune2019 {
enum class TaskMode { idle,
                      small,
                      large,
};
class TaskPowerRune : public rm_task::TaskImageProc {
   public:
    TaskPowerRune(rclcpp::Node::SharedPtr& nh);
    ~TaskPowerRune(){};

   private:
    bool setModeCallBack(const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
                         std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response);
    void gimbalStateCallback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg);
    void taskImageProcess(cv::Mat& img, double img_stamp);

   private:
    ////ros2 node and communcation
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_ctrl_pub_;
    rclcpp::Publisher<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_pub_;
    rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
    rclcpp::Service<rmoss_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
    ////tool
    std::shared_ptr<SimplePowerRuneAlgo> power_rune_algo_;
    std::shared_ptr<rm_projectile_motion::GimbalTransformTool> projectile_tansform_tool_;
    ////data
    // offset of cam-gimbal
    cv::Point3f offset_trans_;  //云台相对于相机的坐标（直接测量获得），单位为cm
    float offset_pitch_;        // pitch轴偏移量
    float offset_yaw_;          // yaw轴偏移量
    // tmp data
    float current_pitch_;
    bool is_need_clear_{false};
    bool is_need_reshoot_{false};
    TaskMode current_mode_{TaskMode::idle};
};
}  // namespace rm_power_rune2019

#endif  // RM_POWER_RUNE_2019_TASK_POWER_RUNE_HPP
