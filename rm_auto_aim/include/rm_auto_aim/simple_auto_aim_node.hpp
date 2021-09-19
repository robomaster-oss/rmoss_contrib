// Copyright 2020 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_AUTO_AIM__SIMPLE_AUTO_AIM_NODE_HPP_
#define RM_AUTO_AIM__SIMPLE_AUTO_AIM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "rm_cam/cam_client.hpp"
#include "rm_auto_aim/simple_auto_aim_algo.hpp"
#include "rm_projectile_motion/gimbal_transform_tool.hpp"

#include "rmoss_interfaces/msg/gimbal.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "rmoss_interfaces/srv/set_mode.hpp"

namespace rm_auto_aim
{
class SimpleAutoAimNode
{
public:
  explicit SimpleAutoAimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void process_image(cv::Mat & img, double img_stamp);
  bool set_mode_cb(
    const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response);
  void gimbal_state_cb(const rmoss_interfaces::msg::Gimbal::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rm_cam::CamClient> cam_client_;
  // 自瞄算法类
  std::shared_ptr<SimpleAutoAimAlgo> auto_aim_algo_;
  // 坐标变换工具类
  std::shared_ptr<rm_projectile_motion::GimbalTransformTool> gimbal_tansform_tool_;
  // ros pub
  rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_cmd_pub_;
  // ros sub
  rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
  // ros srv
  rclcpp::Service<rmoss_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  // data
  bool debug_{false};
  // offset of cam-gimbal
  cv::Point3f offset_trans_;       // 云台相对于相机的坐标（直接测量获得），单位为cm
  float offset_pitch_;             // pitch轴偏移量
  float offset_yaw_;               // yaw轴偏移量
  // tmp data
  bool gimbal_ctrl_flag_{true};
  bool shoot_ctrl_flag_{true};
  float current_pitch_{0};
};
}  // namespace rm_auto_aim

#endif  // RM_AUTO_AIM__SIMPLE_AUTO_AIM_NODE_HPP_
