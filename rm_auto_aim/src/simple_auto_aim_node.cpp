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

#include "rm_auto_aim/simple_auto_aim_node.hpp"

#include "rm_util/debug.hpp"

namespace rm_auto_aim{

SimpleAutoAimNode::SimpleAutoAimNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("simple_auto_aim", options);
  std::string camera_name = "camera";
  std::string robot_color = "red";
  bool auto_start = false;
  // parameters
  node_->declare_parameter("robot_color", robot_color);
  node_->declare_parameter("debug", debug_);
  node_->declare_parameter("camera_name", camera_name);
  node_->declare_parameter("auto_start", auto_start);
  node_->get_parameter("robot_color", robot_color);
  node_->get_parameter("debug", debug_);
  node_->get_parameter("camera_name", camera_name);
  node_->get_parameter("auto_start", auto_start);
  bool is_red = (robot_color == "red");
  rm_util::set_debug(debug_);
  // create pub,sub,srv
  using namespace std::placeholders;
  gimbal_cmd_pub_ = node_->create_publisher<rmoss_interfaces::msg::GimbalCmd>(
    "robot_base/gimbal_cmd", 10);
  shoot_cmd_pub_ = node_->create_publisher<rmoss_interfaces::msg::ShootCmd>(
    "robot_base/shoot_cmd", 10);
  gimbal_state_sub_ = node_->create_subscription<rmoss_interfaces::msg::Gimbal>(
    "robot_base/gimbal_state", 10, std::bind(&SimpleAutoAimNode::gimbal_state_cb, this, _1));
  set_mode_srv_ = node_->create_service<rmoss_interfaces::srv::SetMode>(
    "task_auto_aim/set_mode", std::bind(&SimpleAutoAimNode::set_mode_cb, this, _1, _2));
  // create image task server
  cam_client_ = std::make_shared<rm_cam::CamClient>(
    node_, camera_name, std::bind(&SimpleAutoAimNode::process_image, this, _1, _2), true);
  // wait camera parameters camera_k,camera_d,camera_p
  sensor_msgs::msg::CameraInfo info;
  if(!cam_client_->get_camera_info(info)){
    RCLCPP_ERROR(node_->get_logger(), "get camera info failed!");
    return;
  }
  std::ostringstream oss;
  oss << "k:";
  for (auto & x : info.k) {
    oss << x <<" ";
  }
  oss << ",d:";
  for (auto & x : info.d) {
    oss << x <<" ";
  }
  RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());
  // init tool class
  std::vector<double> camera_k(9, 0);
  std::copy_n(info.k.begin(), 9, camera_k.begin());
  auto_aim_algo_ = std::make_shared<SimpleAutoAimAlgo>(camera_k, info.d);
  // set enemy robot color
  auto_aim_algo_->set_target_color(!is_red);
  gimbal_tansform_tool_ = std::make_shared<rm_projectile_motion::GimbalTransformTool>();
  //gimbal_tansform_tool_->set_projectile_solver(NULL);
  RCLCPP_INFO(node_->get_logger(), "init successfully!");
  if (auto_start) {
    cam_client_->start();
    RCLCPP_INFO(node_->get_logger(), "auto start!");
  }
}

void SimpleAutoAimNode::process_image(cv::Mat & img, double /*img_stamp*/)
{
  int ret = auto_aim_algo_->process(img, current_pitch_);
  if (ret == 0) {
    ArmorTarget target = auto_aim_algo_->getTarget();
    target.postion = target.postion / 100;
    // transform frame (tmp)
    cv::Point3f position;
    position.x = target.postion.z;
    position.y = target.postion.x;
    position.z = target.postion.y;
    // transform 
    float pitch, yaw;
    if (!gimbal_tansform_tool_->solve(position, pitch, yaw)) {
      RCLCPP_ERROR(node_->get_logger(), "transform failed: (%f,%f,%f)", position.x,position.y,position.z);
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "find target: (%f,%f,%f)", position.x,position.y,position.z);
    //发布云台控制topic
    if (gimbal_ctrl_flag_) {
      rmoss_interfaces::msg::GimbalCmd gimbal_cmd;
      gimbal_cmd.type = 0x00;
      gimbal_cmd.position.pitch = pitch;
      gimbal_cmd.position.yaw = yaw;
      gimbal_cmd_pub_->publish(gimbal_cmd);
    }
    //发射子弹
    if (shoot_ctrl_flag_) {
      rmoss_interfaces::msg::ShootCmd shoot_cmd;
      shoot_cmd.projectile_num = 3;
      shoot_cmd_pub_->publish(shoot_cmd);
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "not find target!");     //表示没有发现目标
  }
}

bool SimpleAutoAimNode::set_mode_cb(
  const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response)
{
  // 0x00,休眠模式，0x01:自动射击模式，0x02：自动瞄准模式（不发子弹）,0x03,测试模式,不控制.
  // 0x10,设置目标为红色，0x11,设置目标为蓝色
  response->success = true;
  if (request->mode == 0x00) {
    cam_client_->stop();
  } else if (request->mode == 0x01) {
    cam_client_->start();
    gimbal_ctrl_flag_ = true;
    shoot_ctrl_flag_ = true;
  } else if (request->mode == 0x02) {
    cam_client_->start();
    gimbal_ctrl_flag_ = true;
    shoot_ctrl_flag_ = false;
  } else if (request->mode == 0x03) {
    cam_client_->start();
    gimbal_ctrl_flag_ = false;
    shoot_ctrl_flag_ = false;
  } else if (request->mode == 0x10) {
    //red color config
    auto_aim_algo_->set_target_color(true);
    RCLCPP_INFO(node_->get_logger(), "set target color red");
  } else if (request->mode == 0x11) {
    //blue color config
    auto_aim_algo_->set_target_color(false);
    RCLCPP_INFO(node_->get_logger(), "set target color blue");
  } else {
    response->success = false;
  }
  return true;
}

void SimpleAutoAimNode::gimbal_state_cb(const rmoss_interfaces::msg::Gimbal::SharedPtr msg)
{
  current_pitch_ = msg->pitch;
}

}  // namespace rm_auto_aim