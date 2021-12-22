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

#include "rmoss_auto_aim/simple_auto_aim_node.hpp"

#include "rmoss_util/debug.hpp"

namespace rmoss_auto_aim{

SimpleAutoAimNode::SimpleAutoAimNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("simple_auto_aim", options);
  std::string camera_name = "camera";
  std::string target_color = "red";
  bool autostart = false;
  // parameters
  node_->declare_parameter("target_color", target_color);
  node_->declare_parameter("debug", debug_);
  node_->declare_parameter("camera_name", camera_name);
  node_->declare_parameter("autostart", autostart);
  node_->get_parameter("target_color", target_color);
  node_->get_parameter("debug", debug_);
  node_->get_parameter("camera_name", camera_name);
  node_->get_parameter("autostart", autostart);
  bool is_red = (target_color == "red");
  rmoss_util::set_debug(debug_);
  // create pub,sub,srv
  using namespace std::placeholders;
  gimbal_cmd_pub_ = node_->create_publisher<rmoss_interfaces::msg::GimbalCmd>(
    "robot_base/gimbal_cmd", 10);
  shoot_cmd_pub_ = node_->create_publisher<rmoss_interfaces::msg::ShootCmd>(
    "robot_base/shoot_cmd", 10);
  gimbal_state_sub_ = node_->create_subscription<rmoss_interfaces::msg::Gimbal>(
    "robot_base/gimbal_state", 10, std::bind(&SimpleAutoAimNode::gimbal_state_cb, this, _1));
  set_color_srv_ = node_->create_service<rmoss_interfaces::srv::SetColor>(
    "auto_aim/set_color", std::bind(&SimpleAutoAimNode::set_color_cb, this, _1, _2));
  // create image task client
  cam_client_ = std::make_shared<rmoss_cam::CamClient>(
    node_, camera_name, std::bind(&SimpleAutoAimNode::process_image, this, _1, _2), false);
  // init task manager
  task_manager_ = std::make_shared<rmoss_util::TaskManager>(
    node_,
    std::bind(&SimpleAutoAimNode::get_task_status_cb, this),
    std::bind(&SimpleAutoAimNode::control_task_cb, this, _1));
  // wait camera parameters camera_k,camera_d,camera_p
  sensor_msgs::msg::CameraInfo info;
  if(!cam_client_->get_camera_info(info)){
    RCLCPP_ERROR(node_->get_logger(), "get camera info failed!");
    return;
  }
  // RMOSS_DEBUG(std::cout << "camera info :" << std::endl);
  // init tool class
  std::vector<double> camera_k(9, 0);
  std::copy_n(info.k.begin(), 9, camera_k.begin());
  auto_aim_algo_ = std::make_shared<SimpleAutoAimAlgo>(camera_k, info.d);
  trans_gc_ = Eigen::Isometry3d::Identity();
  for(int i=0;i<12;i++){
    trans_gc_(i/4,i%4) = info.p[i];
  }
  RMOSS_DEBUG(std::cout << "trans_gc_:\n" << trans_gc_.matrix() << std::endl);
  // set enemy robot color
  auto_aim_algo_->set_target_color(is_red);
  gimbal_tansformoss_tool_ = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>();
  //gimbal_tansformoss_tool_->set_projectile_solver(NULL);
  if (autostart) {
    run_flag_ = true;
  }
  RCLCPP_INFO(node_->get_logger(), "init successfully!");
}

void SimpleAutoAimNode::process_image(const cv::Mat & img, const rclcpp::Time &/*stamp*/)
{
  if (!run_flag_) {
    return;
  }
  int ret = auto_aim_algo_->process(img, current_pitch_);
  if (ret != 0) {
    RCLCPP_INFO(node_->get_logger(), "not find target!");     //表示没有发现目标
    return;
  }
  ArmorTarget target = auto_aim_algo_->getTarget();
  target.postion = target.postion / 100;
  Eigen::Vector3d point(target.postion.x,target.postion.y,target.postion.z);
  // trans_bg : 云台到云台基座(共坐标原点)的坐标变换 T_{base_gimbal}, trans_bg由current_pitch_计算
  Eigen::Isometry3d trans_bg = Eigen::Isometry3d::Identity();
  // transform frame {camera frame -> gimbal frame -> gimbal base frame}
  point = trans_bg * trans_gc_ * point;
  // calcaule angle of gimbal (in gimbal base frame)
  double pitch, yaw;
  if (!gimbal_tansformoss_tool_->solve(point, pitch, yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "transform failed: (%.3lf,%.3lf,%.3lf)", point(0), point(1), point(2));
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "find target: (%.3lf,%.3lf,%.3lf)", point(0), point(1), point(2));

  // 发布云台控制topic,relative angle
  rmoss_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.position.pitch = pitch - current_pitch_;
  gimbal_cmd.position.yaw = yaw;
  gimbal_cmd_pub_->publish(gimbal_cmd);
  
  //发射子弹
  if(fabs(yaw) < 0.01){
    rmoss_interfaces::msg::ShootCmd shoot_cmd;
    shoot_cmd.projectile_num = 3;
    shoot_cmd_pub_->publish(shoot_cmd);
  }
}

bool SimpleAutoAimNode::set_color_cb(
    const rmoss_interfaces::srv::SetColor::Request::SharedPtr request,
    rmoss_interfaces::srv::SetColor::Response::SharedPtr response)
{
  response->success = true;
  if (request->color == request->RED) {
    // red color config
    auto_aim_algo_->set_target_color(true);
    RCLCPP_INFO(node_->get_logger(), "set target color red");
  } else if (request->color == request->BLUE) {
    // blue color config
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

rmoss_util::TaskStatus SimpleAutoAimNode::get_task_status_cb(){
  return rmoss_util::TaskStatus::Running;
}

bool SimpleAutoAimNode::control_task_cb(rmoss_util::TaskCmd cmd){
  if (cmd == rmoss_util::TaskCmd::Start) {
    run_flag_ = true;
  } else if (cmd == rmoss_util::TaskCmd::Stop) {
    run_flag_ = false;
  } else {
    return false;
  }
  return true;
}

}  // namespace rmoss_auto_aim