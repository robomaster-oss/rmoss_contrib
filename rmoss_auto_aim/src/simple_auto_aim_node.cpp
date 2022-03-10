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

using namespace std::chrono_literals;

namespace rmoss_auto_aim{

SimpleAutoAimNode::SimpleAutoAimNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("simple_auto_aim", options);
  bool autostart = false;
  // parameters
  node_->declare_parameter("target_color", target_color_);
  node_->declare_parameter("debug", debug_);
  node_->declare_parameter("camera_name", camera_name_);
  node_->declare_parameter("autostart", autostart);
  node_->get_parameter("target_color", target_color_);
  node_->get_parameter("debug", debug_);
  node_->get_parameter("camera_name", camera_name_);
  node_->get_parameter("autostart", autostart);
  rmoss_util::set_debug(debug_);
  // create pub,sub,srv
  using namespace std::placeholders;
  gimbal_cmd_pub_ = node_->create_publisher<rmoss_interfaces::msg::GimbalCmd>(
    "robot_base/gimbal_cmd", 10);
  shoot_cmd_pub_ = node_->create_publisher<rmoss_interfaces::msg::ShootCmd>(
    "robot_base/shoot_cmd", 10);
  set_color_srv_ = node_->create_service<rmoss_interfaces::srv::SetColor>(
    "auto_aim/set_color", 
    [this](const rmoss_interfaces::srv::SetColor::Request::SharedPtr request,
          rmoss_interfaces::srv::SetColor::Response::SharedPtr response){
      response->success = true;
      this->set_color(request->color == request->RED);
      return true;
    });
  // init tool
  cam_client_ = std::make_shared<rmoss_cam::CamClient>(node_);
  gimbal_tansformoss_tool_ = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>();
  auto_aim_algo_ = std::make_shared<SimpleAutoAimAlgo>();
  // init task manager
  task_manager_ = std::make_shared<rmoss_util::TaskManager>(
    node_,
    [this](){ return this->get_task_status_cb(); },
    [this](rmoss_util::TaskCmd cmd){ return this->control_task_cb(cmd); });
  // timer for init
  init_timer_ = node_->create_wall_timer(
    0s, [this]() {
      init_timer_->cancel();
      this->init();
    });
  if (autostart) {
    run_flag_ = true;
  }
}

void SimpleAutoAimNode::init(){
  // get camera info
  sensor_msgs::msg::CameraInfo info;
  if(!cam_client_->get_camera_info(info)){
    RCLCPP_ERROR(node_->get_logger(), "get camera info failed!");
    return;
  }
  // set camera info
  std::vector<double> camera_k(9, 0);
  std::copy_n(info.k.begin(), 9, camera_k.begin());
  auto_aim_algo_->set_camera_info(camera_k, info.d);
  // set enemy robot color
  bool is_red = (target_color_ == "red");
  auto_aim_algo_->set_target_color(is_red);
  cam_client_->connect(
    camera_name_,
    [this](const cv::Mat & img, const rclcpp::Time & stamp)
    {
      this->process_image(img, stamp);
    });
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

void SimpleAutoAimNode::set_color(bool is_red)
{
  if (is_red) {
    // red color config
    auto_aim_algo_->set_target_color(true);
    RCLCPP_INFO(node_->get_logger(), "set target color red");
  } else {
    // blue color config
    auto_aim_algo_->set_target_color(false);
    RCLCPP_INFO(node_->get_logger(), "set target color blue");
  }
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