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

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "rmoss_util/debug.hpp"
#include "rmoss_util/image_utils.hpp"
#include "rmoss_cam/cam_client.hpp"
#include "rmoss_auto_aim/armor_detector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // creat ros2 node
  auto node = std::make_shared<rclcpp::Node>("armor_detector_test");
  // parameters
  std::string camera_name = "camera";
  std::string robot_color = "red";
  node->declare_parameter("camera_name", camera_name);
  node->declare_parameter("robot_color", robot_color);
  node->get_parameter("camera_name", camera_name);
  node->get_parameter("robot_color", robot_color);
  // create ArmorDetector
  rmoss_auto_aim::ArmorDetector armor_detector;
  bool is_red = (robot_color == "red");
  armor_detector.set_target_color(is_red);
  rmoss_util::set_debug(true);
  auto process_image_fn = [&](cv::Mat & img, double /*img_stamp*/){
    armor_detector.process(img);
    auto results = armor_detector.getArmorVector();
    if (results.size() != 0) {
      for (auto result : results) {
        RMOSS_DEBUG(rmoss_util::draw_4points(img, result.points));
      }
      cv::imshow("result", img);
      //wait to exit
      cv::waitKey(0);
    } else {
      std::cout << "not find" << std::endl;
    }
  };
  // create cam client
  auto cam_client = std::make_shared<rmoss_cam::CamClient>(
    node, camera_name, process_image_fn, false);
  // spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
