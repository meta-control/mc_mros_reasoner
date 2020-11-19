// Copyright 2020 Intelligent Robotics Lab
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

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mros2_msgs/action/navigate_to_pose_qos.hpp"

#include "mros2_wrapper/mros2_wrapper.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using Nav2Wrapper = 
    mros2_wrapper::Mros2Wrapper<nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>;
  
  auto node = std::make_shared<Nav2Wrapper>("nav2_wrapper", "navigate_to_pose");
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
