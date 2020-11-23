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

#include <memory>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mros2_msgs/action/navigate_to_pose_qos.hpp"

#include "mros2_wrapper/mros2_wrapper.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class NavigateToPoseWrapper : public mros2_wrapper::Mros2Wrapper<
    nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>
{
  using mros2_wrapper::Mros2Wrapper<
    nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>::Mros2Wrapper;

protected:
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal>
  fromMrosGoal(std::shared_ptr<const mros2_msgs::action::NavigateToPoseQos::Goal> mros_goal)
  {
    auto ret = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();

    ret->pose = mros_goal->pose;
    ret->behavior_tree = mros_goal->behavior_tree;

    return ret;
  }

  std::shared_ptr<mros2_msgs::action::NavigateToPoseQos::Feedback>
  fromRos2Feedback(std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> ros2_feedback)
  {
    auto ret = std::make_shared<mros2_msgs::action::NavigateToPoseQos::Feedback>();

    ret->current_pose = ros2_feedback->current_pose;
    ret->distance_remaining = ros2_feedback->distance_remaining;
    ret->navigation_time = ros2_feedback->navigation_time;
    ret->number_of_recoveries = ros2_feedback->number_of_recoveries;

    return ret;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using Nav2Wrapper =
    mros2_wrapper::Mros2Wrapper<
    nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>;

  auto node = std::make_shared<Nav2Wrapper>("nav2_wrapper", "navigate_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
