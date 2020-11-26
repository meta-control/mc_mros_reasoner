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

#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

#include "mros2_wrapper/mros2_wrapper.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


using namespace std::chrono_literals;
class NavigateToPoseWrapper : public mros2_wrapper::Mros2Wrapper<
    nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>
{
  using mros2_wrapper::Mros2Wrapper<
    nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>::Mros2Wrapper;

public:
  void init()
  {
    navigation_manage_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_navigation/manage_nodes");
    localization_manage_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "/lifecycle_manager_localization/manage_nodes");
    last_reconfiguration_ = now();
  }
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

  void process_feedback_mech(mros2_msgs::msg::QoS feedback)
  {
    // At this moment nav2_reconfig is performed using the System Modes pkg.
    if (previous_mode_ == "" && feedback.selected_mode != "") {
      previous_mode_ = feedback.selected_mode;
      //  nav2_reconfig(); 
    } else if (feedback.selected_mode != previous_mode_) {
      RCLCPP_DEBUG(get_logger(), "New mode selected: [%s]", feedback.selected_mode.c_str());
      //  nav2_reconfig();
    }
  }

  void nav2_reconfig()
  {
    if (now() - last_reconfiguration_ > rclcpp::Duration(5s)) {
      last_reconfiguration_ = now(); 
      auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::RESET;

      while (!navigation_manage_client_->wait_for_service(1s) && !localization_manage_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(get_logger(), "Navigation manage service not available, waiting again...");
      }
      auto manage_nav_result = navigation_manage_client_->async_send_request(request);
      auto manage_loc_result = localization_manage_client_->async_send_request(request);
      while(rclcpp::ok()) {
        if (manage_loc_result.valid() && manage_nav_result.valid()) 
        {
          request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
          manage_loc_result = localization_manage_client_->async_send_request(request);
          manage_nav_result = navigation_manage_client_->async_send_request(request);
          break;
        }
      }
    }
  }

private:
  rclcpp::Time last_reconfiguration_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_manage_client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_manage_client_;
  std::string previous_mode_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavigateToPoseWrapper>("nav2_wrapper", "navigate_to_pose");
  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
