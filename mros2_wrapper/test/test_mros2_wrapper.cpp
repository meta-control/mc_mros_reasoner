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
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mros2_msgs/action/navigate_to_pose_qos.hpp"
#include "mros2_msgs/action/control_qos.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "mros2_wrapper/mros2_wrapper.hpp"
#include "mros2_wrapper/simple_action_client.hpp"
#include "mros2_wrapper/simple_action_server.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


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
  fromRos2Feedback(
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> ros2_feedback)
  {
    auto ret = std::make_shared<mros2_msgs::action::NavigateToPoseQos::Feedback>();

    ret->current_pose = ros2_feedback->current_pose;
    ret->distance_remaining = ros2_feedback->distance_remaining;
    ret->navigation_time = ros2_feedback->navigation_time;
    ret->number_of_recoveries = ros2_feedback->number_of_recoveries;

    return ret;
  }
};

TEST(WrapperTest, client_server_tests)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin_some(node->get_node_base_interface());
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::spin_some(node->get_node_base_interface());

  using Ros2ActionServer =
    mros2_wrapper::SimpleActionServer<
    nav2_msgs::action::NavigateToPose, rclcpp_lifecycle::LifecycleNode>;
  using Ros2ActionClient =
    mros2_wrapper::SimpleActionClient<
    nav2_msgs::action::NavigateToPose, rclcpp_lifecycle::LifecycleNode>;

  std::unique_ptr<Ros2ActionServer> nav2_server;
  std::unique_ptr<Ros2ActionClient> nav2_client;

  int counter = 0;
  double distance_remaining = 5.0;
  double result_obtained = false;

  auto action_server_loop = [&]() {
      rclcpp::Rate rate(10);
      while (rclcpp::ok() && counter++ < 5) {
        std::shared_ptr<nav2_msgs::action::NavigateToPose::Feedback> feedback_msg =
          std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

        feedback_msg->distance_remaining = 5.0 - counter;
        nav2_server->publish_feedback(feedback_msg);

        rate.sleep();
      }

      nav2_server->succeeded_current();
    };

  auto feedback_callback = [&](
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
      distance_remaining = feedback->distance_remaining;
    };

  auto result_callback = [&](
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &) {
      result_obtained = true;
    };

  nav2_server = std::make_unique<Ros2ActionServer>(node, "navigate_to_pose", action_server_loop);
  nav2_client = std::make_unique<Ros2ActionClient>(
    node, "navigate_to_pose", feedback_callback, result_callback);

  nav2_server->activate();
  rclcpp::spin_some(node->get_node_base_interface());

  nav2_msgs::action::NavigateToPose::Goal goal;
  nav2_client->send_goal(goal);

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start).seconds() < 2.0) {
    rclcpp::spin_some(node->get_node_base_interface());
  }

  ASSERT_EQ(counter, 6);
  ASSERT_LT(distance_remaining, 1.0f);
  ASSERT_TRUE(result_obtained);
}

TEST(ActionTest, test_wrapper)
{
  using MissionClientT =
    mros2_wrapper::SimpleActionClient<mros2_msgs::action::NavigateToPoseQos, rclcpp::Node>;
  using NavigationServerT =
    mros2_wrapper::SimpleActionServer<nav2_msgs::action::NavigateToPose, rclcpp::Node>;
  using MetacontrollerServerT =
    mros2_wrapper::SimpleActionServer<mros2_msgs::action::ControlQos, rclcpp::Node>;

  std::unique_ptr<MissionClientT> mission_client;
  std::unique_ptr<NavigationServerT> navigation_server;
  std::unique_ptr<MetacontrollerServerT> metacontroller_server;

  auto wrapper_node = std::make_shared<NavigateToPoseWrapper>("nav2_wrapper", "navigate_to_pose");
  auto mission_node = rclcpp::Node::make_shared("mission_node");
  auto metacontroller_node = rclcpp::Node::make_shared("metacontroller_node");
  auto nav2_node = rclcpp::Node::make_shared("nav2_node");

  double last_feedback_mission = 0;
  bool result_received = false;
  // Mission component
  auto mission_feedback = [&](
    rclcpp_action::ClientGoalHandle<mros2_msgs::action::NavigateToPoseQos>::SharedPtr,
    const std::shared_ptr<const mros2_msgs::action::NavigateToPoseQos::Feedback> feedback) {
      std::cerr << "Mission: received feedback " << feedback->distance_remaining << std::endl;
      last_feedback_mission = feedback->distance_remaining;
    };
  auto mission_result = [&](
    const rclcpp_action::ClientGoalHandle<mros2_msgs::action::NavigateToPoseQos>::WrappedResult &) {
      std::cerr << "Mission: received result " << std::endl;
      result_received = true;
    };

  mission_client = std::make_unique<MissionClientT>(
    mission_node, "navigate_to_pose_qos", mission_feedback, mission_result);

  // Navigation component
  auto nav2_server_loop = [&]() {
      int counter = 0;
      rclcpp::Rate rate(1);
      while (rclcpp::ok() && counter++ < 5) {
        std::shared_ptr<nav2_msgs::action::NavigateToPose::Feedback> feedback_msg =
          std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

        feedback_msg->distance_remaining = counter;

        std::cerr << "Nav2: Sending feedback " << counter << std::endl;
        navigation_server->publish_feedback(feedback_msg);

        rate.sleep();
      }
      std::cerr << "Nav2: Sending result" << std::endl;
      navigation_server->succeeded_current();
    };

  navigation_server = std::make_unique<NavigationServerT>(
    nav2_node, "navigate_to_pose", nav2_server_loop);

  // Metacontroller component
  auto mc_server_loop = [&]() {
      rclcpp::Rate rate(10);
      while (rclcpp::ok() && !metacontroller_server->is_cancel_requested()) {
        std::shared_ptr<mros2_msgs::action::ControlQos::Feedback> feedback_msg =
          std::make_shared<mros2_msgs::action::ControlQos::Feedback>();

        std::cerr << "Metacontroller: Sending feedback" << std::endl;
        metacontroller_server->publish_feedback(feedback_msg);

        rate.sleep();
      }

      metacontroller_server->succeeded_current();
    };

  metacontroller_server = std::make_unique<MetacontrollerServerT>(

    nav2_node, "metacontrol_objective", mc_server_loop);


  // Start system
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(wrapper_node->get_node_base_interface());
  executor.add_node(mission_node->get_node_base_interface());
  executor.add_node(metacontroller_node->get_node_base_interface());
  executor.add_node(nav2_node->get_node_base_interface());

  wrapper_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  wrapper_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin_some();

  metacontroller_server->activate();
  navigation_server->activate();

  executor.spin_some();

  mros2_msgs::action::NavigateToPoseQos::Goal goal;
  mission_client->send_goal(goal);

  auto start = wrapper_node->now();
  while (rclcpp::ok() && (wrapper_node->now() - start).seconds() < 12.0) {
    executor.spin_some();
  }

  ASSERT_NEAR(last_feedback_mission, 5, 0.0001);
  ASSERT_TRUE(result_received);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  rclcpp::Rate(1).sleep();
  return result;
}
