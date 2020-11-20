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
#include "lifecycle_msgs/msg/transition.hpp"

// #include "mros2_wrapper/mros2_wrapper.hpp"
#include "mros2_wrapper/simple_action_client.hpp"
#include "mros2_wrapper/simple_action_server.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


/*class NavigateToPoseWrapper : public mros2_wrapper::Mros2Wrapper<nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>
{
  using mros2_wrapper::Mros2Wrapper<nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>::Mros2Wrapper; 

protected:

  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal>
  fromMrosGoal(std::shared_ptr<const mros2_msgs::action::NavigateToPoseQos::Goal> mros2_goal)
  {
    auto ret = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();

    ret->pose = mros2_goal->pose;
    ret->behavior_tree = mros2_goal->behavior_tree;

    return ret;
  }
};
*/

TEST(WrapperTest, client_server_tests)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin_some(node->get_node_base_interface());
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::spin_some(node->get_node_base_interface());

  using Ros2ActionServer =
    mros2_wrapper::SimpleActionServer<nav2_msgs::action::NavigateToPose, rclcpp_lifecycle::LifecycleNode>;
  using Ros2ActionClient =
    mros2_wrapper::SimpleActionClient<nav2_msgs::action::NavigateToPose, rclcpp_lifecycle::LifecycleNode>;

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
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      result_obtained = true;
    };  

  nav2_server = std::make_unique<Ros2ActionServer>(node, "navigate_to_pose", action_server_loop);
  nav2_client = std::make_unique<Ros2ActionClient>(
    node, "navigate_to_pose", feedback_callback, result_callback);

  nav2_server->activate();
  rclcpp::spin_some(node->get_node_base_interface());

  nav2_msgs::action::NavigateToPose::Goal goal;
  nav2_client->call_server(goal);

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start).seconds() <  2.0) {
    rclcpp::spin_some(node->get_node_base_interface());
  }

  ASSERT_EQ(counter, 6);
  ASSERT_LT(distance_remaining, 1.0f);
  ASSERT_TRUE(result_obtained);
}

/*TEST(ActionTest, test_wrapper)
{
  auto node = std::make_shared<NavigateToPoseWrapper>("nav2_wrapper", "navigate_to_pose");
  auto test_client_node = std::make_shared<TestActionClient<mros2_msgs::action::NavigateToPoseQos>>("navigate_to_pose_qos");
  //auto test_server_node = std::make_shared<TestActionServer<nav2_msgs::action::NavigateToPose>>("navigate_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(test_client_node->get_node_base_interface());

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin_some();

//  test_server_node->start_server();
//  
//  auto start = node->now();
//  while (rclcpp::ok() && (node->now() - start).seconds() <  1.0) {
//    executor.spin_some();
//  }


  test_client_node->call_server();

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start).seconds() <  5.0) {
    executor.spin_some();
  }

}
*/


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  
  auto result = RUN_ALL_TESTS();
  
  rclcpp::shutdown();
  rclcpp::Rate(1).sleep();
  return result;
}
