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

#include "mros2_wrapper/mros2_wrapper.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


using std::placeholders::_1;
using std::placeholders::_2;

template<class ActionT>
class TestActionClient : public rclcpp::Node
{
public:
  TestActionClient(const std::string & action_name)
  : Node("test_action_client"), action_name_(action_name)
  {
  }

  void call_server()
  {
    action_client_ = rclcpp_action::create_client<ActionT>(
      shared_from_this(), action_name_);

    auto goal_msg = typename ActionT::Goal();

    auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&TestActionClient::feedback_callback, this, _1, _2);
    send_goal_options.feedback_callback =
      std::bind(&TestActionClient::result_callback, this, _1);

    auto goal_handle_future_ = action_client_->async_send_goal(goal_msg);
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
    const std::shared_ptr<const ActionT::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Feedback");
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }

    if (result.result->success) {
      RCLCPP_INFO(get_logger(), "Success");
    } else {
      RCLCPP_INFO(get_logger(), "Failed");
    }
  }

private:
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::string action_name_;
};

TEST(ActionTest, test_wrapper)
{
  using Nav2Wrapper = 
    mros2_wrapper::Mros2Wrapper<nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>;
  int executed = false;
  auto node = std::make_shared<Nav2Wrapper>("nav2_wrapper", "navigate_to_pose", [&executed] {executed = true;});
  auto test_node = std::make_shared<TestActionClient<mros2_msgs::action::NavigateToPoseQos>>("navigate_to_pose_qos");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(test_node->get_node_base_interface());

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin_some();

  test_node->call_server();

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start).seconds() <  5.0) {
    executor.spin_some();
  }

  ASSERT_TRUE(executed);
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
