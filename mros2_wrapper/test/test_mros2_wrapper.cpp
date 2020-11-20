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
    std::cerr << "Calling server...";
    action_client_ = rclcpp_action::create_client<ActionT>(
      shared_from_this(), action_name_);

    auto goal_msg = typename ActionT::Goal();

    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&TestActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TestActionClient::result_callback, this, _1);

    auto goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    std::cerr << "done" << std::endl;
  }

  void feedback_callback(
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Feedback");
  }

  void result_callback(const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
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
  }

private:
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::string action_name_;
};

template<class ActionT>
class TestActionServer : public rclcpp::Node
{
public:
  TestActionServer(const std::string & action_name)
  : Node("test_action_server"), action_name_(action_name)
  {
  }

  void start_server()
  {
    action_server_ = rclcpp_action::create_server<ActionT>(
      shared_from_this(),
      action_name_,
      std::bind(&TestActionServer::handle_goal, this, _1, _2),
      std::bind(&TestActionServer::handle_cancel, this, _1),
      std::bind(&TestActionServer::handle_accepted, this, _1));

      std::cerr << "server started" << std::endl;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename ActionT::Goal> goal)
  {
    std::cerr << "Received action request in ROS2 action server" << std::endl;
    current_goal_ = *goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<typename rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<typename rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    current_handle_ = goal_handle;
    execution_future_ = std::async(std::launch::async, [this]() {work();});
  }

  void work()
  {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    auto result = std::make_shared<typename ActionT::Result>();

    rclcpp::Rate loop_rate(1);
    int current_times = 0;
    while (rclcpp::ok() && current_times < 3) {
      current_times++;

      std::cerr << current_times << std::endl;

      current_handle_->publish_feedback(feedback);
      loop_rate.sleep();
    }

  current_handle_->publish_feedback(feedback);
  }

private:
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  typename ActionT::Goal current_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::string action_name_;
  std::future<void> execution_future_;
};


class NavigateToPoseWrapper : public mros2_wrapper::Mros2Wrapper<nav2_msgs::action::NavigateToPose, mros2_msgs::action::NavigateToPoseQos>
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

TEST(ActionTest, test_wrapper)
{
  auto node = std::make_shared<NavigateToPoseWrapper>("nav2_wrapper", "navigate_to_pose");
  auto test_client_node = std::make_shared<TestActionClient<mros2_msgs::action::NavigateToPoseQos>>("navigate_to_pose_qos");
  auto test_server_node = std::make_shared<TestActionServer<nav2_msgs::action::NavigateToPose>>("navigate_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(test_client_node->get_node_base_interface());

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor.spin_some();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor.spin_some();

  test_server_node->start_server();
  
  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start).seconds() <  1.0) {
    executor.spin_some();
  }


  test_client_node->call_server();

  start = node->now();
  while (rclcpp::ok() && (node->now() - start).seconds() <  5.0) {
    executor.spin_some();
  }

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
