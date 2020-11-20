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

#ifndef MROS2_WRAPPER__SIMPLE_ACTION_CLIENT_HPP_
#define MROS2_WRAPPER__SIMPLE_ACTION_CLIENT_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mros2_wrapper
{

template<typename ActionT, typename nodeT = rclcpp::Node>
class SimpleActionClient
{
public:
  using GoalHandleAction = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using GoalHandleActionWrappedResult =
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

  typedef std::function<void (
        std::shared_ptr<GoalHandleAction>,
        const std::shared_ptr<const typename ActionT::Feedback>
      )> FeedbackCallback;

  typedef std::function<void (const GoalHandleActionWrappedResult &)> ResultCallback;

  explicit SimpleActionClient(
    typename nodeT::SharedPtr node,
    const std::string & action_name,
    FeedbackCallback feedback_callback,
    ResultCallback result_callback)
  : SimpleActionClient(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name, feedback_callback, result_callback)
  {}

  explicit SimpleActionClient(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string & action_name,
    FeedbackCallback feedback_callback,
    ResultCallback result_callback)
  : node_base_interface_(node_base_interface),
    node_graph_interface_(node_graph_interface),
    node_logging_interface_(node_logging_interface),
    node_waitables_interface_(node_waitables_interface),
    action_name_(action_name)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(
      node_base_interface_,
      node_graph_interface_,
      node_logging_interface_,
      node_waitables_interface_,
      action_name_);

    client_goal_options_.feedback_callback = feedback_callback;
    client_goal_options_.result_callback = result_callback;
  }

  void send_goal(const typename ActionT::Goal & goal)
  {
    execution_future_ = std::async(std::launch::async, [goal, this]() {work(goal);});
  }

  void abort_action()
  {
    action_client_->async_cancel_goal(goal_handle_future_.get());
  }

  void work(const typename ActionT::Goal & goal)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "Action server not available after waiting");
      return;
    }

    goal_handle_future_ = action_client_->async_send_goal(
      goal, client_goal_options_);

    while (!goal_handle_future_.valid()) {
    }

    auto goal_handle = goal_handle_future_.get();
    if (!goal_handle) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "ExecutorClient: Execution was rejected by the action server");
      return;
    }
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
  std::string action_name_;

  typename rclcpp_action::Client<ActionT>::SendGoalOptions client_goal_options_;
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  std::future<void> execution_future_;
  std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>
  goal_handle_future_;
};

}  // namespace mros2_wrapper

#endif   // MROS2_WRAPPER__SIMPLE_ACTION_CLIENT_HPP_
