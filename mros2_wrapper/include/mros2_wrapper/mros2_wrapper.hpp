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

#ifndef MROS2_WRAPPER__MROS2_WRAPPER_HPP_
#define MROS2_WRAPPER__MROS2_WRAPPER_HPP_

#include <string>
#include <memory>

#include "mros2_wrapper/simple_action_server.hpp"
#include "mros2_wrapper/simple_action_client.hpp"

#include "mros2_msgs/action/control_qos.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mros2_wrapper
{

using std::placeholders::_1;
using std::placeholders::_2;

template<class Ros2ActionT, class MrosActionT>
class Mros2Wrapper : public rclcpp_lifecycle::LifecycleNode
{
public:
  Mros2Wrapper(
    const std::string & node_name,
    const std::string & action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode(node_name, options),
    action_name_(action_name)
  {
  }

protected:
  using Mros2ActionServer = SimpleActionServer<MrosActionT>;
  using Ros2ActionClient = SimpleActionClient<Ros2ActionT>;
  std::unique_ptr<Mros2ActionServer> mros_action_server_;

  std::string action_name_;

  virtual std::shared_ptr<typename Ros2ActionT::Goal>
  fromMrosGoal(std::shared_ptr<const typename MrosActionT::Goal>)
  {
    RCLCPP_ERROR(
      get_logger(), "You must provide a fromMrosGoal method for %s <- %s",
      typeid(typename Ros2ActionT::Goal()).name(), typeid(typename MrosActionT::Goal).name());
    return nullptr;
  }

  virtual std::shared_ptr<typename MrosActionT::Feedback>
  fromRos2Feedback(std::shared_ptr<const typename Ros2ActionT::Feedback>)
  {
    RCLCPP_ERROR(
      get_logger(), "You must provide a fromRos2Feedback method for %s <- %s",
      typeid(typename MrosActionT::Feedback()).name(),
      typeid(typename Ros2ActionT::Feedback).name());
    return nullptr;
  }

  void manageMrosAction()
  {
    bool mros_action_finished = false;

    mros2_msgs::msg::QoS::SharedPtr last_qos_status = nullptr;

    // Call to Controlled System
    auto on_ros2_feedback = [&](
      typename rclcpp_action::ClientGoalHandle<Ros2ActionT>::SharedPtr,
      const std::shared_ptr<const typename Ros2ActionT::Feedback> feedback)
      {
        auto ret_feedback = fromRos2Feedback(feedback);

        if (last_qos_status != nullptr) {  // No feedback from Metacontroller yet
          ret_feedback->qos_status = *last_qos_status;
        }
        mros_action_server_->publish_feedback(ret_feedback);
      };

    auto on_ros2_result = [&](
      const typename rclcpp_action::ClientGoalHandle<Ros2ActionT>::WrappedResult &)
      {
        mros_action_finished = true;
        mros_action_server_->succeeded_current();
      };

    auto ros2_action_client = std::make_unique<Ros2ActionClient>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      action_name_, on_ros2_feedback, on_ros2_result);

    auto ros2_goal = fromMrosGoal(mros_action_server_->get_current_goal());
    ros2_action_client->send_goal(*ros2_goal);

    // Call to Metracontroller
    auto on_mros_feedback = [&](
      rclcpp_action::ClientGoalHandle<mros2_msgs::action::ControlQos>::SharedPtr,
      const std::shared_ptr<const mros2_msgs::action::ControlQos::Feedback> feedback)
      {
        last_qos_status = std::make_shared<mros2_msgs::msg::QoS>();
        *last_qos_status = feedback->qos_status;
      };

    auto on_mros_result = [&](
      const rclcpp_action::ClientGoalHandle<mros2_msgs::action::ControlQos>::WrappedResult &)
      {
      };

    auto mros_action_client = std::make_unique<SimpleActionClient<mros2_msgs::action::ControlQos>>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "control_qos", on_mros_feedback, on_mros_result);

    auto mros_goal = std::make_shared<mros2_msgs::action::ControlQos::Goal>();
    mros_goal->qos_expected = mros_action_server_->get_current_goal()->qos_expected;

    mros_action_client->send_goal(*mros_goal);

    // Main loop, waiting to finish the action
    while (!mros_action_finished) {
    }

    mros_action_client->abort_action();
    rclcpp::Rate(1).sleep();  // Wait until everything finishes ok
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &)
  {
    mros_action_server_ = std::make_unique<Mros2ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      action_name_ + "_qos", std::bind(&Mros2Wrapper::manageMrosAction, this), false);

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State &)
  {
    mros_action_server_->activate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &)
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State &)
  {
    return CallbackReturnT::SUCCESS;
  }
};

}  // namespace mros2_wrapper

#endif  // MROS2_WRAPPER__MROS2_WRAPPER_HPP_
