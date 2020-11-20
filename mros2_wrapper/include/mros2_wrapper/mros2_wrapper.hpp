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

#include "mros2_wrapper/simple_action_server.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mros2_wrapper
{

using std::placeholders::_1;
using std::placeholders::_2;

template<class Ros2ActionT, class Mros2ActionT>
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
  using Mros2ActionServer = SimpleActionServer<Mros2ActionT>;
  using Ros2ActionClient = SimpleActionClient<Ros2ActionT>;
  std::unique_ptr<Mros2ActionServer> mros_action_server_;

  std::string action_name_;

  virtual std::shared_ptr<typename Ros2ActionT::Goal>
  fromMrosGoal(std::shared_ptr<const typename Mros2ActionT::Goal> mros2_goal)
  {
    RCLCPP_ERROR(get_logger(), "You must provide a fromMrosGoal method for %s <- %s", 
      typeid(typename Ros2ActionT::Goal()).name(), typeid(typename Mros2ActionT::Goal).name());
    return nullptr;
  }

  void manageMrosAction()
  {
    std::cerr << "Performing action" << std::endl;
    bool mros_action_finished = false;
  
    auto on_ros2_feedback = [&]() {
      std::cerr << "Got feedback" << std::endl;
    }

    auto on_ros2_result = [&]() {
      std::cerr << "Got result" << std::endl;
      mros_action_finished = true;
      mros_action_server_->succeeded_current();
    }

    auto ros_action_client = std::make_unique<Ros2ActionClient>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      action_name_, on_ros2_feedback, on_ros2_result);

    ros_action_client->call_server();

    while (!mros_action_finished) {

    }
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
    std::cerr << "Action server active" << std::endl;

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