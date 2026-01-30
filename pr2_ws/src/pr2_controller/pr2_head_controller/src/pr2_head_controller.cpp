// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "pr2_head_controller/pr2_head_controller.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <realtime_tools/realtime_buffer.h>
namespace pr2_head_controller
{
  
static inline double clamp(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

controller_interface::InterfaceConfiguration Pr2HandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  command_interfaces_config.names.push_back(std::string(HEAD_PAN_JOINT) + "/" + hardware_interface::HW_IF_POSITION);
  command_interfaces_config.names.push_back(std::string(HEAD_TILT_JOINT) + "/" + hardware_interface::HW_IF_POSITION);
  command_interfaces_config.names.push_back(std::string(LASER_TILT_JOINT) + "/" + hardware_interface::HW_IF_POSITION);
  
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration Pr2HandController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::return_type Pr2HandController::update(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& period)
{
  auto joy_command = joy_command_ptr_.readFromRT();
  if (!joy_command || !(*joy_command)) {
    return controller_interface::return_type::OK;
  }
  const auto& axes = (*joy_command)->axes;
  const double dt = period.seconds();

  auto axis = [&](int i) -> double
  {
    if (i < 0 || static_cast<size_t>(i) >= axes.size()) {
      return 0.0;
    }
    return std::clamp(axes[i], -1.0f, 1.0f);
  };

// ================= head_pan =================
  double a_pan = std::clamp((double)axes[0], -1.0, 1.0);
  double head_pan_pos = 0.0;

  if (a_pan >= 0.0) {
    // [0, 1] -> [0, HEAD_PAN_MAX]
    head_pan_pos = a_pan * HEAD_PAN_MAX;
  } else {
    // [-1, 0) -> [HEAD_PAN_MIN, 0]
    head_pan_pos = (-a_pan) * HEAD_PAN_MIN;
  }
  head_pan_pos = std::clamp(head_pan_pos, HEAD_PAN_MIN, HEAD_PAN_MAX);

  // ================= head_tilt =================
  double a_tilt = std::clamp((double)axes[1], -1.0, 1.0);
  double head_tilt_pos = 0.0;

  if (a_tilt >= 0.0) {
    // [0, 1] -> [0, HEAD_TILT_MAX]
    head_tilt_pos = a_tilt * HEAD_TILT_MAX;
  } else {
    // [-1, 0) -> [HEAD_TILT_MIN, 0]
    head_tilt_pos = (-a_tilt) * HEAD_TILT_MIN;
  }
  head_tilt_pos = std::clamp(head_tilt_pos, HEAD_TILT_MIN, HEAD_TILT_MAX);

  // ================= laser tilt =================
  // 左右扳机差值，保持 0 点
  double a_laser = std::clamp(
      (double)axes[5] - (double)axes[2], -1.0, 1.0);

  double laser_pos = 0.0;
  if (a_laser >= 0.0) {
    laser_pos = a_laser * LASER_TILT_MAX;
  } else {
    laser_pos = (-a_laser) * LASER_TILT_MIN;
  }
  laser_pos = std::clamp(laser_pos, LASER_TILT_MIN, LASER_TILT_MAX);

  // 写 command interface（3 个 position）
  command_interfaces_[0].set_value(head_pan_pos);
  command_interfaces_[1].set_value(head_tilt_pos);
  command_interfaces_[2].set_value(laser_pos);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Pr2HandController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Check command interfaces.
  if (command_interfaces_.size() != 3)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %d command interfaces, but got %zu.", 2,
                 command_interfaces_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  command_interfaces_[0].set_value(head_pan_pos_);
  command_interfaces_[1].set_value(head_tilt_pos_);
  command_interfaces_[2].set_value(laser_tilt_pos_);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Pr2HandController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_interfaces_[0].set_value(0.0);
  command_interfaces_[1].set_value(0.0);
  command_interfaces_[2].set_value(0.0);
  try
  {
    reactivate_head_srv_.reset();
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Pr2HandController::on_init()
{
  std::string joy_topic_name=get_node()->get_parameter("joy_topic_name").as_string();
  joy_command_subsciption_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(joy_topic_name, rclcpp::SystemDefaultsQoS(), [this](const  sensor_msgs::msg::Joy::SharedPtr twist)
  {
    joy_command_ptr_.writeFromNonRT(twist);
  });
   // 初始位置
  head_pan_pos_   = 0.0;
  head_tilt_pos_  = 0.0;
  laser_tilt_pos_ = 0.0;
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool Pr2HandController::reactivateHand(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                                    std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].set_value(ASYNC_WAITING);
  command_interfaces_[REACTIVATE_GRIPPER_CMD].set_value(1.0);

  while (command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_value() == ASYNC_WAITING)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  resp->success = command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_value();

  return resp->success;
}
}  // namespace pr2_head_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pr2_head_controller::Pr2HandController, controller_interface::ControllerInterface)
