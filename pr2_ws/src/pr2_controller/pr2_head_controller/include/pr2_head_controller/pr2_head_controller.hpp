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

#pragma once

#include "controller_interface/controller_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <realtime_tools/realtime_buffer.h>

#define HEAD_PAN_JOINT        "head_pan_joint"
#define HEAD_TILT_JOINT       "head_tilt_joint"
#define LASER_TILT_JOINT      "laser_tilt_mount_joint"

// joystick axis mapping
// 左摇杆
#define AXIS_HEAD_PAN         0   // left stick X
#define AXIS_HEAD_TILT        1   // left stick Y

// 扳机
#define AXIS_LEFT_TRIGGER     2
#define AXIS_RIGHT_TRIGGER    5

// velocity scale (rad/s when axis = 1)
#define HEAD_PAN_VEL_SCALE    1.0
#define HEAD_TILT_VEL_SCALE   1.0
#define LASER_TILT_VEL_SCALE  1.0

// position limits
#define HEAD_PAN_MIN         -2.645
#define HEAD_PAN_MAX          2.645

#define HEAD_TILT_MIN        -0.471238
#define HEAD_TILT_MAX         1.39626

#define LASER_TILT_MIN       -0.65
#define LASER_TILT_MAX        0.65

namespace pr2_head_controller
{
class Pr2HandController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  bool reactivateHand(std_srvs::srv::Trigger::Request::SharedPtr req,
                         std_srvs::srv::Trigger::Response::SharedPtr resp);

  static constexpr double ASYNC_WAITING = 0.0;
  enum CommandInterfaces
  {
    REACTIVATE_GRIPPER_CMD,
    REACTIVATE_GRIPPER_RESPONSE
  };

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_command_subsciption_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::Joy::SharedPtr> joy_command_ptr_;

  double head_pan_pos_{0.0};
  double head_tilt_pos_{0.0};
  double laser_tilt_pos_{0.0};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reactivate_head_srv_;
};
}  // namespace pr2_head_controller
