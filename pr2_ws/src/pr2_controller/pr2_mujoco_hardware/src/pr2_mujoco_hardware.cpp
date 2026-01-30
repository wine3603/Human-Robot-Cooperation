// Copyright 2021 ros2_control Development Team
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

#include "pr2_mujoco_hardware/pr2_mujoco_hardware.hpp"

#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

// using namespace elfin_hardware_interface;
namespace pr2_mujoco_hardware
{
hardware_interface::CallbackReturn Pr2MujocoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  //config ethercat manager
      if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Pr2MujocoHardware"), "Parent on_init failed");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化关节和轮子的控制缓存
    pr2_tau_commands_.assign(15, 0.0);
    pr2_joint_positions_.assign(15, 0.0);
    pr2_joint_velocities_.assign(15, 0.0);

    fl_rotation_cmd_ = fr_rotation_cmd_ = bl_rotation_cmd_ = br_rotation_cmd_ = 0.0;
    fl_rotation_state_ = fr_rotation_state_ = bl_rotation_state_ = br_rotation_state_ = 0.0;

    fl_l_cmd_ = fl_r_cmd_ = fr_l_cmd_ = fr_r_cmd_ = bl_l_cmd_ = bl_r_cmd_ = br_l_cmd_ = br_r_cmd_ = 0.0;
    fl_l_position_state_ = fl_r_position_state_ = fr_l_position_state_ = fr_r_position_state_ = bl_l_position_state_ = bl_r_position_state_ = br_l_position_state_ = br_r_position_state_ = 0.0;
    fl_l_velocity_state_ = fl_r_velocity_state_ = fr_l_velocity_state_ = fr_r_velocity_state_ = bl_l_velocity_state_ = bl_r_velocity_state_ = br_l_velocity_state_ = br_r_velocity_state_ = 0.0;

    left_robotiq_joint_ = right_robotiq_joint_ = 0.0;
    left_robotiq_joint_cmd_ = right_robotiq_joint_cmd_ = 0.0 ;
    head_pan_joint_ = head_pan_joint_cmd_ = 0.0 ;
    head_tilt_joint_ = head_tilt_joint_cmd_ = 0.0 ;
    laser_tilt_mount_joint_ = laser_tilt_mount_joint_cmd_ = 0.0 ;
    // 读取 ros2_control 传入的参数
    if (const auto it = info.hardware_parameters.find("network_interface"); it != info.hardware_parameters.end())
    {
      network_interface_ = it->second;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("Pr2MujocoHardware"), "Missing 'network_interface' parameter");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (const auto it = info.hardware_parameters.find("domain"); it != info.hardware_parameters.end())
    {
      domain_id_ = std::stoi(it->second);
    }

    // 初始化 DDS 通信
    unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

    pr2_cmd_publisher_ = std::make_shared<unitree::robot::ChannelPublisher<pr2cmd::msg::dds_::PR2ActuatorCmd >>("pr2/cmd");
    pr2_cmd_publisher_->InitChannel();

    pr2_state_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<pr2cmd::msg::dds_::PR2SensorState>>("pr2/state");

    pr2_state_subscriber_->InitChannel(
      static_cast<std::function<void(const void*)>>(
        [this](const void* msg_ptr)
        {
          const auto* msg = static_cast<const pr2cmd::msg::dds_::PR2SensorState*>(msg_ptr);
          pr2_state_ = *msg;
        }
      ),
      1);
    RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"), "Joint list:");
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"), "  [%lu] %s", i, info_.joints[i].name.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"), "on_init finished.");
  return hardware_interface::CallbackReturn::SUCCESS;

}


std::vector<hardware_interface::StateInterface> Pr2MujocoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"),"Exporting state interfaces...");
  
  size_t i = 0;
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &left_robotiq_joint_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &right_robotiq_joint_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &head_pan_joint_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &head_tilt_joint_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &laser_tilt_mount_joint_);


  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &fl_rotation_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &fr_rotation_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &bl_rotation_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &br_rotation_state_);

     // PR2 底盘8轮状态接口（位置 + 速度）
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &fl_l_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fl_l_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &fl_r_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fl_r_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &fr_l_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fr_l_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &fr_r_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fr_r_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &bl_l_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &bl_l_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &bl_r_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &bl_r_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &br_l_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &br_l_velocity_state_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &br_r_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &br_r_velocity_state_);
  size_t local_i = i;
     // PR2 机械臂关节状态接口
  for (  ;i < 15 + local_i; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pr2_joint_positions_[i-local_i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &pr2_joint_velocities_[i-local_i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pr2MujocoHardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"),"Command");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  size_t i = 0;
  // ======================
  // 2.1 gripper + head + laser (position)  [0..4]
  // ======================
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &left_robotiq_joint_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &right_robotiq_joint_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &head_pan_joint_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &head_tilt_joint_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &laser_tilt_mount_joint_cmd_);
  // ======================
  // 2.2 caster steer (position)            [5..8]
  // ======================
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &fl_rotation_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &fr_rotation_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &bl_rotation_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &br_rotation_cmd_);
  // ======================
  // 2.3 wheel velocity (velocity)          [9..16]
  // ======================
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fl_l_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fl_r_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fr_l_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &fr_r_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &bl_l_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &bl_r_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &br_l_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &br_r_cmd_);
  // ======================
  // 2.4 torso lift (effort)                [17]
  // 2.5 right arm (effort x7)              [18..24]
  // 2.6 left arm (effort x7)               [25..31]
  // ======================
  size_t local_i = i;
  for (; i < local_i + 15; ++i)
  {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &pr2_tau_commands_[i-local_i]);
  }

  return command_interfaces;
}


hardware_interface::CallbackReturn Pr2MujocoHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"), "Activate");

  // 如果你确实要检查接口是否有效
  const auto interfaces = export_command_interfaces();
  for (size_t i = 0; i < interfaces.size(); ++i)
  {
    if (std::isnan(interfaces[i].get_value()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("Pr2MujocoHardware"),
                   "Command interface value for joint [%s] is NaN.",
                   info_.joints[i].name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"), "Activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pr2MujocoHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("Pr2MujocoHardware"), "Deactivated successfully.");
  return CallbackReturn::SUCCESS;

}

hardware_interface::return_type Pr2MujocoHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Pr2MujocoHardware"), "Reading state...");
  left_robotiq_joint_       = left_robotiq_joint_cmd_;
  right_robotiq_joint_      = right_robotiq_joint_cmd_;
  head_pan_joint_           = head_pan_joint_cmd_;
  head_tilt_joint_          = head_tilt_joint_cmd_;
  laser_tilt_mount_joint_   = laser_tilt_mount_joint_cmd_;
  // =========================================================
  // 1) caster steer (position) 4个：MuJoCo 消息里有
  // =========================================================
  fl_rotation_state_ = pr2_state_.caster_rotation_q()[0];
  fr_rotation_state_ = pr2_state_.caster_rotation_q()[1];
  bl_rotation_state_ = pr2_state_.caster_rotation_q()[2];
  br_rotation_state_ = pr2_state_.caster_rotation_q()[3];
  // =========================================================
  // 2) wheels (position + velocity) 8个：MuJoCo 消息里有，索引约定：0=fl, 1=fr, 2=bl, 3=br
  // =========================================================
  // fl
  fl_l_position_state_ = pr2_state_.caster_l_wheel_q()[0];
  fl_l_velocity_state_ = pr2_state_.caster_l_wheel_dq()[0];
  fl_r_position_state_ = pr2_state_.caster_r_wheel_q()[0];
  fl_r_velocity_state_ = pr2_state_.caster_r_wheel_dq()[0];
  // fr
  fr_l_position_state_ = pr2_state_.caster_l_wheel_q()[1];
  fr_l_velocity_state_ = pr2_state_.caster_l_wheel_dq()[1];
  fr_r_position_state_ = pr2_state_.caster_r_wheel_q()[1];
  fr_r_velocity_state_ = pr2_state_.caster_r_wheel_dq()[1];
  // bl
  bl_l_position_state_ = pr2_state_.caster_l_wheel_q()[2];
  bl_l_velocity_state_ = pr2_state_.caster_l_wheel_dq()[2];
  bl_r_position_state_ = pr2_state_.caster_r_wheel_q()[2];
  bl_r_velocity_state_ = pr2_state_.caster_r_wheel_dq()[2];
  // br
  br_l_position_state_ = pr2_state_.caster_l_wheel_q()[3];
  br_l_velocity_state_ = pr2_state_.caster_l_wheel_dq()[3];
  br_r_position_state_ = pr2_state_.caster_r_wheel_q()[3];
  br_r_velocity_state_ = pr2_state_.caster_r_wheel_dq()[3];
  // =========================================================
  // 3) torso + arms：写到 pr2_joint_positions_/velocities_（全局索引 17..31）
  // =========================================================
  pr2_joint_positions_[0]  = pr2_state_.torso_lift_q();
  pr2_joint_velocities_[0] = pr2_state_.torso_lift_dq();
  // right arm (global index 18..24)
  for (int k = 0; k < 7; ++k)
  {
    pr2_joint_positions_[1 + k]  = pr2_state_.r_arm_q()[k];
    pr2_joint_velocities_[1 + k] = pr2_state_.r_arm_dq()[k];
  }
  // left arm (global index 25..31)
  for (int k = 0; k < 7; ++k)
  {
    pr2_joint_positions_[8 + k]  = pr2_state_.l_arm_q()[k];
    pr2_joint_velocities_[8 + k] = pr2_state_.l_arm_dq()[k];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pr2MujocoHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Pr2MujocoHardware"), "Writing PR2 command...");
  // ----------------------
  // 1. gripper + head + laser (position)
  // ----------------------
  pr2_cmd_.r_gripper_pos() = 0.548 - right_robotiq_joint_cmd_;
  pr2_cmd_.l_gripper_pos() = 0.548 - left_robotiq_joint_cmd_;
  pr2_cmd_.head_pan_pos()  = head_pan_joint_cmd_;
  pr2_cmd_.head_tilt_pos() = head_tilt_joint_cmd_;
  pr2_cmd_.laser_tilt_pos() = laser_tilt_mount_joint_cmd_;
  // ----------------------
  // 2. caster steer (position)
  // ----------------------
  pr2_cmd_.fl_caster_steer() = fl_rotation_cmd_;
  pr2_cmd_.fr_caster_steer() = fr_rotation_cmd_;
  pr2_cmd_.bl_caster_steer() = bl_rotation_cmd_;
  pr2_cmd_.br_caster_steer() = br_rotation_cmd_;
  // ----------------------
  // 3. wheel velocity (velocity)
  // ----------------------
  pr2_cmd_.fl_caster_l_wheel_vel() = fl_l_cmd_;
  pr2_cmd_.fl_caster_r_wheel_vel() = fl_r_cmd_;
  pr2_cmd_.fr_caster_l_wheel_vel() = fr_l_cmd_;
  pr2_cmd_.fr_caster_r_wheel_vel() = fr_r_cmd_;
  pr2_cmd_.bl_caster_l_wheel_vel() = bl_l_cmd_;
  pr2_cmd_.bl_caster_r_wheel_vel() = bl_r_cmd_;
  pr2_cmd_.br_caster_l_wheel_vel() = br_l_cmd_;
  pr2_cmd_.br_caster_r_wheel_vel() = br_r_cmd_;
  // ----------------------
  // 4. torso lift (effort)
  // ----------------------
  if (pr2_tau_commands_.size() == 15)
    pr2_cmd_.torso_lift_tau() = pr2_tau_commands_[0];
  else
    pr2_cmd_.torso_lift_tau() = 0.0;
  // ----------------------
  // 5. right arm torque (7)  global index: 18..24
  // ----------------------
  for (int k = 0; k < 7; ++k)
  {
    pr2_cmd_.r_arm_tau()[k] = pr2_tau_commands_[1 + k];
  }
  // ----------------------
  // 6. left arm torque (7)   global index: 25..31
  // ----------------------
  for (int k = 0; k < 7; ++k)
  {
    pr2_cmd_.l_arm_tau()[k] = pr2_tau_commands_[8 + k];
  }

  if (pr2_cmd_publisher_)
  {
    pr2_cmd_publisher_->Write(pr2_cmd_);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pr2_mujoco_hardware::Pr2MujocoHardware, hardware_interface::SystemInterface)
