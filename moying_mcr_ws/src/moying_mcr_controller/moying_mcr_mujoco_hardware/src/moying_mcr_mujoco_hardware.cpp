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

#include "moying_mcr_mujoco_hardware/moying_mcr_mujoco_hardware.hpp"

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
namespace moying_mcr_mujoco_hardware
{
hardware_interface::CallbackReturn MoyingMcrMujocoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  //config ethercat manager
      if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Parent on_init failed");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化关节和轮子的控制缓存
    mcr_tau_commands_.assign(12, 0.0);
    mcr_joint_positions_.assign(12, 0.0);
    mcr_joint_velocities_.assign(12, 0.0);
    mcr_joint_efforts_.assign(12, 0.0);

    forward_right_cmd_ = 0.0;
    forward_left_cmd_ = 0.0;
    back_right_cmd_ = 0.0;
    back_left_cmd_ = 0.0;

    forward_right_velocity_state_ = 0.0;
    forward_left_velocity_state_ = 0.0;
    back_right_velocity_state_ = 0.0;
    back_left_velocity_state_ = 0.0;

    forward_right_position_state_ = 0.0;
    forward_left_position_state_ = 0.0;
    back_right_position_state_ = 0.0;
    back_left_position_state_ = 0.0;
    left_robotiq_joint_ = left_robotiq_joint_cmd_ = left_dummy_effort_limit_ = 0.0;
    right_robotiq_joint_ = right_robotiq_joint_cmd_ = right_dummy_effort_limit_ = 0.0 ;
    // 读取 ros2_control 传入的参数
    if (const auto it = info.hardware_parameters.find("network_interface"); it != info.hardware_parameters.end())
    {
      network_interface_ = it->second;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Missing 'network_interface' parameter");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (const auto it = info.hardware_parameters.find("domain"); it != info.hardware_parameters.end())
    {
      domain_id_ = std::stoi(it->second);
    }

    // 初始化 DDS 通信
    unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

    mcr_cmd_publisher_ = std::make_shared<unitree::robot::ChannelPublisher<moying::msg::dds_::MoyingMcrCmd_>>("moying/mcr_cmd");
    mcr_cmd_publisher_->InitChannel();

    mcr_state_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<moying::msg::dds_::MoyingState_>>("moying/state");

    mcr_state_subscriber_->InitChannel(
      static_cast<std::function<void(const void*)>>(
        [this](const void* msg_ptr)
        {
          const auto* msg = static_cast<const moying::msg::dds_::MoyingState_*>(msg_ptr);
          moying_state_ = *msg;
        }
      ),
      1);
    RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Joint list:");
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"), "  [%lu] %s", i, info_.joints[i].name.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"), "on_init finished.");
  return hardware_interface::CallbackReturn::SUCCESS;

}


std::vector<hardware_interface::StateInterface> MoyingMcrMujocoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"),"Exporting state interfaces...");
  
  size_t i = 0;
     // MCR 机械臂关节状态接口
  for ( i=0 ;i < 12; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &mcr_joint_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &mcr_joint_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &mcr_joint_efforts_[i]));
  }


   // MCR 底盘四轮状态接口（位置 + 速度）
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &forward_right_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &forward_right_velocity_state_);

  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &forward_left_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &forward_left_velocity_state_);

  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &back_right_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &back_right_velocity_state_);

  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &back_left_position_state_);
  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &back_left_velocity_state_);

  state_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_POSITION, &left_robotiq_joint_);
  state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &right_robotiq_joint_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MoyingMcrMujocoHardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"),"Command");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  size_t i = 0;
  for (i = 0; i < 12; ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &mcr_tau_commands_[i]));
  }

  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &forward_right_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &forward_left_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &back_right_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, hardware_interface::HW_IF_VELOCITY, &back_left_cmd_);

  command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &left_robotiq_joint_cmd_);
  command_interfaces.emplace_back(info_.joints[i++].name, "set_gripper_max_effort", &left_dummy_effort_limit_);

  command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &right_robotiq_joint_cmd_);
  command_interfaces.emplace_back(info_.joints[i].name, "set_gripper_max_effort", &right_dummy_effort_limit_);

  return command_interfaces;
}


hardware_interface::CallbackReturn MoyingMcrMujocoHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Activate");

  // 如果你确实要检查接口是否有效
  const auto interfaces = export_command_interfaces();
  for (size_t i = 0; i < interfaces.size(); ++i)
  {
    if (std::isnan(interfaces[i].get_value()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("MoyingMcrMujocoHardware"),
                   "Command interface value for joint [%s] is NaN.",
                   info_.joints[i].name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoyingMcrMujocoHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Deactivated successfully.");
  return CallbackReturn::SUCCESS;

}

hardware_interface::return_type MoyingMcrMujocoHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MoyingMcrMujocoHardware"), "Reading state...");

  // 读取左侧机械臂关节状态
  for (int i = 0; i < 6; ++i)
  {
    mcr_joint_positions_[i] = moying_state_.mcrl_joint_position()[i];
    mcr_joint_velocities_[i] = moying_state_.mcrl_joint_velocity()[i];
    mcr_joint_efforts_[i] = moying_state_.mcrl_joint_torque()[i];
  }
  // 读取右侧机械臂关节状态
  for (int i = 0; i < 6; ++i)
  {
    mcr_joint_positions_[i+6] = moying_state_.mcrr_joint_position()[i];
    mcr_joint_velocities_[i+6] = moying_state_.mcrr_joint_velocity()[i];
    mcr_joint_efforts_[i+6] = moying_state_.mcrr_joint_torque()[i];
  }
  // 读取底盘轮子位置和速度状态
  forward_right_position_state_ = moying_state_.mcr_wheel_position()[0];
  forward_left_position_state_ = moying_state_.mcr_wheel_position()[1];
  back_right_position_state_ = moying_state_.mcr_wheel_position()[2];
  back_left_position_state_ = moying_state_.mcr_wheel_position()[3];

  forward_right_velocity_state_ = moying_state_.mcr_wheel_velocity()[0];
  forward_left_velocity_state_ = moying_state_.mcr_wheel_velocity()[1];
  back_right_velocity_state_ = moying_state_.mcr_wheel_velocity()[2];
  back_left_velocity_state_ = moying_state_.mcr_wheel_velocity()[3];
  
  left_robotiq_joint_ = moying_state_.mcr_robotiqleft_position();
  right_robotiq_joint_ = moying_state_.mcr_robotiqright_position();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoyingMcrMujocoHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  //chassis
   // 写入四个轮子的速度命令
  moying_cmd_.mcr_wheel_velocity()[0] = forward_right_cmd_;
  moying_cmd_.mcr_wheel_velocity()[1] = forward_left_cmd_;
  moying_cmd_.mcr_wheel_velocity()[2] = back_right_cmd_;
  moying_cmd_.mcr_wheel_velocity()[3] = back_left_cmd_;

  moying_cmd_.mcr_robotiqleft_position() = left_robotiq_joint_cmd_ * 0.85/2.69;
  moying_cmd_.mcr_robotiqright_position() = right_robotiq_joint_cmd_ * 0.85/2.69;

  // 写入机械臂 torque 命令（前6个关节）
  for (size_t i = 0; i < 6; ++i)
  {
    moying_cmd_.mcrl_joint_tau()[i] = mcr_tau_commands_[i];
  }
  for (size_t i = 0; i < 6; ++i)
  {
    moying_cmd_.mcrr_joint_tau()[i] = mcr_tau_commands_[i+6];
  }
  // 通过DDS发布命令消息
  if (mcr_cmd_publisher_)
  {
    mcr_cmd_publisher_->Write(moying_cmd_);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  moying_mcr_mujoco_hardware::MoyingMcrMujocoHardware, hardware_interface::SystemInterface)
