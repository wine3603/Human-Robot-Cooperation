// Copyright (c) 2024, 
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>
#include <string>

#include "moying_mcr_hardware/arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <yaml-cpp/yaml.h>
#include <elfin_ethercat_driver_plant/elfin_ethercat_manager.h>
#include <moying_mcr_hardware/arm_ethercat_driver.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace moying_mcr_hardware
{
hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  plugin_name_ = info_.name;

  std::string config_path,config_file;
  if (auto it = info_.hardware_parameters.find("config_path"); it != info_.hardware_parameters.end())
  {
    config_path = it->second;
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger(plugin_name_),"no setting [config_path] in URDF ROS2 Control");
    return CallbackReturn::ERROR;
  }
  if (auto it = info_.hardware_parameters.find("config_file"); it != info_.hardware_parameters.end())
  {
    config_file= it->second;
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger(plugin_name_),"no setting [config_file] in URDF ROS2 Control");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"######################Setting Info########################");
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"[Arm Driver File]: %s\/%s",config_path.c_str(),config_file.c_str());

  std::string configpath=ament_index_cpp::get_package_share_directory(config_path);
  std::string arm_driver_config_file=configpath+config_file;

  YAML::Node arm_config = YAML::LoadFile(arm_driver_config_file);
  if(arm_config["if_name"]){
    RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Setting [ifname] :'%s' ",(arm_config["if_name"].as<std::string>()).c_str());
    ecat_manager_ = std::make_shared<elfin_ethercat_driver_plant::EtherCatManager>(arm_config["if_name"].as<std::string>());
  }
  else{
    RCLCPP_ERROR(rclcpp::get_logger(plugin_name_),"no setting [ifname] in Driver in %s%s",config_path.c_str(),config_file.c_str());
    return CallbackReturn::ERROR;
  }


  rclcpp::Node::SharedPtr arm_ethercat_node = rclcpp::Node::make_shared(plugin_name_+"arm_ethercat_node");
  arm_handle_ = std::make_shared<moying_mcr_hardware::ArmEthercatDriver>(ecat_manager_,arm_ethercat_node,arm_config);

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"#########################################################");
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Init Finish");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"###############################################");
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"State Interfaces:");
  for(auto& ah:arm_handle_->module_infos)
  {
    RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"state:%s",ah.axis1.name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(ah.axis1.name, hardware_interface::HW_IF_POSITION, &ah.axis1.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ah.axis2.name, hardware_interface::HW_IF_POSITION, &ah.axis2.position));

    state_interfaces.emplace_back(hardware_interface::StateInterface(ah.axis1.name, hardware_interface::HW_IF_VELOCITY, &ah.axis1.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ah.axis2.name, hardware_interface::HW_IF_VELOCITY, &ah.axis2.velocity));

    state_interfaces.emplace_back(hardware_interface::StateInterface(ah.axis1.name, hardware_interface::HW_IF_EFFORT, &ah.axis1.effort));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ah.axis2.name, hardware_interface::HW_IF_EFFORT, &ah.axis2.effort));
  }
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"###############################################");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"###############################################");
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Command Interfaces:");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for(auto& ah:arm_handle_->module_infos)
  {
    RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"command interface:%s",ah.axis1.name.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      ah.axis1.name, hardware_interface::HW_IF_POSITION, &ah.axis1.position_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      ah.axis2.name, hardware_interface::HW_IF_POSITION, &ah.axis2.position_cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      ah.axis1.name, hardware_interface::HW_IF_VELOCITY, &ah.axis1.velocity_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      ah.axis2.name, hardware_interface::HW_IF_VELOCITY, &ah.axis2.velocity_cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      ah.axis1.name, hardware_interface::HW_IF_EFFORT, &ah.axis1.effort_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      ah.axis2.name, hardware_interface::HW_IF_EFFORT, &ah.axis2.effort_cmd));
  }
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"###############################################");

  return command_interfaces;
}
hardware_interface::return_type ArmHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  return arm_handle_->perform_command_mode_switch(start_interfaces,stop_interfaces);
}
hardware_interface::return_type ArmHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type iswell= arm_handle_->prepare_command_mode_switch(start_interfaces,stop_interfaces);
  return iswell;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Activating......");
  for(auto& ah:arm_handle_->module_infos)
  {
    int32_t pos_count1 = ah.client_ptr->getAxis1PosCnt();
    int16_t vel_count1 = ah.client_ptr->getAxis1VelCnt();
    int16_t trq_count1 = ah.client_ptr->getAxis1TrqCnt();
    int32_t pos_count_diff_1 = pos_count1 - ah.axis1.count_zero;

    double position_tmp1 = -1*pos_count_diff_1/ah.axis1.count_rad_factor;
    ah.axis1.position = position_tmp1;
    ah.axis1.velocity = -1*vel_count1/ah.axis1.count_rad_per_s_factor;
    ah.axis1.effort = -1*trq_count1/ah.axis1.count_Nm_factor;

    int32_t pos_count2 = ah.client_ptr->getAxis2PosCnt();
    int16_t vel_count2 = ah.client_ptr->getAxis2VelCnt();
    int16_t trq_count2 = ah.client_ptr->getAxis2TrqCnt();
    int32_t pos_count_diff_2 = pos_count2 - ah.axis2.count_zero;

    double position_tmp2 = -1*pos_count_diff_2/ah.axis2.count_rad_factor;
    ah.axis2.position = position_tmp2;
    ah.axis2.velocity = -1*vel_count2/ah.axis2.count_rad_per_s_factor;
    ah.axis2.effort = -1*trq_count2/ah.axis2.count_Nm_factor;
  }
  for(auto & ah:arm_handle_->module_infos)
  {
    ah.axis1.position_cmd = ah.axis1.position;
    ah.axis1.velocity_cmd = ah.axis1.velocity;
    ah.axis2.position_cmd = ah.axis2.position;
    ah.axis2.velocity_cmd = ah.axis2.velocity;
  }
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Activated");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Deactivating.......");
  for(size_t i=0;i<arm_handle_->ethercat_drivers_.size();i++)
  {
    if(arm_handle_->ethercat_drivers_[i]!=NULL)
    {
      delete arm_handle_->ethercat_drivers_[i];
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),"Deactivated");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(arm_handle_->n_);
  for(auto& ah:arm_handle_->module_infos)
  {
    int32_t pos_count1 = ah.client_ptr->getAxis1PosCnt();
    int16_t vel_count1 = ah.client_ptr->getAxis1VelCnt();
    int16_t trq_count1 = ah.client_ptr->getAxis1TrqCnt();
    int32_t pos_count_diff_1 = pos_count1 - ah.axis1.count_zero;

    double position_tmp1 = -1*pos_count_diff_1/ah.axis1.count_rad_factor;
    ah.axis1.position = position_tmp1;
    ah.axis1.velocity = -1*vel_count1/ah.axis1.count_rad_per_s_factor;
    ah.axis1.effort = -1*trq_count1/ah.axis1.count_Nm_factor;

    int32_t pos_count2 = ah.client_ptr->getAxis2PosCnt();
    int16_t vel_count2 = ah.client_ptr->getAxis2VelCnt();
    int16_t trq_count2 = ah.client_ptr->getAxis2TrqCnt();
    int32_t pos_count_diff_2 = pos_count2 - ah.axis2.count_zero;

    double position_tmp2 = -1*pos_count_diff_2/ah.axis2.count_rad_factor;
    ah.axis2.position = position_tmp2;
    ah.axis2.velocity = -1*vel_count2/ah.axis2.count_rad_per_s_factor;
    ah.axis2.effort = -1*trq_count2/ah.axis2.count_Nm_factor;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool all_salve_enable = true;
  for(unsigned int i=0;i<arm_handle_->ethercat_drivers_.size();i++)
  {
    all_salve_enable = all_salve_enable && arm_handle_->ethercat_drivers_[i]->getEnableState();
  }
  if(all_salve_enable)
  {
    int i =0;
    for(auto& ah:arm_handle_->module_infos)
    {

      if(!ah.client_ptr->inPosBasedMode())
      {
        ah.axis1.position_cmd = ah.axis1.position;
        ah.axis2.position_cmd = ah.axis2.position;
      }
      double position_cmd_count1 = -1*ah.axis1.position_cmd * ah.axis1.count_rad_factor + ah.axis1.count_zero;
      double position_cmd_count2 = -1*ah.axis2.position_cmd * ah.axis2.count_rad_factor + ah.axis2.count_zero;

      ah.client_ptr->setAxis1PosCnt(int32_t(position_cmd_count1));
      ah.client_ptr->setAxis2PosCnt(int32_t(position_cmd_count2));
  
      bool is_preparing_switch;
      boost::mutex::scoped_lock pre_switch_flags_lock(*((arm_handle_->pre_switch_mutex_ptrs_)[i]));
      is_preparing_switch = (arm_handle_->pre_switch_flags_)[i];
      pre_switch_flags_lock.unlock();

      if(!is_preparing_switch)
      {
        double vel_ff_cmd_count1 = -1 * ah.axis1.vel_ff_cmd * ah.axis1.count_rad_per_s_factor /16.0;
        double vel_ff_cmd_count2 = -1 * ah.axis2.vel_ff_cmd * ah.axis2.count_rad_per_s_factor /16.0;

        ah.client_ptr->setAxis1VelFFCnt(int16_t(vel_ff_cmd_count1));
        ah.client_ptr->setAxis2VelFFCnt(int16_t(vel_ff_cmd_count2));

        double torque_cmd_count1 = -1*(ah.axis1.effort_cmd) * ah.axis1.count_Nm_factor;
        double torque_cmd_count2 = -1*(ah.axis2.effort_cmd) * ah.axis2.count_Nm_factor;

        ah.client_ptr->setAxis1TrqCnt(int16_t(torque_cmd_count1));
        ah.client_ptr->setAxis2TrqCnt(int16_t(torque_cmd_count2));
      }
      i++;
    }
  }
  else
  {
    for(auto& ah:arm_handle_->module_infos)
    {
      ah.axis1.position_cmd = ah.axis1.position;
      ah.axis2.position_cmd = ah.axis2.position;

      ah.axis1.velocity_cmd = ah.axis1.velocity;
      ah.axis2.velocity_cmd = ah.axis2.velocity;

      ah.client_ptr->setAxis1VelFFCnt(0);
      ah.client_ptr->setAxis2VelFFCnt(0);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace moying_mcr_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moying_mcr_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
