
#pragma once

#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "yaml-cpp/yaml.h"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "elfin_ethercat_driver_plant/elfin_ethercat_manager.h"
#include "elfin_ethercat_driver_plant/elfin_ethercat_client.h"
#include "elfin_ethercat_driver_plant/elfin_ethercat_driver.h"



namespace moying_mcr_hardware
{
typedef struct{
  std::string name;
  double reduction_ratio;
  double count_rad_factor;
  double count_rad_per_s_factor;
  double count_Nm_factor;
  int32_t count_zero;

  double axis_position_factor;
  double axis_torque_factor;

  double position;
  double velocity;
  double effort;

  double position_cmd;
  double velocity_cmd;
  double vel_ff_cmd;
  double effort_cmd;
  double gc_offset_cmd;

  double position_cmd_last;
}AxisInfo;

typedef struct{
  elfin_ethercat_driver_plant::ElfinEtherCATClient* client_ptr;
  AxisInfo axis1;
  AxisInfo axis2;
}ModuleInfo;


class ArmEthercatDriver
{
public:
  ArmEthercatDriver(std::shared_ptr<elfin_ethercat_driver_plant::EtherCatManager> ecat_manager,
    const rclcpp::Node::SharedPtr& node,YAML::Node& config);
  ~ArmEthercatDriver();
  bool setGroupTrqMode(const std::vector<int> &module_no);
  bool setGroupPosMode(const std::vector<int> &module_no);
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces);

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) ;
  std::vector<ModuleInfo> module_infos;
  rclcpp::Node::SharedPtr n_;
  rclcpp::Node::SharedPtr m_;
  double motion_threshold_ = 5e-5;

  bool pos_interface_claimed = false;
  bool pos_interface_running = false;
  bool first_pass_ = true;
  bool initialized_ = false;
  std::vector<std::string> elfin_driver_names_;
  elfin_ethercat_driver_plant::EtherCatManager* em;
  std::vector<elfin_ethercat_driver_plant::ElfinEtherCATDriver*> ethercat_drivers_;
  std::vector<bool> in_effort_mode_;


  std::vector<bool> pre_switch_flags_;
  std::vector<boost::shared_ptr<boost::mutex> > pre_switch_mutex_ptrs_;



};

}// namespace moying_mcr_hardware