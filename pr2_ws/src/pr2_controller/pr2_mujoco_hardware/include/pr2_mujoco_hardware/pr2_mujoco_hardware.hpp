#ifndef PR2_MUJOCO_HARDWARE__PR2_MUJOCO_HARDWARE_HPP_
#define PR2_MUJOCO_HARDWARE__PR2_MUJOCO_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pr2_mujoco_hardware/visibility_control.h"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/pr2/pr2_state_.hpp>
#include <unitree/idl/pr2/pr2_cmd_.hpp>

// using namespace elfin_hardware_interface;
namespace pr2_mujoco_hardware
{
class Pr2MujocoHardware : public  hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pr2MujocoHardware);

  PR2_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  PR2_MUJOCO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PR2_MUJOCO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PR2_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PR2_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  
  // Parameters for the chassis
  double fl_rotation_cmd_,fr_rotation_cmd_,bl_rotation_cmd_,br_rotation_cmd_;          //舵角指令
  double fl_rotation_state_,fr_rotation_state_,bl_rotation_state_,br_rotation_state_;  //舵角状态
  double fl_l_cmd_,fl_r_cmd_,fr_l_cmd_,fr_r_cmd_,bl_l_cmd_,bl_r_cmd_,br_l_cmd_,br_r_cmd_;
  double fl_l_position_state_,fl_r_position_state_,fr_l_position_state_,fr_r_position_state_,bl_l_position_state_,bl_r_position_state_,br_l_position_state_,br_r_position_state_;
  double fl_l_velocity_state_,fl_r_velocity_state_,fr_l_velocity_state_,fr_r_velocity_state_,bl_l_velocity_state_,bl_r_velocity_state_,br_l_velocity_state_,br_r_velocity_state_;
  double left_robotiq_joint_ , left_robotiq_joint_cmd_ ;
  double right_robotiq_joint_ , right_robotiq_joint_cmd_ ;
  double head_pan_joint_ , head_pan_joint_cmd_;
  double head_tilt_joint_ , head_tilt_joint_cmd_ ;
  double laser_tilt_mount_joint_ , laser_tilt_mount_joint_cmd_ ;
  // Store the command for the simulated robot
  std::vector<double> pr2_tau_commands_;
  std::vector<double> pr2_joint_positions_;
  std::vector<double> pr2_joint_velocities_;


  pr2cmd::msg::dds_::PR2ActuatorCmd pr2_cmd_{};
  pr2cmd::msg::dds_::PR2SensorState pr2_state_{};
   /*publisher*/
  unitree::robot::ChannelPublisherPtr<pr2cmd::msg::dds_::PR2ActuatorCmd> pr2_cmd_publisher_;
    /*subscriber*/
  unitree::robot::ChannelSubscriberPtr<pr2cmd::msg::dds_::PR2SensorState> pr2_state_subscriber_;
 
  // Configuration
  std::string network_interface_;
  int domain_id_ = 0;

};

}  // namespace 

#endif  // PR2_MUJOCO_HARDWARE__PR2_MUJOCO_HARDWARE_HPP_
