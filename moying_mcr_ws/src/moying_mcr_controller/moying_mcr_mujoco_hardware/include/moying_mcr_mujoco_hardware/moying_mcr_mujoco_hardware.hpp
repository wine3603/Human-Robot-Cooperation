#ifndef MOYING_MCR_MUJOCO_HARDWARE__MOYING_MCR_MUJOCO_HARDWARE_HPP_
#define MOYING_MCR_MUJOCO_HARDWARE__MOYING_MCR_MUJOCO_HARDWARE_HPP_

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

#include "moying_mcr_mujoco_hardware/visibility_control.h"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/moying/MoyingState_.hpp>
#include <unitree/idl/moying/Moying_Mcr_Cmd_.hpp>

// using namespace elfin_hardware_interface;
namespace moying_mcr_mujoco_hardware
{
class MoyingMcrMujocoHardware : public  hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MoyingMcrMujocoHardware);

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MOYING_MCR_MUJOCO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  
  // Parameters for the chassis
  double forward_right_cmd_,forward_left_cmd_,back_right_cmd_,back_left_cmd_;
  double forward_right_velocity_state_,forward_left_velocity_state_,back_right_velocity_state_,back_left_velocity_state_;
  double forward_right_position_state_,forward_left_position_state_,back_right_position_state_,back_left_position_state_;
  double left_robotiq_joint_ , left_robotiq_joint_cmd_ ,left_dummy_effort_limit_;
  double right_robotiq_joint_ , right_robotiq_joint_cmd_ ,right_dummy_effort_limit_;
  // Store the command for the simulated robot
  std::vector<double> mcr_tau_commands_;
  std::vector<double> mcr_joint_positions_;
  std::vector<double> mcr_joint_velocities_;
  std::vector<double> mcr_joint_efforts_;


  moying::msg::dds_::MoyingMcrCmd_ moying_cmd_{};
  moying::msg::dds_::MoyingState_ moying_state_{};
   /*publisher*/
  unitree::robot::ChannelPublisherPtr<moying::msg::dds_::MoyingMcrCmd_> mcr_cmd_publisher_;
    /*subscriber*/
  unitree::robot::ChannelSubscriberPtr<moying::msg::dds_::MoyingState_> mcr_state_subscriber_;
 
  // Configuration
  std::string network_interface_;
  int domain_id_ = 0;

};

}  // namespace 

#endif  // MOYING_MCR_MUJOCO_HARDWARE__MOYING_MCR_MUJOCO_HARDWARE_HPP_
