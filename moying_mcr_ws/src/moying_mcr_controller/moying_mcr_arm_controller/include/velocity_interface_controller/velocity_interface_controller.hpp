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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef VELOCITY_INTERFACE_CONTROLLER__VELOCITY_INTERFACE_CONTROLLER_HPP_
#define VELOCITY_INTERFACE_CONTROLLER__VELOCITY_INTERFACE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "velocity_interface_controller_parameters.hpp"
#include "velocity_interface_controller/visibility_control.h"
#include "velocity_interface_controller/KalmanFilter.h"
#include <moying_mcr_arm_dynamics_models/dynamics_models.h>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include <pinocchio/fwd.hpp>  // 需要包含前向声明
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <std_msgs/msg/float64.hpp> 
#include <pinocchio/spatial/fwd.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace velocity_interface_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class VelcittInterfaceController : public controller_interface::ChainableControllerInterface
{
public:
  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  VelcittInterfaceController();

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = std_msgs::msg::Float64MultiArray;

protected:
  std::shared_ptr<velocity_interface_controller::ParamListener> param_listener_;
  velocity_interface_controller::Params params_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr load_sub_;
  
  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::vector<double> elfin5_basic_param_;
  std::vector<double> elfin3_basic_param_;
  std::vector<double> elfin_param_;

  std::vector<double> last_joint_velocity_;
  std::shared_ptr<baichuan::arm::BasicDynamicsModel> dyn_model_;
  bool on_set_chained_mode(bool chained_mode) override;
  std::shared_ptr<KalmanFilter> kf_;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr ka_publisher_;
  std::vector<double> i_part_;
  std::string urdf_file;
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex sensor_frame_id_;
  pinocchio::SE3 computeEndEffectorWorldPose(const Eigen::VectorXd& q);
  void basePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void loadCallback(const std_msgs::msg::Float64::SharedPtr msg);
  pinocchio::SE3 arm_goal_pose_;
  bool goal_pose_received_ = false;
  pinocchio::SE3 T_world_base_;
  bool base_pose_received_ = false;
  Eigen::VectorXd impedance_tau_;
  double gripper_mass_;
  std::vector<double> coulomb_scale_;
  //ft_sensor
  bool ft_init_done_;
  int ft_init_count_;
  std::vector<double> ft_sensor_offset_;
  int total_ft_init_count_;
  std::vector<double> ft_data;
  Eigen::VectorXd F_world;
  double desk_measure_;
  bool use_pid_;
  std::vector<double> effort_limit;
  std::vector<double> kp_cart_;  // Cartesian stiffness
  double imp_limit_;
  double load_mass_kg_;

private:
  // callback for topic interface
  VELOCITY_INTERFACE_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  std::vector<double> last_error;
};

}  // namespace velocity_interface_controller

#endif  // VELOCITY_INTERFACE_CONTROLLER__VELOCITY_INTERFACE_CONTROLLER_HPP_
