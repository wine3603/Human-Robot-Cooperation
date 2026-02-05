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

#ifndef PR2_WHOLEBODY_CONTROLLER__PR2_WHOLEBODY_CONTROLLER_HPP_
#define PR2_WHOLEBODY_CONTROLLER__PR2_WHOLEBODY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pr2_wholebody_controller_parameters.hpp"
#include "pr2_wholebody_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// #include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <pr2_wholebody_interface/wholebody_interface.h>
#include <pr2_wholebody_interface/command/target_trajectories_interactive_marker.h>
#include <pr2_wholebody_interface/command/joy_target_trajectories.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

namespace pr2_wholebody_controller
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

class PR2WBCController : public controller_interface::ControllerInterface
{
public:
  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  PR2WBCController();

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<pr2_wholebody_controller::ParamListener> param_listener_;
  pr2_wholebody_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mcr_pose_sub_;
  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  

private:
  void setupMrt();
  void setupMpc();
  // void tf_callback();
  void mcrposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void updateStateEstimation(const rclcpp::Time time,const rclcpp::Duration &period);
  ocs2::TargetTrajectories goalPoseToTargetTrajectories(const std::vector<Eigen::Vector3d>& position, const std::vector<Eigen::Quaterniond>& orientation,
                                                const ocs2::SystemObservation& observation);

  // callback for topic interface
  PR2_WHOLEBODY_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::mutex transform_mutex_;
  double x_, y_, yaw_;
  double roll_, pitch_;
  bool read_tf_;

  std::string robot_name_;
  std::string task_file_;
  std::shared_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  std::shared_ptr<pr2_wholebody_interface::PR2WBCInterface> mmInterface_;
  rclcpp::Node::SharedPtr mpc_node_;
  rclcpp::Node::SharedPtr mrt_node_;
  rclcpp::Node::SharedPtr target_node_;
  std::shared_ptr<ocs2::MPC_ROS_Interface> mpcRosInterface_;
  std::shared_ptr<ocs2::MRT_ROS_Interface> mrtRosInterface_;
  std::shared_ptr<pr2_wholebody_interface::PR2WBCTargetTrajectoriesInteractiveMarker> targetPoseCommandInterface_;
  std::shared_ptr<pr2_wholebody_interface::PR2WBCJoyTargetTrajectories> targetJoyPoseCommandInterface_;
  ocs2::vector_t optimizedInput_;
  //State Estimation
  ocs2::SystemObservation currentObservation_;
  std::thread mpcThread_;
  std::thread mrtThread_;
  std::thread targetThread_;
  std::thread mrtspinThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};


  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obs_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_publisher_;

};

}  // namespace pr2_wholebody_controller

#endif  // PR2_WHOLEBODY_CONTROLLER__PR2_WHOLEBODY_CONTROLLER_HPP_
