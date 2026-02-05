// Copyright (c) 2024, JK
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

#include "pr2_mobile_controller/pr2_omni_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "controller_interface/helpers.hpp"
#include <functional>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = pr2_omni_controller::OmniController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace pr2_omni_controller
{
OmniController::OmniController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn OmniController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);
  input_ref_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>>();

  try
  {
    param_listener_ = std::make_shared<pr2_omni_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  velocity_command_subsciption_ = get_node()->create_subscription<Twist>("/pr2/mecanum_controller/cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const Twist::SharedPtr twist)
  {
    velocity_command_ptr_.writeFromNonRT(twist);
  });
  last_steer_angle_.assign(4, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.steer_joints.empty())
  {
    RCLCPP_FATAL(get_node()->get_logger(),"Parameter 'steer_joints' is empty. Controller cannot be configured.");
    return CallbackReturn::FAILURE;
  }
  if (params_.wheel_left_joints.empty())
  {
    RCLCPP_FATAL(get_node()->get_logger(),"Parameter 'wheel_left_joints' is empty. Controller cannot be configured.");
    return CallbackReturn::FAILURE;
  }
  if (params_.wheel_right_joints.empty())
  {
    RCLCPP_FATAL(get_node()->get_logger(),"Parameter 'wheel_right_joints' is empty. Controller cannot be configured.");
    return CallbackReturn::FAILURE;
  }

  if (params_.steer_joints.size() != params_.wheel_left_joints.size() ||
      params_.steer_joints.size() != params_.wheel_right_joints.size())
  {
    RCLCPP_FATAL(get_node()->get_logger(),
      "Size mismatch: steer_joints=%zu, wheel_left_joints=%zu, wheel_right_joints=%zu. "
      "They must have the same length.",
      params_.steer_joints.size(),
      params_.wheel_left_joints.size(),
      params_.wheel_right_joints.size());
    return CallbackReturn::FAILURE;
  }
  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&OmniController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();

  // 2) Reset ref msg using a consistent joint list (choose steer_joints as "module list")
  reset_controller_reference_msg(msg, params_.steer_joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  state_publisher_->lock();
  // 3) frame_id: do NOT use params_.joints[0] anymore
  state_publisher_->msg_.header.frame_id = params_.steer_joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void OmniController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.steer_joints.size() + params_.wheel_left_joints.size() + params_.wheel_right_joints.size()) {
    input_ref_.writeFromNonRT(msg);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu joints, but expected %zu joints in command (steer: %zu, wheel_left: %zu, wheel_right: %zu). Ignoring message.",
      msg->joint_names.size(),
      params_.steer_joints.size() + params_.wheel_left_joints.size() + params_.wheel_right_joints.size(),
      params_.steer_joints.size(),
      params_.wheel_left_joints.size(),
      params_.wheel_right_joints.size());
  }
}

controller_interface::InterfaceConfiguration OmniController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : params_.steer_joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  
  for (const auto & joint : params_.wheel_left_joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  for (const auto & joint : params_.wheel_right_joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration OmniController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : params_.steer_joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  for (const auto & joint : params_.wheel_left_joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  for (const auto & joint : params_.wheel_right_joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn OmniController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
   const size_t n = params_.steer_joints.size();
  if (n == 0 || params_.wheel_left_joints.size() != n || params_.wheel_right_joints.size() != n)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Invalid joint params: steer=%zu left=%zu right=%zu",
      params_.steer_joints.size(),
      params_.wheel_left_joints.size(),
      params_.wheel_right_joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 最简：直接把所有命令接口清零
  for (auto & itf : command_interfaces_)
  {
    itf.set_value(0.0);
  }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (auto & itf : command_interfaces_)
  {
    itf.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmniController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & itf : command_interfaces_)
  {
    if (itf.get_name().empty())  // Check if the interface name is valid (you may modify this logic)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface '%s' is invalid!", itf.get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
  }


  auto velocity_command = velocity_command_ptr_.readFromRT();
  if (!velocity_command || !(*velocity_command)) {
    return controller_interface::return_type::OK;
  }
  const size_t n = params_.steer_joints.size();
  if (n == 0) {
    return controller_interface::return_type::OK;
  }
  // === 1. Base velocity (SI units) ===
  auto clamp = [](double v, double lo, double hi) {
    return std::min(std::max(v, lo), hi);
  };

  const double min_speed = params_.min_steer_speed;  // (m/s)

  double vx = (*velocity_command)->linear.x;
  double vy = (*velocity_command)->linear.y;
  double wz = (*velocity_command)->angular.z;

  vx = clamp(vx, -params_.max_vx, params_.max_vx);
  vy = clamp(vy, -params_.max_vy, params_.max_vy);
  wz = clamp(wz, -params_.max_wz, params_.max_wz);

  double v_trans = std::hypot(vx, vy);
  if (v_trans < min_speed) {
    vx = 0.0;
    vy = 0.0;
    v_trans = 0.0;
  }
  if (std::abs(wz) < 0.005) {
    wz = 0.0;
  }
// === 2. Parameters ===
  const double r = 0.5 * params_.wheel_diameter;   // wheel radius (m)
  if (!(r > 0.0)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Invalid wheel radius (wheel_diameter=%f).", params_.wheel_diameter);
    return controller_interface::return_type::OK;
  }

  const double lx = params_.wheelbase_x;
  const double ly = params_.wheelbase_y;
  const double track = params_.wheel_track;  // (m) left-right distance within same module

  // === 3. Wheel group positions ===
  std::array<Eigen::Vector3d, 4> wheel_pos;
  wheel_pos[0] = Eigen::Vector3d( lx,  ly, 0.0);
  wheel_pos[1] = Eigen::Vector3d( lx, -ly, 0.0);
  wheel_pos[2] = Eigen::Vector3d(-lx,  ly, 0.0);
  wheel_pos[3] = Eigen::Vector3d(-lx, -ly, 0.0);

  const Eigen::Vector3d v_base(vx, vy, 0.0);
  const Eigen::Vector3d w_base(0.0, 0.0, wz);

  auto wrapToPi = [](double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;  // (-pi, pi]
  };
  for (size_t i = 0; i < n; ++i)
  {
    const Eigen::Vector3d v_i = v_base + w_base.cross(wheel_pos[i]);
    const double speed = v_i.head<2>().norm();

    // const double theta = std::atan2(v_i.y(), v_i.x());   // [-pi, pi]
    const double theta_meas = state_interfaces_[i].get_value();  // continuous, no wrap

    auto nearestEquivalent = [&](double theta_wrapped, double theta_ref_cont) {
      const double e = wrapToPi(theta_wrapped - theta_ref_cont); // [-pi, pi]
      return theta_ref_cont + e;                                  // continuous target
    };

    double steer_cmd = last_steer_angle_[i];
    bool flip_drive = false;

    if (v_trans == 0.0 && wz == 0.0)
    {
      command_interfaces_[i].set_value(theta_meas);
      command_interfaces_[n + i].set_value(0.0);
      command_interfaces_[2 * n + i].set_value(0.0);
      last_steer_angle_[i] = theta_meas;
    }
    else
    {
      double theta_des;
      if (v_trans == 0.0 && wz != 0.0)
      {
        // 纯旋转：切向方向（wz × wheel_pos[i]）
        const double vx_spin = -wz * wheel_pos[i].y();
        const double vy_spin =  wz * wheel_pos[i].x();
        theta_des = std::atan2(vy_spin, vx_spin);
      }
      else
      {
        // 正常 / 混合运动
        theta_des = std::atan2(v_i.y(), v_i.x());
      }
      const double thetaA_wrapped = wrapToPi(theta_des);
      const double thetaB_wrapped = wrapToPi(theta_des + M_PI);

      // 将两个候选角展开到当前实测角附近，形成连续目标
      const double thetaA_cont = nearestEquivalent(thetaA_wrapped, theta_meas);
      const double thetaB_cont = nearestEquivalent(thetaB_wrapped, theta_meas);

      // 用最短角差比较（误差wrap，状态不wrap）
      const double diffA = wrapToPi(thetaA_cont - theta_meas);
      const double diffB = wrapToPi(thetaB_cont - theta_meas);

      if (std::abs(diffB) < std::abs(diffA)) {
        steer_cmd  = thetaB_cont;
        flip_drive = true;
      } else {
        steer_cmd  = thetaA_cont;
        flip_drive = false;
      }

      last_steer_angle_[i] = steer_cmd;
    }

    const double delta_v = wz * (track * 0.5);

    const double dir = flip_drive ? -1.0 : 1.0;
    const double v_left  = dir * speed - delta_v;
    const double v_right = dir * speed + delta_v;

    const double omega_left  = v_left  / r;
    const double omega_right = v_right / r;

    command_interfaces_[i].set_value(steer_cmd);            // continuous steer position target
    command_interfaces_[n + i].set_value(omega_left);
    command_interfaces_[2 * n + i].set_value(omega_right);
  }
  // for (size_t i = 0; i < n; ++i)
  // {
  //     // v_i = v + w x r_i
  //   const Eigen::Vector3d v_i = v_base + w_base.cross(wheel_pos[i]);
  //   const double speed = v_i.head<2>().norm();  // center speed of module (m/s)

  //   // raw desired steering direction
  //   const double theta = std::atan2(v_i.y(), v_i.x());  // [-pi, pi]

  //   // measured current steering angle from state_interfaces_[0..3]
  //   const double theta_meas = wrapToPi(state_interfaces_[i].get_value());

  //   double steer_cmd = last_steer_angle_[i];  // default for low-speed hold
  //   bool flip_drive = false;

  //   if (speed < min_speed)
  //   {
  //     steer_cmd = last_steer_angle_[i];
  //     flip_drive = false;
  //   }
  //   else
  //   {
  //     const double thetaA = wrapToPi(theta);
  //     const double diffA  = wrapToPi(thetaA - theta_meas);

  //     // Candidate B: theta + pi (equivalent, requires reversing wheel speed)
  //     const double thetaB = wrapToPi(theta + M_PI);
  //     const double diffB  = wrapToPi(thetaB - theta_meas);
  //     if (std::abs(diffB) < std::abs(diffA)) {
  //       steer_cmd  = thetaB;
  //       flip_drive = true;
  //     } else {
  //       steer_cmd  = thetaA;
  //       flip_drive = false;
  //     }

  //     last_steer_angle_[i] = steer_cmd;
  //   }

  //   // 2) left/right wheel speed correction due to intra-module track
  //   // First-order correction using base yaw rate:
  //   const double delta_v = wz * (track * 0.5);  // m/s

  //   const double dir = flip_drive ? -1.0 : 1.0;

  //   const double v_left  = dir * speed - delta_v;   // m/s
  //   const double v_right = dir * speed + delta_v;   // m/s

  //   const double omega_left  = v_left  / r;     // rad/s
  //   const double omega_right = v_right / r;     // rad/s

  //   // 3) write commands (fixed index layout)
  //   command_interfaces_[i].set_value(steer_cmd);            // steer position
  //   command_interfaces_[n + i].set_value(omega_left);       // left wheel velocity
  //   command_interfaces_[2 * n + i].set_value(omega_right);  // right wheel velocity
  // }

  return controller_interface::return_type::OK;
}

}  // namespace pr2_omni_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pr2_omni_controller::OmniController, controller_interface::ControllerInterface)
