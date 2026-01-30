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

#include "velocity_interface_controller/velocity_interface_controller.hpp"
#include "velocity_interface_controller/gravity_compensation.h"
#include "velocity_interface_controller/KalmanFilter.h"
#include <moying_mcr_arm_dynamics_models/dynamics_models.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <pinocchio/fwd.hpp>  // 需要包含前向声明
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp> 
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>


#include "controller_interface/helpers.hpp"

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

using ControllerReferenceMsg = velocity_interface_controller::VelcittInterfaceController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace velocity_interface_controller
{
VelcittInterfaceController::VelcittInterfaceController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn VelcittInterfaceController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);
  try
  {
    param_listener_ = std::make_shared<velocity_interface_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  std::vector<double> elfin5_basic_param = {
            5.5834, 20.6174, 11.3804, -3.93229, 1.11347, 0.544532, -0.85155, -0.241093,
            4.68726, 0.202398, 6.71532, 10.4753, -0.0813052, 0.189768, -0.291738, 0.821854,
            -0.800556, -0.144557, 1.94197, 0.0169887, 4.39409, 4.80435, 0.608595, 0.156044,
            -0.0732623, 0.0953026, -0.462754, 0.0115155, -0.124098, 0.680114, 5.51965, 3.97381,
            -0.456986, 0.0365659, -0.163843, -0.0654413, 0.0572246, 0.0263394, 0.191243, 0.751408,
            3.63408, 2.46535, -0.134029, 0.0722608, -0.00596429, -0.125216, -0.0023432, -0.0204092,
            -1.62148e-05, 0.344182, 3.56147, 3.12429};
  std::vector<double> elfin3_basic_param = {
            -9.5323, -0.494575, 0.217726, 10.8791, 6.97552, 11.1947, -2.01641,
            1.54969, 1.3911, 1.0394, 1.89893, 0.281033, 7.13797, 6.14408, 2.63808, 0.815542,
            2.21467, 1.38974, -0.3248, 0.0666458, 1.75996, 0.724542, 4.01041, 3.30275, 0.69838,
            -0.168122, -0.173828, 0.0744014, 0.408503, 0.00621546, -0.127075, 1.2218, 3.71925,
            2.70396, -0.172553, 0.0422848, -0.0134991, -0.180478, 0.238172, -0.0160892, 0.379865,
            0.820333, 2.89437, 2.21194, 0.152857, 0.0173616, -0.0890912, -0.0366125, 0.00348761,
            0.0175657, 0.00762579, 0.441349, 2.95611, 2.67612};
  elfin3_basic_param_ = elfin3_basic_param;
  elfin5_basic_param_ = elfin5_basic_param;
  effort_limit.resize(6);
  last_error.resize(6);
  impedance_tau_ = Eigen::VectorXd::Zero(6);
  kp_cart_ = {50.0, 50.0, 1000.0, 5.0, 5.0, 5.0};  
  imp_limit_ = 0.08;
  load_mass_kg_ = 0.0;
  for(auto& lp:last_error) {lp=0.0;}

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelcittInterfaceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  
  F_world = Eigen::VectorXd::Zero(6);
  for(auto &af:F_world)
    af=0.0;

  if (params_.effort_limit.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "effort_limit 长度错误，应为 6，当前为 %ld", params_.effort_limit.size());
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < 6; ++i) {
    effort_limit[i] = params_.effort_limit[i];
    impedance_tau_[i] = effort_limit[i];
  }
   
  std::string lib_path =  ament_index_cpp::get_package_share_directory(params_.urdf_package);
  std::string urdf_filename = lib_path + std::string(params_.urdf_path);
  // pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model_);
  // pinocchio::Data data(model);
  data_ = pinocchio::Data (model_);
  std::string sensor_frame_name = params_.end_link_name;  // 替换为实际传感器框架名称
  RCLCPP_ERROR(get_node()->get_logger(), "sensor_frame_name: %s", sensor_frame_name.c_str());
  sensor_frame_id_ = model_.getFrameId(sensor_frame_name);

  RCLCPP_ERROR(get_node()->get_logger(), "model_.frames.size(): %d", model_.frames.size());

  if (sensor_frame_id_ >= model_.frames.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid frame ID: %d", sensor_frame_id_);
    return CallbackReturn::FAILURE;
  }
  gripper_mass_ = params_.gripper_mass;
  coulomb_scale_ = params_.coulomb_scale; //库伦摩擦补偿系数
  kp_cart_ = params_.kp_cart;
  imp_limit_ = params_.imp_limit;
  F_world(2) = gripper_mass_ * 9.81; 
  if(coulomb_scale_.size()!=6)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Coulomb_scale size is not 6!");
    return CallbackReturn::FAILURE;
  }

  ft_init_done_ = false;
  ft_init_count_ = 0;
  ft_sensor_offset_.resize(6);
  for(auto& data:ft_sensor_offset_)
    data = 0.0;
  total_ft_init_count_ = params_.total_ft_init_count;
  ft_data.resize(6);
  for(auto& data: ft_data)
    data = 0.0;

  RCLCPP_INFO(get_node()->get_logger(),"##############################");
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] URDF : %s",urdf_filename.c_str());
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] End Link Name: %s",sensor_frame_name.c_str());
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] End Link ID: %d",sensor_frame_id_);
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] imp_limit: %0.3f",imp_limit_);
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] Gripper Mass: %f",gripper_mass_);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),"[Velocity Interface] Coulomb scale:"<<coulomb_scale_[0]<<" "
    <<coulomb_scale_[1]<<" "
    <<coulomb_scale_[2]<<" "
    <<coulomb_scale_[3]<<" "
    <<coulomb_scale_[4]<<" "
    <<coulomb_scale_[5]);
  RCLCPP_INFO(get_node()->get_logger(), "effort_limit 加载成功: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    effort_limit[0], effort_limit[1], effort_limit[2],
    effort_limit[3], effort_limit[4], effort_limit[5]);
  RCLCPP_INFO(get_node()->get_logger(), "kp_cart_ 加载成功: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    kp_cart_[0], kp_cart_[1], kp_cart_[2],
    kp_cart_[3], kp_cart_[4], kp_cart_[5]);
  RCLCPP_INFO(get_node()->get_logger(),"##############################");

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();
  ka_publisher_ = get_node()->create_publisher<ControllerStateMsg>("~/state_ka",10);

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&VelcittInterfaceController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
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

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  // state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration VelcittInterfaceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  if(params_.use_dynamics_models == true){
    for (const auto & joint : params_.joints)
    {
      command_interfaces_config.names.push_back(joint + "/" + "effort");
    }
  }
  else
  {
    for (const auto & joint : params_.joints)
    {
      command_interfaces_config.names.push_back(joint + "/" + "position");
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration VelcittInterfaceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + "velocity");
  }
  for (const auto & joint : params_.joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + "position");
  }

  return state_interfaces_config;
}

void VelcittInterfaceController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
}

std::vector<hardware_interface::CommandInterface> VelcittInterfaceController::on_export_reference_interfaces()//控制器管理器注册“参考命令接口”
{
  reference_interfaces_.resize(params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  //reference_interfaces_ 是你控制器内存储目标速度的容器
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());
  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), params_.joints[i] + "/" + "velocity",
      &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool VelcittInterfaceController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn VelcittInterfaceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), state_joints_);
  for(auto & ref : reference_interfaces_)
  {
    ref = 0.0;
  }
  std::vector<double> jnt_vel = {state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value(), state_interfaces_[4].get_value(), state_interfaces_[5].get_value()};
  std::vector<double> jnt_pos = {state_interfaces_[6].get_value(), state_interfaces_[7].get_value(), state_interfaces_[8].get_value(), state_interfaces_[9].get_value(), state_interfaces_[10].get_value(), state_interfaces_[11].get_value()};
  std::vector<double> zero_value(6,0.0),out_value;
  RCLCPP_INFO(get_node()->get_logger(),"Chain Controller Activated");
  grav_compen(jnt_pos,jnt_vel,zero_value,out_value);
  for(size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(out_value[i]);
  }
  if(params_.use_dynamics_models == true){
    RCLCPP_INFO(get_node()->get_logger(),"[Dynamics Models] using...");
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(),"[Dynamics Models] not using");
  }

  if(params_.device_name == "elfin5")
  {
    elfin_param_ = elfin5_basic_param_;
    dyn_model_ = std::make_shared<baichuan::arm::elfin::Elfin5Pose1DynamicsModel>();
    dyn_model_->init(elfin_param_);

    base_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrpn/mor/pose", 
      rclcpp::QoS(10),
      std::bind(&VelcittInterfaceController::basePoseCallback, this, std::placeholders::_1)
    );
    arm_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mor/arm_goal_pose",   // 改为你实际使用的目标话题名
      rclcpp::QoS(10),
      std::bind(&VelcittInterfaceController::goalPoseCallback, this, std::placeholders::_1)
    );
    load_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "/mor/arm_contact_load", 10,
      std::bind(&VelcittInterfaceController::loadCallback, this, std::placeholders::_1));
  }
  else if(params_.device_name == "elfin3")
  {
    elfin_param_ = elfin3_basic_param_;
    dyn_model_ =  std::make_shared<baichuan::arm::elfin::Elfin3Pose2DynamicsModel>();
    dyn_model_->init(elfin_param_);
     base_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vrpn/mcr/pose", 
      rclcpp::QoS(10),
      std::bind(&VelcittInterfaceController::basePoseCallback, this, std::placeholders::_1)
    );
    std::string end_link = params_.end_link_name;

    if (end_link.find("left") != std::string::npos)
    {
      arm_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mcr/left_goal_pose",   // 改为你实际使用的目标话题名
        rclcpp::QoS(10),
        std::bind(&VelcittInterfaceController::goalPoseCallback, this, std::placeholders::_1)
      );
      load_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        "/mcr/left_contact_load", 10,
        std::bind(&VelcittInterfaceController::loadCallback, this, std::placeholders::_1));
    }
    else if (end_link.find("right") != std::string::npos)
    {
      arm_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mcr/right_goal_pose",   // 改为你实际使用的目标话题名
        rclcpp::QoS(10),
        std::bind(&VelcittInterfaceController::goalPoseCallback, this, std::placeholders::_1)
      );
      load_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        "/mcr/right_contact_load", 10,
        std::bind(&VelcittInterfaceController::loadCallback, this, std::placeholders::_1));
    }
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(),"[Set up error] Device name is not correct");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(),"[Device name] Device name is %s",params_.device_name.c_str());
  if(params_.use_dynamics_models == true){
    std::vector<double> joint_position = {state_interfaces_[6].get_value(), state_interfaces_[7].get_value(), state_interfaces_[8].get_value(), state_interfaces_[9].get_value(), state_interfaces_[10].get_value(), state_interfaces_[11].get_value()};
    std::vector<double> joint_velocity = {state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value(), state_interfaces_[4].get_value(), state_interfaces_[5].get_value()};
    std::vector<double> joint_acceleration(6,0.0);
    Eigen::VectorXd gravity =  dyn_model_->gravityTerm(joint_position);

    for(size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(gravity(i));
    }
  }
  i_part_.resize(command_interfaces_.size());
  for(auto & i_a_part:i_part_)
  {
    i_a_part = 0.0;
  }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelcittInterfaceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  std::vector<double> jnt_vel = {state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value(), state_interfaces_[4].get_value(), state_interfaces_[5].get_value()};
  std::vector<double> jnt_pos = {state_interfaces_[6].get_value(), state_interfaces_[7].get_value(), state_interfaces_[8].get_value(), state_interfaces_[9].get_value(), state_interfaces_[10].get_value(), state_interfaces_[11].get_value()};
  std::vector<double> zero_value(6,0.0),out_value;
  RCLCPP_INFO(get_node()->get_logger(),"Chain Controller Deactivated");
  grav_compen(jnt_pos,jnt_vel,zero_value,out_value);
  for(size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(out_value[i]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VelcittInterfaceController::update_reference_from_subscribers()
{
  auto current_ref = input_ref_.readFromRT();

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      reference_interfaces_[i] = (*current_ref)->displacements[i];

      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type VelcittInterfaceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  //update params
  if(params_.use_dynamics_models == false){
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }
    for(size_t i=0;i<reference_interfaces_.size();i++)
    { 
      double cmd = reference_interfaces_[i];
      if(cmd < -0.2)
        cmd = -0.2;
      else if(cmd > 0.2)
        cmd = 0.2;
      command_interfaces_[i].set_value(state_interfaces_[6+i].get_value()+cmd*period.seconds());
    }
  }
  else
  {
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }
    // === 构造笛卡尔刚度矩阵 Kp（对角形式） ===
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 6; ++i)
      Kp(i, i) = kp_cart_[i];
    std::vector<double> joint_position = {state_interfaces_[6].get_value(), state_interfaces_[7].get_value(), state_interfaces_[8].get_value(), state_interfaces_[9].get_value(), state_interfaces_[10].get_value(), state_interfaces_[11].get_value()};
    std::vector<double> joint_velocity = {state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value(), state_interfaces_[4].get_value(), state_interfaces_[5].get_value()};
    std::vector<double> joint_acceleration(6,0.0);
    std::vector<double> pd_value(command_interfaces_.size(), 0.0);
    std::vector<double> kp = {params_.elfin1.kp, params_.elfin2.kp, params_.elfin3.kp, params_.elfin4.kp, params_.elfin5.kp, params_.elfin6.kp};
    std::vector<double> kd = {params_.elfin1.kd, params_.elfin2.kd, params_.elfin3.kd, params_.elfin4.kd, params_.elfin5.kd, params_.elfin6.kd};
    std::vector<double> ki = {params_.elfin1.ki, params_.elfin2.ki, params_.elfin3.ki, params_.elfin4.ki, params_.elfin5.ki, params_.elfin6.ki};
    std::vector<double> ki_max = {params_.elfin1.ki_max, params_.elfin2.ki_max, params_.elfin3.ki_max, params_.elfin4.ki_max, params_.elfin5.ki_max, params_.elfin6.ki_max};
    Eigen::VectorXd q = pinocchio::neutral(model_);  // 机械臂的中立位置
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
    // RCLCPP_INFO(get_node()->get_logger(),"%d,%d",q.size(),model_.nv);
    q << state_interfaces_[6].get_value(), state_interfaces_[7].get_value(), state_interfaces_[8].get_value(), state_interfaces_[9].get_value(), state_interfaces_[10].get_value(), state_interfaces_[11].get_value();

    Eigen::MatrixXd J(6, model_.nv);  // 初始化雅可比矩阵
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeFrameJacobian(model_, data_, q, sensor_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    Eigen::VectorXd gravity_torque = data_.g;
    F_world(2) = (gripper_mass_ + load_mass_kg_) * 9.81; 
    // if (load_mass_kg_ > 1) RCLCPP_ERROR(get_node()->get_logger(),"get load_mass_kg_ = %f",load_mass_kg_);
    Eigen::VectorXd tau_gripper = J.transpose() * F_world;

    for(size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      double cmd = reference_interfaces_[i];
      double state = joint_velocity[i];
      double error = cmd - state;
       // === P 部分 ===
      double p_part = kp[i] * error;
      // === I 部分 ===
      i_part_[i] += ki[i] * error * period.seconds();
      // === D 部分 ===
      double d_part = kd[i] * (error - last_error[i]) / period.seconds();
      // 抗积分饱和
      if(i_part_[i] >= ki_max[i]) {i_part_[i] = ki_max[i];}
      if(i_part_[i] <= -ki_max[i]) {i_part_[i] = -ki_max[i];}
      last_error[i] = error;
      pd_value[i] = p_part + d_part + i_part_[i];
              // === 异常力矩输出检测 ===
      
      if (std::abs(pd_value[i]) > effort_limit[i])
      {
        RCLCPP_WARN(get_node()->get_logger(),
          "⚠️ Joint %lu PID输出异常: %.3f 超过 30%% 力矩限制 %.3f\n"
          "   cmd=%.3f  state=%.3f  error=%.3f\n"
          "   P=%.3f  I=%.3f  D=%.3f",
          i, pd_value[i], effort_limit[i],
          cmd, state, error,
          p_part, i_part_[i], d_part);
      }
    }
    if (base_pose_received_ && goal_pose_received_)
    {
      pinocchio::SE3 T_world_ee = computeEndEffectorWorldPose(q);
      // 位置误差 = 目标 - 当前
      Eigen::Vector3d pos_err = arm_goal_pose_.translation() - T_world_ee.translation();
       // 姿态误差 = 目标旋转 * 当前旋转的逆
      Eigen::Matrix3d R_err = arm_goal_pose_.rotation() * T_world_ee.rotation().transpose();
      Eigen::AngleAxisd aa_err(R_err);
      Eigen::Vector3d ori_err = aa_err.angle() * aa_err.axis();  // 旋转向量

      Eigen::Matrix<double, 6, 1> dx;
      dx.head<3>() = pos_err;
      dx.tail<3>() = ori_err;                                    //组合误差
      Eigen::Matrix<double, 6, 1> F_ee = Kp * dx; 
      // === 关节空间力矩 ===
      impedance_tau_ = J.transpose() * F_ee; 
    }

    Eigen::VectorXd gravity =  dyn_model_->gravityTerm(joint_position);

    std::vector<double> filtered_velocity;
    double threshold = 0.01;
    for (const auto& v : joint_velocity) {
        if (std::abs(v) < threshold)
            filtered_velocity.push_back(0.0);
        else
            filtered_velocity.push_back(v);
    }
    // Eigen::VectorXd frictionTerm =  dyn_model_->frictionTerm(joint_position,filtered_velocity);//根据指令进行力矩补偿
    Eigen::VectorXd frictionTerm =  dyn_model_->FrictionCompensationwithstart(joint_position,filtered_velocity,reference_interfaces_,1.1);//根据指令进行力矩补偿
    

    for(size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      threshold = effort_limit[i] * imp_limit_;
      double pid_out = pd_value[i];
      double imp_tau = impedance_tau_[i];
      // double cmd_effort = gravity(i) + tau_gripper[i];          //实物重力补偿
      double cmd_effort = gravity_torque(i) + tau_gripper[i] ;    //仿真重力补偿
      if (std::abs(imp_tau) <= threshold)
      {
        // 误差很小的情况下，直接采用pid输出
        cmd_effort += pd_value[i] + coulomb_scale_[i] * frictionTerm[i];
      }
      else
      {
        if (std::abs(pid_out) > std::abs(imp_tau))
        {
          if(params_.device_name == "elfin5"&& (i == 1 || i == 2))
          {
            RCLCPP_INFO(get_node()->get_logger(),"jointid = %d ,pid_out = %f,imp_tau = %f",i+1 ,pid_out,imp_tau);
          }
          if (pid_out > std::abs(imp_tau)) pd_value[i] = std::abs(imp_tau);
          else if (pid_out < -1.0 * std::abs(imp_tau)) pd_value[i] = -1.0 * std::abs(imp_tau);          
        }
        cmd_effort += pd_value[i] + coulomb_scale_[i] * frictionTerm[i];
      }
      //limit max effort value
      if(cmd_effort > effort_limit[i])
        cmd_effort = effort_limit[i];
      else if(cmd_effort < -1.0*effort_limit[i])
        cmd_effort = -effort_limit[i];
      command_interfaces_[i].set_value(cmd_effort);
    }
  }
  return controller_interface::return_type::OK;
}


pinocchio::SE3 VelcittInterfaceController::computeEndEffectorWorldPose(const Eigen::VectorXd& q)
{
  // 1. 正向运动学
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  // 2. 获取 base_link → ee 的变换
  pinocchio::SE3 T_base_ee = data_.oMf[sensor_frame_id_];
  // 3. 乘以 base 在世界的变换，得到 world → ee
  pinocchio::SE3 T_world_ee = T_world_base_ * T_base_ee;
  return T_world_ee;
}
void VelcittInterfaceController::basePoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 提取平移
  Eigen::Vector3d trans(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);

  // 提取四元数（注意顺序：w, x, y, z）
  Eigen::Quaterniond quat(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z);

  // 构造 pinocchio::SE3
  T_world_base_ = pinocchio::SE3(quat.toRotationMatrix(), trans);
  base_pose_received_ = true;
}
void VelcittInterfaceController::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Eigen::Vector3d trans(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);

  Eigen::Quaterniond quat(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z);

  arm_goal_pose_ = pinocchio::SE3(quat.toRotationMatrix(), trans);
  goal_pose_received_ = true;
}
void VelcittInterfaceController::loadCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  load_mass_kg_ = msg->data;
  // RCLCPP_INFO(get_node()->get_logger(),"load_mass_kg_ = %f",load_mass_kg_);
}
}  // namespace velocity_interface_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_interface_controller::VelcittInterfaceController, controller_interface::ChainableControllerInterface)
