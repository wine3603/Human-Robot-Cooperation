// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "pr2_wholebody_controller/pr2_wholebody_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <pr2_wholebody_interface/wholebody_interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
// #include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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

using ControllerReferenceMsg = pr2_wholebody_controller::PR2WBCController::ControllerReferenceMsg;

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

namespace pr2_wholebody_controller
{
PR2WBCController::PR2WBCController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PR2WBCController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<pr2_wholebody_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  read_tf_=false;
  std::string task_package_name = param_listener_->get_params().task_package_name;
  std::string urdf_package_name = param_listener_->get_params().urdf_package_name;
  std::string lib_package_name = param_listener_->get_params().lib_package_name;
  RCLCPP_INFO(get_node()->get_logger(), "task_package_name: %s", task_package_name.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "urdf_package_name: %s", urdf_package_name.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "lib_package_name: %s", lib_package_name.c_str());
  std::string task_path =  ament_index_cpp::get_package_share_directory(task_package_name);
  std::string urdf_path =  ament_index_cpp::get_package_share_directory(urdf_package_name);
  std::string lib_path =  ament_index_cpp::get_package_share_directory(lib_package_name);
  RCLCPP_INFO(get_node()->get_logger(), "task_path: %s", task_path.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "urdf_path: %s", urdf_path.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "lib_path: %s", lib_path.c_str());
  if(task_path.length() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get task package path");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(urdf_path.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get urdf package path");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(lib_path.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get lib package path");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string urdfFile;
  std::string taskFile;
  std::string libFold;

  urdfFile = urdf_path + param_listener_->get_params().urdf_file;
  taskFile = task_path + param_listener_->get_params().task_file;
  task_file_ = taskFile;
  libFold =  lib_path + param_listener_->get_params().lib_folder;
  robot_name_ = param_listener_->get_params().robot_name;

  cmd_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Twist>("/pr2/mecanum_controller/cmd_vel", 10);//底盘速度接口

  mmInterface_ = std::make_shared<pr2_wholebody_interface::PR2WBCInterface>(taskFile, libFold, urdfFile);
  mpc_node_ = rclcpp::Node::make_shared(
    robot_name_ + "_mpc",
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));
  mrt_node_ = rclcpp::Node::make_shared(
    robot_name_+ "_mrt",
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));
  target_node_ = rclcpp::Node::make_shared(
    robot_name_+ "_traget",
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));

  setupMpc();
  setupMrt();
  
  mcr_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vrpn/pr2/pose",  // 话题名称
    10,  // QoS 队列深度
    std::bind(&PR2WBCController::mcrposeCallback, this, std::placeholders::_1)
  );

  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // update_timer_ = get_node()->create_wall_timer(
  //       std::chrono::milliseconds(10),
  //       [this]() {
  //         this->tf_callback();
  //     });

  ref_publisher_ =  get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/mpc_cmd", 10);
  obs_publisher_ =  get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/mpc_obs", 10);
  std::cerr<<"Controller on_init finished!\n";
  return controller_interface::CallbackReturn::SUCCESS;
}

void PR2WBCController::mcrposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{

  read_tf_=true;

  x_ = msg->pose.position.x;    // 直接使用消息中的X坐标
  y_ = msg->pose.position.y;    // 直接使用消息中的Y坐标
  // double z = msg->pose.position.z;    // 如果需要Z坐标

  // ===== 姿态信息转换 =====
  tf2::Quaternion quat(
    msg->pose.orientation.x,   // 从消息中提取四元数
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );
  
  // double roll,pitch, yaw;

  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll_, pitch_, yaw_); // 转换为欧拉角（与原始代码保持一致）

  // ===== 调试输出 =====
  // RCLCPP_INFO(get_node()->get_logger(), "Position: (%.2f, %.2f), Yaw: %.2f rad", x_, y_, yaw_); 
}

void PR2WBCController::setupMpc()
{
  mpc_ = std::make_shared<ocs2::GaussNewtonDDP_MPC>(mmInterface_->mpcSettings(),mmInterface_->ddpSettings(),mmInterface_->getRollout(),
                                            mmInterface_->getOptimalControlProblem(),mmInterface_->getInitializer());
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robot_name_, mmInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(mpc_node_);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpcRosInterface_ = std::make_shared<ocs2::MPC_ROS_Interface>(*mpc_,robot_name_);
}
void PR2WBCController::setupMrt()
{
  mrtRosInterface_ = std::make_shared<ocs2::MRT_ROS_Interface>(robot_name_);
  mrtRosInterface_->initRollout(&(mmInterface_->getRollout()));
}

controller_interface::CallbackReturn PR2WBCController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{  
  params_ = param_listener_->get_params();

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

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&PR2WBCController::reference_callback, this, std::placeholders::_1));

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

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void PR2WBCController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
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

controller_interface::InterfaceConfiguration PR2WBCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());//预分配内存，params_.joints 是一个存储关节名称的列表
  for (const auto & joint : params_.joints)
  {
    if(params_.use_chain_interface == false)
      command_interfaces_config.names.push_back(joint + "/" + "velocity");
    else
      command_interfaces_config.names.push_back(params_.chain_velocity_interface_name+"/"+joint + "/" + "velocity");

  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PR2WBCController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + "position");
  }

  return state_interfaces_config;
}

ocs2::TargetTrajectories PR2WBCController::goalPoseToTargetTrajectories(const std::vector<Eigen::Vector3d>& position, const std::vector<Eigen::Quaterniond>& orientation,
                                                const ocs2::SystemObservation& observation)
{
  const ocs2::scalar_array_t timeTrajectory{currentObservation_.time};
  Eigen::Vector3d deskposleft(position[0].x(),position[0].y(),position[0].z());
  Eigen::Quaterniond deskorileft(orientation[0].w(),orientation[0].x(),orientation[0].y(),orientation[0].z());

  Eigen::Vector3d deskposright(position[1].x(),position[1].y(),position[1].z());
  Eigen::Quaterniond deskoriright(orientation[1].w(),orientation[1].x(),orientation[1].y(),orientation[1].z());

  Eigen::Vector3d basepos(position[2].x(),position[2].y(),position[2].z());
  Eigen::Quaterniond baseori(orientation[2].w(),orientation[2].x(),orientation[2].y(),orientation[2].z());


  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const ocs2::vector_t target = (ocs2::vector_t(21) << deskposleft, deskorileft.coeffs(),deskposright, deskoriright.coeffs(),basepos,baseori.coeffs()).finished();
  const ocs2::vector_array_t stateTrajectory{target};
  // input trajectory
  const ocs2::vector_array_t inputTrajectory{ocs2::vector_t::Zero(observation.input.size())};
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

controller_interface::CallbackReturn PR2WBCController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(),"on activate");

  RCLCPP_INFO(get_node()->get_logger(),"Using Joy target");
  target_node_->declare_parameter("topic_name",params_.joy_topic_name);
  auto bound_function = std::bind(&PR2WBCController::goalPoseToTargetTrajectories, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
  targetJoyPoseCommandInterface_= std::make_shared<pr2_wholebody_interface::PR2WBCJoyTargetTrajectories>(target_node_,robot_name_,bound_function,task_file_);
  targetThread_ = std::thread([&](){
      targetJoyPoseCommandInterface_->publishInteractiveMarker();
  });
  // }
  mpcThread_ = std::thread([&](){
    mpcRosInterface_->launchNodes(mpc_node_);
  });
  ocs2::setThreadPriority(1,mpcThread_);
  mrtRosInterface_->launchNodes(mrt_node_);
  ocs2::SystemObservation initObservation;
  ocs2::vector_t state(15);
  for(size_t i=0;i<15;i++){
    state(i)=0.0;
  }
  // std::cout<<"state初始化:"<<state.transpose()<<std::endl;
  currentObservation_.state = state;
  // std::cout<<"currentObservation_.state初始化:"<<currentObservation_.state.transpose()<<std::endl;
  // std::cout<<"command_interface的大小:"<<command_interfaces_.size()<<std::endl;
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PR2WBCController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  if (targetThread_.joinable()) {
    targetThread_.join();
  }
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
void PR2WBCController::updateStateEstimation(const rclcpp::Time time,const rclcpp::Duration &period)
{
  double roll,pitch,yaw;
  double posx,posy;

  double ox,oy,oz,ow=1.0;
  std::lock_guard<std::mutex> lock(transform_mutex_);
  posx = x_;
  posy = y_;
  yaw = yaw_;
  currentObservation_.time += period.seconds();
  ocs2::vector_t state(mmInterface_->getPR2WBCModuelInfo().stateDim);
  state(0) = posx;
  state(1) = posy;
  ocs2::scalar_t yawLast = currentObservation_.state(2);
  state(2) = yawLast + angles::shortest_angular_distance(yawLast, yaw) ;
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    state(i+3) = state_interfaces_[i].get_value();
  }
  currentObservation_.state = state;
}

controller_interface::return_type PR2WBCController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  static bool first_use=false;
  static bool mpc_node_well = false;
  
  if(read_tf_ == false)
  {
    // RCLCPP_INFO(get_node()->get_logger(),"read_tf:%d",read_tf_);
    return controller_interface::return_type::OK; 
  }
  updateStateEstimation(time,period);
  if(first_use == false)
  {
    ocs2::SystemObservation initObservation;
    initObservation = currentObservation_;

    ocs2::vector_t left_pos = ocs2::vector_t::Zero(3);
    ocs2::vector_t right_pos = ocs2::vector_t::Zero(3);
    ocs2::vector_t base_pos = ocs2::vector_t::Zero(3);
    ocs2::vector_t left_ori = ocs2::vector_t::Zero(4);
    ocs2::vector_t right_ori = ocs2::vector_t::Zero(4);
    ocs2::vector_t base_ori = ocs2::vector_t::Zero(4);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.left_position",left_pos);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.left_orientation", left_ori);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.right_position",right_pos);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.right_orientation", right_ori);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.base_position",base_pos);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.base_orientation", base_ori);
    //init pos
    Eigen::Vector3d left_init_pos(left_pos(0),left_pos(1),left_pos(2));
    Eigen::Vector3d right_init_pos(right_pos(0),right_pos(1),right_pos(2));
    Eigen::Vector3d base_init_pos(base_pos(0),base_pos(1),base_pos(2));

    // Eigen::Vector3d deskposleft(pos(0)-0.2,pos(1),pos(2));
    Eigen::Vector3d deskposleft = left_init_pos;
    Eigen::Quaterniond deskorileft(left_ori(0),left_ori(1),left_ori(2),left_ori(3));

    // Eigen::Vector3d deskposright(pos(0)+0.2,pos(1),pos(2));
    Eigen::Vector3d deskposright = right_init_pos;
    Eigen::Quaterniond deskoriright(right_ori(0),right_ori(1),right_ori(2),right_ori(3));

    Eigen::Vector3d basepos = base_init_pos;
    Eigen::Quaterniond baseori(base_ori(0),base_ori(1),base_ori(2),base_ori(3));

    ocs2::vector_t leftInitTarget(7);
    leftInitTarget.head(3) << deskposleft;
    leftInitTarget.tail(4) << deskorileft.coeffs();;
    ocs2::vector_t rightInitTarget(7);
    rightInitTarget.head(3) << deskposright ;
    rightInitTarget.tail(4) << deskoriright.coeffs();
    ocs2::vector_t baseInitTarget(7);
    baseInitTarget.head(3) << basepos;
    baseInitTarget.tail(4) << baseori.coeffs();
    ocs2::vector_t initTarget(21);

    initTarget << leftInitTarget, rightInitTarget,baseInitTarget;

    const ocs2::vector_t zeroInput =
        ocs2::vector_t::Zero(mmInterface_->getPR2WBCModuelInfo().inputDim);

    const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time},
                                                    {initTarget}, {zeroInput});
    mrtRosInterface_->resetMpcNode(initTargetTrajectories);
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0);
    }
    first_use=true;
    // return controller_interface::return_type::OK;
  }
  if(mpc_node_well==false){
    if(!mrtRosInterface_->initialPolicyReceived() && rclcpp::ok()) {
        mrtRosInterface_->spinMRT();
        mrtRosInterface_->setCurrentObservation(currentObservation_);
    }
    else{
      mpc_node_well = true;
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial MPC Done.");
    }
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0);
    }
    return controller_interface::return_type::OK;
  }
  mrtRosInterface_->setCurrentObservation(currentObservation_);
  mrtRosInterface_->spinMRT();
  mrtRosInterface_->updatePolicy();

  ocs2::vector_t optimizedState, optimizedInput;

  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  const ocs2::scalar_t dt = 1.0/mmInterface_->mpcSettings().mrtDesiredFrequency_;
  // mrtRosInterface_->rolloutPolicy(currentObservation_.time, currentObservation_.state,dt,optimizedState, optimizedInput, plannedMode);
  mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state,optimizedState, optimizedInput, plannedMode);
  currentObservation_.input = optimizedInput;

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(optimizedInput(i+3));
  }

  geometry_msgs::msg::Twist cmd_msg;
  cmd_msg.linear.x = optimizedInput(0);
  cmd_msg.linear.y = optimizedInput(1);
  cmd_msg.angular.z = optimizedInput(2);
  cmd_publisher_->publish(cmd_msg);

  return controller_interface::return_type::OK;
}

}  // namespace pr2_wholebody_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pr2_wholebody_controller::PR2WBCController, controller_interface::ControllerInterface)
