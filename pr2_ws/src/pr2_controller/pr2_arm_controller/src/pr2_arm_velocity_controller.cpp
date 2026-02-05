#include "pr2_arm_controller/pr2_arm_velocity_controller.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

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

namespace pr2_arm_velocity_controller
{

Pr2armVelocityController::Pr2armVelocityController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn Pr2armVelocityController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<pr2_arm_velocity_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller init: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // DoF & containers are sized in on_configure()
  dof_ = 15;  // Manually set the DoF to 15
  left_contact_load_ = 0.0;  // 左侧负载初始化
  right_contact_load_ = 0.0;  // 右侧负载初始化
  i_part_.resize(dof_, 0.0);
  last_error_.resize(dof_, 0.0);
  effort_limit_.resize(dof_, 0.0);

  // 初始化负载信息的订阅器
  load_sub_left_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "/mcr/left_contact_load", 10, std::bind(&Pr2armVelocityController::loadCallbackLeft, this, std::placeholders::_1));

  load_sub_right_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "/mcr/right_contact_load", 10, std::bind(&Pr2armVelocityController::loadCallbackRight, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Pr2armVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    params_ = param_listener_->get_params();

  // 1) DoF and end_link_names from joints list
  dof_ = params_.joints.size();
  if (dof_ == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "params.joints is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (dof_ != 15)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Expected 15 DoF, got %zu (continue anyway).", dof_);
  }



  // 2) Validate PID vectors
  if (params_.kp.size() != dof_ ||
      params_.ki.size() != dof_ ||
      params_.kd.size() != dof_ ||
      params_.ki_max.size() != dof_)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "PID parameter size mismatch. Expect %zu for kp/ki/kd/ki_max.", dof_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // 3) Torque limits
  if (params_.effort_limit.size() != dof_)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "effort_limit size mismatch: expect %zu, got %zu",
      dof_, params_.effort_limit.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  effort_limit_ = params_.effort_limit;

  // 4) State joints list consistency
  state_joints_ = params_.state_joints.empty() ? params_.joints : params_.state_joints;
  if (state_joints_.size() != dof_)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "state_joints size mismatch: expect %zu, got %zu",
      dof_, state_joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 5) Allocate controller internal states
  i_part_.resize(dof_, 0.0);
  last_error_.resize(dof_, 0.0);

  // 6) Build Pinocchio model (15DoF sub-URDF)
  const std::string pkg_path = ament_index_cpp::get_package_share_directory(params_.urdf_package);
  const std::string urdf_file = pkg_path + params_.urdf_path;

  try
  {
    pinocchio::urdf::buildModel(urdf_file, model_);
    data_ = pinocchio::Data(model_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build Pinocchio model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (static_cast<size_t>(model_.nv) != dof_)
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "Pinocchio model nv=%d differs from dof=%zu. Make sure your URDF is exactly the 15DoF sub-model.",
      model_.nv, dof_);
  }
    
  std::string left_end_link_name = params_.left_end_link_name; 
  RCLCPP_ERROR(get_node()->get_logger(), "left_end_link_name: %s", left_end_link_name.c_str());
  left_end_frame_id_ = model_.getFrameId(left_end_link_name);

  RCLCPP_ERROR(get_node()->get_logger(), "model_.frames.size(): %d", model_.frames.size());

  if (left_end_frame_id_ >= model_.frames.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid frame ID: %d", left_end_frame_id_);
    return CallbackReturn::FAILURE;
  }
  
  std::string right_end_link_name = params_.right_end_link_name; 
  RCLCPP_ERROR(get_node()->get_logger(), "right_end_link_name: %s", right_end_link_name.c_str());
  right_end_frame_id_ = model_.getFrameId(right_end_link_name);

  RCLCPP_ERROR(get_node()->get_logger(), "model_.frames.size(): %d", model_.frames.size());

  if (right_end_frame_id_ >= model_.frames.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid frame ID: %d", right_end_frame_id_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(), "==============================");
  RCLCPP_INFO(get_node()->get_logger(), "  [PR2] Configure OK  ");
  RCLCPP_INFO(get_node()->get_logger(), "  DoF   : %zu", dof_);
  RCLCPP_INFO(get_node()->get_logger(), "  URDF  : %s", urdf_file.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "  Mode  : effort interface + joint-velocity PID + Pinocchio gravity");
  RCLCPP_INFO(get_node()->get_logger(), "  Input : chained ref interfaces (%zu) (NO topic overwrite)", dof_);
  // 7) Debug: print Pinocchio joint order and indices
  RCLCPP_INFO(get_node()->get_logger(), "========== [Pinocchio] Model Joint Order ==========");
  RCLCPP_INFO(get_node()->get_logger(), "model_.nq=%d, model_.nv=%d, model_.njoints=%zu",
              model_.nq, model_.nv, model_.njoints);
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] Left End Link Name: %s",left_end_link_name.c_str());
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] Right Link ID: %d",left_end_frame_id_);
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] Left End Link Name: %s",right_end_link_name.c_str());
  RCLCPP_INFO(get_node()->get_logger(),"[Velocity Interface] Right Link ID: %d",right_end_frame_id_);

  RCLCPP_INFO(get_node()->get_logger(), "==============================");
  // 7.1 Print full joint list in Pinocchio internal order (skip universe=0)
  for (pinocchio::JointIndex jid = 1; jid < model_.njoints; ++jid)
  {
    const auto & j = model_.joints[jid];
    RCLCPP_INFO(get_node()->get_logger(),
                "[PIN] jid=%u name=%s idx_q=%d nq=%d idx_v=%d nv=%d",
                static_cast<unsigned>(jid),
                model_.names[jid].c_str(),
                j.idx_q(), j.nq(),
                j.idx_v(), j.nv());
  }

  // 7.2 Print mapping for controller joint list (params_.joints order)
  RCLCPP_INFO(get_node()->get_logger(), "========== [Pinocchio] Controller Joint Mapping ==========");
  for (size_t i = 0; i < params_.joints.size(); ++i)
  {
    const std::string & jname = params_.joints[i];
    pinocchio::JointIndex jid = model_.getJointId(jname);

    if (jid == 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "[CTL] i=%zu name=%s -> NOT FOUND in Pinocchio model",
                   i, jname.c_str());
      continue;
    }

    const auto & j = model_.joints[jid];
    RCLCPP_INFO(get_node()->get_logger(),
                "[CTL] i=%zu name=%s -> jid=%u idx_q=%d nq=%d idx_v=%d nv=%d",
                i, jname.c_str(),
                static_cast<unsigned>(jid),
                j.idx_q(), j.nq(),
                j.idx_v(), j.nv());
  }
  RCLCPP_INFO(get_node()->get_logger(), "===================================================");


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Pr2armVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  cfg.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    cfg.names.push_back(joint + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration Pr2armVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  cfg.names.reserve(2 * params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    cfg.names.push_back(joint + "/velocity");
  }
  for (const auto & joint : params_.joints)
  {
    cfg.names.push_back(joint + "/position");
  }
  return cfg;
}

std::vector<hardware_interface::CommandInterface>
Pr2armVelocityController::on_export_reference_interfaces()
{
  // Provide chained velocity reference interfaces
  reference_interfaces_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  // 确保 reference_interfaces_ 大小正确后，创建并返回 CommandInterface
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

bool Pr2armVelocityController::on_set_chained_mode(bool /*chained_mode*/)
{
  return true;
}

controller_interface::CallbackReturn Pr2armVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "PR2 Controller Activated (15DoF, effort)");

  // Reset chained refs
  for (auto & r : reference_interfaces_)
  {
    r = 0.0;
  }

  // Reset PID integrators
  std::fill(i_part_.begin(), i_part_.end(), 0.0);
  std::fill(last_error_.begin(), last_error_.end(), 0.0);

  // Safe output: zero torque
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Pr2armVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "PR2 Controller Deactivated (15DoF, effort)");

  for (auto & r : reference_interfaces_)
  {
    r = 0.0;
  }

  std::fill(i_part_.begin(), i_part_.end(), 0.0);
  std::fill(last_error_.begin(), last_error_.end(), 0.0);

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Pr2armVelocityController::update_reference_from_subscribers()
{
  // Intentionally do nothing: avoid any topic-based overwrite.
  return controller_interface::return_type::OK;
}

controller_interface::return_type Pr2armVelocityController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }

  const size_t dof = dof_;
  if (dof == 0 ||
      reference_interfaces_.size() != dof ||
      command_interfaces_.size() != dof ||
      state_interfaces_.size() != 2 * dof)
  {
    return controller_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  if (dt <= 0.0)
  {
    return controller_interface::return_type::OK;
  }

  // 1) Read q, qdot
  Eigen::VectorXd q(dof), qdot(dof);
  for (size_t i = 0; i < dof; ++i)
  {
    qdot(static_cast<Eigen::Index>(i)) = state_interfaces_[i].get_value();
    q(static_cast<Eigen::Index>(i))    = state_interfaces_[dof + i].get_value();
  }

  // 2) Read chained qdot_ref (NO clamp)
  Eigen::VectorXd qdot_ref(dof);
  for (size_t i = 0; i < dof; ++i)
  {
    qdot_ref(static_cast<Eigen::Index>(i)) = reference_interfaces_[i];
  }

  // 3) Joint velocity PID -> tau_pid
  Eigen::VectorXd tau_pid = Eigen::VectorXd::Zero(dof);
  for (size_t i = 0; i < dof; ++i)
  {
    const double err = qdot_ref(static_cast<Eigen::Index>(i)) - qdot(static_cast<Eigen::Index>(i));

    const double p = params_.kp[i] * err;

    i_part_[i] += params_.ki[i] * err * dt;
    if (i_part_[i] >  params_.ki_max[i]) i_part_[i] =  params_.ki_max[i];
    if (i_part_[i] < -params_.ki_max[i]) i_part_[i] = -params_.ki_max[i];

    const double d = params_.kd[i] * (err - last_error_[i]) / dt;
    last_error_[i] = err;

    tau_pid(static_cast<Eigen::Index>(i)) = p + i_part_[i] + d;
  }

  // 4) Gravity compensation from Pinocchio
  pinocchio::computeGeneralizedGravity(model_, data_, q);
  const Eigen::VectorXd tau_g = data_.g;
  Eigen::VectorXd tau = tau_g + tau_pid;

  // 5) Output tau = tau_g + tau_pid with torque limits
  if(left_contact_load_ !=0 || right_contact_load_ !=0 ) 
  {
    Eigen::VectorXd tau_load_compensation = Eigen::VectorXd::Zero(dof_);
    compute_jacobian_and_apply_load_compensation(q, tau_load_compensation);
    tau += tau_load_compensation;
  }


  for (size_t i = 0; i < dof; ++i)
  {
    double u = tau(static_cast<Eigen::Index>(i));
    if (u >  effort_limit_[i]) u =  effort_limit_[i];
    if (u < -effort_limit_[i]) u = -effort_limit_[i];
    command_interfaces_[i].set_value(u);
  }

  return controller_interface::return_type::OK;
}

// 左侧负载回调函数
void Pr2armVelocityController::loadCallbackLeft(const std_msgs::msg::Float64::SharedPtr msg)
{
  left_contact_load_ = msg->data;  // 更新左侧负载
}

// 右侧负载回调函数
void Pr2armVelocityController::loadCallbackRight(const std_msgs::msg::Float64::SharedPtr msg)
{
  right_contact_load_ = msg->data;  // 更新右侧负载
}

void Pr2armVelocityController::compute_jacobian_and_apply_load_compensation(
  const Eigen::VectorXd& q, Eigen::VectorXd& tau_load)
{
  // 1. 计算左臂和右臂的雅可比矩阵
  Eigen::MatrixXd J_left(6, model_.nv) ;
  Eigen::MatrixXd J_right(6, model_.nv) ;  // 初始化雅可比矩阵
  pinocchio::computeFrameJacobian(model_, data_, q, left_end_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_left);
  pinocchio::computeFrameJacobian(model_, data_, q, right_end_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_right);
  // RCLCPP_INFO(get_node()->get_logger(),"Left Jacobian size: %ld x %ld, model_.nv=%d",J_left.rows(), J_left.cols(), model_.nv);


  // 2. 设定末端负载力，假设负载力矩由外部传感器或模拟计算得到
  Eigen::VectorXd F_left_end(6);  // 6维：3维力 + 3维力矩
  Eigen::VectorXd F_right_end(6); // 6维：3维力 + 3维力矩
  F_left_end.setZero();
  F_right_end.setZero();
  // 3. 设置左臂和右臂的末端负载力（假设负载力矩分别作用于力的方向）
  F_left_end(2) = left_contact_load_ * 9.81;  // 左负载补偿（假设负载作用于Z轴力）
  F_right_end(2) = right_contact_load_ * 9.81; // 右负载补偿（假设负载作用于Z轴力）

  // 4. 通过雅可比矩阵将末端负载力转换为关节空间的力矩
  Eigen::VectorXd tau_left_compensation = J_left.transpose() * F_left_end;  // 左臂的负载补偿力矩
  Eigen::VectorXd tau_right_compensation = J_right.transpose() * F_right_end;  // 右臂的负载补偿力矩

  // 5. 合并左右臂的负载补偿力矩
  tau_load = tau_left_compensation + tau_right_compensation; // 总的负载补偿力矩

}

}  // namespace pr2_arm_velocity_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pr2_arm_velocity_controller::Pr2armVelocityController,
  controller_interface::ChainableControllerInterface)
