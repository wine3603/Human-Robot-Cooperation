#ifndef PR2_ARM_VELOCITY_CONTROLLER__PR2_ARM_VELOCITY_CONTROLLER_HPP_
#define PR2_ARM_VELOCITY_CONTROLLER__PR2_ARM_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "pr2_arm_velocity_controller_parameters.hpp"
#include "pr2_arm_controller/visibility_control.h"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <std_msgs/msg/float64.hpp> 
#include <pinocchio/spatial/fwd.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace pr2_arm_velocity_controller
{

class Pr2armVelocityController : public controller_interface::ChainableControllerInterface
{
public:
  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  Pr2armVelocityController();

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  PR2_ARM_VELOCITY_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<pr2_arm_velocity_controller::ParamListener> param_listener_;
  pr2_arm_velocity_controller::Params params_;

  size_t dof_{0};
  std::vector<std::string> state_joints_;

  // Chained velocity reference interfaces (upstream keeps unchanged)
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // Joint-space PID state
  std::vector<double> i_part_;
  std::vector<double> last_error_;

  // Safety torque limit (joint-space)
  std::vector<double> effort_limit_;

  // Pinocchio model for gravity compensation
  pinocchio::Model model_;
  pinocchio::Data data_{model_};

  // Optional: accept chained mode always
  bool on_set_chained_mode(bool chained_mode) override;

private:
  // no topic callback: we intentionally avoid topic overwrite
    // 负载信息成员变量
  double left_contact_load_;
  double right_contact_load_;

  // 负载信息订阅器
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr load_sub_left_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr load_sub_right_;

  // 负载信息的回调
  void loadCallbackLeft(const std_msgs::msg::Float64::SharedPtr msg);
  void loadCallbackRight(const std_msgs::msg::Float64::SharedPtr msg);

  pinocchio::FrameIndex left_end_frame_id_;
  pinocchio::FrameIndex right_end_frame_id_;

  // 雅可比矩阵计算与负载补偿
  void compute_jacobian_and_apply_load_compensation(const Eigen::VectorXd& q, Eigen::VectorXd& tau_load);
};

}  // namespace pr2_arm_velocity_controller

#endif  // PR2_ARM_VELOCITY_CONTROLLER__PR2_ARM_VELOCITY_CONTROLLER_HPP_
