#pragma once

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <functional>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <memory>
#include <mutex>

namespace pr2_wholebody_interface{
using namespace ocs2;
/**
 * This class lets the user to command robot form interactive marker.
 */
class PR2WBCTargetTrajectoriesInteractiveMarker final {
 public:
  using GaolPoseToTargetTrajectories = std::function<ocs2::TargetTrajectories(
      const std::vector<Eigen::Vector3d>& position, const std::vector<Eigen::Quaterniond>& orientation,
      const SystemObservation& observation)>;

  /**
   * Constructor
   *
   * @param [in] node: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on
   * "topicPrefix_mpc_target" topic. Moreover, the latest observation is be
   * expected on "topicPrefix_mpc_observation" topic.
   * @param [in] gaolPoseToTargetTrajectories: A function which transforms the
   * commanded pose to TargetTrajectories.
   */
  PR2WBCTargetTrajectoriesInteractiveMarker(
      const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
      GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories);

  /**
   * Spins ROS to update the interactive markers.
   */
  void publishInteractiveMarker() { rclcpp::spin(node_); }

 private:
  visualization_msgs::msg::InteractiveMarker createInteractiveMarker(int mode) const;
  void leftProcessFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback);
  void rightProcessFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback);

  rclcpp::Node::SharedPtr node_;
  interactive_markers::MenuHandler left_menuHandler_;
  interactive_markers::MenuHandler right_menuHandler_;
  interactive_markers::InteractiveMarkerServer left_server_;
  interactive_markers::InteractiveMarkerServer right_server_;
  std::vector<Eigen::Vector3d> position_;
  std::vector<Eigen::Quaterniond> orientation_;

  GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories_;

  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher>
      targetTrajectoriesPublisherPtr_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSubscriber_;
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  