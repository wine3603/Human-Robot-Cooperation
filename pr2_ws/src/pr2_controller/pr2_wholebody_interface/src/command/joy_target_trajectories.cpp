#include "pr2_wholebody_interface/command/joy_target_trajectories.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace pr2_wholebody_interface{
using namespace std::chrono_literals;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PR2WBCJoyTargetTrajectories::PR2WBCJoyTargetTrajectories(
    const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
    GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories,std::string taskFile)
    : node_(node),
      gaolPoseToTargetTrajectories_(std::move(gaolPoseToTargetTrajectories)) {
  // observation subscriber
  auto observationCallback =
      [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
      };
  observationSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
          topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(
      new TargetTrajectoriesRosPublisher(node_, topicPrefix));
  timer_ = node_->create_wall_timer(
      100ms, std::bind(&PR2WBCJoyTargetTrajectories::timer_callback,this));
  // std::string joy_topic_name=node->get_parameter("joy_topic_name").as_string();
  joy_command_subsciption_ = node_->create_subscription<sensor_msgs::msg::Joy>("/mcr/joy", rclcpp::SystemDefaultsQoS(), [this](const  sensor_msgs::msg::Joy::SharedPtr twist)
  {
    joy_command_ptr_.writeFromNonRT(twist);
  });
  left_goal_point_subscription_= node_->create_subscription<geometry_msgs::msg::PoseStamped>("/mcr/left_goal_pose", rclcpp::SystemDefaultsQoS(), [this](const  geometry_msgs::msg::PoseStamped::SharedPtr gpoint)
  {
    left_goal_point_ptr_.writeFromNonRT(gpoint);
  });
  right_goal_point_subscription_= node_->create_subscription<geometry_msgs::msg::PoseStamped>("/mcr/right_goal_pose", rclcpp::SystemDefaultsQoS(), [this](const  geometry_msgs::msg::PoseStamped::SharedPtr gpoint)
  {
    right_goal_point_ptr_.writeFromNonRT(gpoint);
  });
  base_goal_point_subscription_= node_->create_subscription<geometry_msgs::msg::PoseStamped>("/mcr/base_goal_pose", rclcpp::SystemDefaultsQoS(), [this](const  geometry_msgs::msg::PoseStamped::SharedPtr gpoint)
  {
    base_goal_point_ptr_.writeFromNonRT(gpoint);
  });


  RCLCPP_INFO_STREAM(node_->get_logger(),"[PR2WBCJoyTargetTrajectory] Loading...");
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    RCLCPP_INFO_STREAM(node_->get_logger(),"[PR2WBCJoyTargetTrajectory] Loading task from:"<<taskFilePath);
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"[PR2WBCJoyTargetTrajectory] Can't loading task file");
    throw std::invalid_argument("[PR2WBCJoyTargetTrajectory] task file not found: " + taskFilePath.string());
  }
  ocs2::vector_t left_pos = ocs2::vector_t::Zero(3);
  ocs2::vector_t right_pos = ocs2::vector_t::Zero(3);
  ocs2::vector_t base_pos = ocs2::vector_t::Zero(3);
  ocs2::vector_t left_ori = ocs2::vector_t::Zero(4);
  ocs2::vector_t right_ori = ocs2::vector_t::Zero(4);
  ocs2::vector_t base_ori = ocs2::vector_t::Zero(4);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.left_position",left_pos);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.left_orientation", left_ori);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.right_position",right_pos);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.right_orientation", right_ori);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.base_position",base_pos);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.base_orientation", base_ori);
  //init pos
  Eigen::Vector3d left_init_pos(left_pos(0),left_pos(1),left_pos(2));
  Eigen::Vector3d right_init_pos(right_pos(0),right_pos(1),right_pos(2));
  Eigen::Vector3d base_init_pos(base_pos(0),base_pos(1),base_pos(2));

  left_position_ = left_init_pos;
  init_left_position_ = left_position_;

  right_position_ = right_init_pos;
  init_right_position_ = right_position_;

  init_base_position_ = base_init_pos;
  base_position_ = init_base_position_;

  //init ori
  init_left_orientation_ = Eigen::Quaterniond(left_ori(0),left_ori(1),left_ori(2),left_ori(3));
  init_right_orientation_ = Eigen::Quaterniond(right_ori(0),right_ori(1),right_ori(2),right_ori(3));
  init_base_orientation_ = Eigen::Quaterniond(base_ori(0),base_ori(1),base_ori(2),base_ori(3));

  left_orientation_ = init_left_orientation_;
  right_orientation_ = init_right_orientation_;
  base_orientation_ = init_base_orientation_;
  
  use_cmd = false;
  use_joy = false;
  close_end_eff = false;

}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PR2WBCJoyTargetTrajectories::timer_callback() 
{
  // if((*joy_command)->buttons[2] == 1)
  // {
  use_cmd = false;
  if(use_cmd==true)
  {

    auto joy_command = joy_command_ptr_.readFromRT();
    if (!joy_command || !(*joy_command)) {
      std::cerr<<"No joy command received"<<std::endl;
      return;
    }
    auto base_gpoint = base_goal_point_ptr_.readFromRT();
    if (!base_gpoint|| !(*base_gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Goa Point] no pub yet.");
      return;
    }
    auto left_gpoint = left_goal_point_ptr_.readFromRT();
    if (!left_gpoint|| !(*left_gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Goa Point] no pub yet.");
      return;
    }
    RCLCPP_INFO_ONCE(node_->get_logger(),"[GoalPoint] Use Init info get.");

    std::vector<Eigen::Vector3d> vpos(3);
    std::vector<Eigen::Quaterniond> vori(3);
    vpos[0] = init_left_position_;
    vpos[1] = init_right_position_;
    // vpos[2] = init_base_position_;
    vori[0] = init_left_orientation_;
    vori[1] = init_right_orientation_;
    // vori[2] = init_base_orientation_;
    vpos[2] = Eigen::Vector3d((*base_gpoint)->pose.position.x,(*base_gpoint)->pose.position.y,init_base_position_.z());
    Eigen::AngleAxisd rotation_vector((*base_gpoint)->pose.position.z, Eigen::Vector3d::UnitZ());
    vori[2] = Eigen::Quaterniond(rotation_vector);
    double offset  = (*joy_command)->axes[2]-1;
    double roty_angle = (*joy_command)->axes[0]* -30.0 * M_PI/180.0;
    double rotny_angle = (*joy_command)->axes[0]* -30.0 * M_PI/180.0;
    double add_height = (*joy_command)->axes[4]*0.1;
    Eigen::Vector3d centor_pos((*left_gpoint)->pose.position.x,(*left_gpoint)->pose.position.y,(*left_gpoint)->pose.position.z);
    Eigen::Vector3d centor_offset = Eigen::Vector3d::Zero();
    // if(centor_pos.norm()<0.5)
      centor_offset = centor_pos;


    Eigen::AngleAxisd roty_vec(roty_angle,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotny_vec(rotny_angle,Eigen::Vector3d::UnitX());
    Eigen::Quaterniond roty_qua(roty_vec);
    Eigen::Quaterniond rotny_qua(rotny_vec);
    


    vpos[0] = vpos[0] + Eigen::Vector3d(0,offset/10.0,0.2*sin(roty_angle)+add_height) + centor_offset;
    vori[0] = vori[0]*roty_qua;
    vpos[1] = vpos[1] + Eigen::Vector3d(0,offset/-10.0,-0.2*sin(rotny_angle)+add_height) + centor_offset;
    vori[1] = vori[1]*rotny_qua;

    ocs2::SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    const auto targetTrajectories =
      gaolPoseToTargetTrajectories_(vpos, vori, observation);

    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        targetTrajectories);

  }
  //use cmd mode
  else
  {
    auto left_gpoint = left_goal_point_ptr_.readFromRT();
    if (!left_gpoint|| !(*left_gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Goa Point] no pub yet.");
      return;
    }
    auto right_gpoint = right_goal_point_ptr_.readFromRT();
    if (!right_gpoint|| !(*right_gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Goa Point] no pub yet.");
      return;
    }
    auto base_gpoint = base_goal_point_ptr_.readFromRT();
    if (!base_gpoint|| !(*base_gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Goa Point] no pub yet.");
      return;
    }

    RCLCPP_INFO_ONCE(node_->get_logger(),"[GoalPoint] get.");

    left_position_.x()  =  (*left_gpoint)->pose.position.x;
    left_position_.y()  =  (*left_gpoint)->pose.position.y;
    left_position_.z()  =  (*left_gpoint)->pose.position.z;
    left_orientation_.x()  =  (*left_gpoint)->pose.orientation.x;
    left_orientation_.y()  =  (*left_gpoint)->pose.orientation.y;
    left_orientation_.z()  =  (*left_gpoint)->pose.orientation.z;
    left_orientation_.w()  =  (*left_gpoint)->pose.orientation.w;

    right_position_.x()  =  (*right_gpoint)->pose.position.x;
    right_position_.y()  =  (*right_gpoint)->pose.position.y;
    right_position_.z()  =  (*right_gpoint)->pose.position.z;
    right_orientation_.x()  =  (*right_gpoint)->pose.orientation.x;
    right_orientation_.y()  =  (*right_gpoint)->pose.orientation.y;
    right_orientation_.z()  =  (*right_gpoint)->pose.orientation.z;
    right_orientation_.w()  =  (*right_gpoint)->pose.orientation.w;

    base_position_.x()  =  (*base_gpoint)->pose.position.x;
    base_position_.y()  =  (*base_gpoint)->pose.position.y;
    base_position_.z()  =  (*base_gpoint)->pose.position.z;
    base_orientation_.x()  =  (*base_gpoint)->pose.orientation.x;
    base_orientation_.y()  =  (*base_gpoint)->pose.orientation.y;
    base_orientation_.z()  =  (*base_gpoint)->pose.orientation.z;
    base_orientation_.w()  =  (*base_gpoint)->pose.orientation.w;

    std::vector<Eigen::Vector3d> vpos(3);
    std::vector<Eigen::Quaterniond> vori(3);
    vpos[0] = left_position_;
    vpos[1] = right_position_;
    vpos[2] = base_position_;
    vori[0] = left_orientation_;
    vori[1] = right_orientation_;
    vori[2] = base_orientation_;

    ocs2::SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    const auto targetTrajectories =
      gaolPoseToTargetTrajectories_(vpos, vori, observation);

    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        targetTrajectories);
  }
}
}  // namespace pr2_wholebody_interfac