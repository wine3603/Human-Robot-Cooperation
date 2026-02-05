#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/wrench.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/moying/MoyingState_.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>


#define TOPIC_MOYINGSTATE "moying/state"

class MoyingTfNode : public rclcpp::Node
{
public:
    MoyingTfNode()
        : Node("moying_mujoco_sensor_node")
    {
        // 初始化 TF 发布器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初始化 Cyclone DDS
        unitree::robot::ChannelFactory::Instance()->Init(1, "lo");

        // 创建 DDS 订阅器
        state_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<moying::msg::dds_::MoyingState_>>(TOPIC_MOYINGSTATE);
        state_subscriber_->InitChannel(
            static_cast<std::function<void(const void*)>>(
                [this](const void *msg_ptr)
                {
                    const auto *msg = static_cast<const moying::msg::dds_::MoyingState_ *>(msg_ptr);
                    publish_tf("world", "mcr/base_link", msg->mcr_base_pos(), msg->mcr_base_quat());
                    publish_tf("world", "desk/base_link", msg->desk_base_pos(), msg->desk_base_quat());
                    publish_wrenches(msg);
                }),
            1);
        mcr_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vrpn/mcr/pose", 10);
        desk_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vrpn/desk/pose", 10);
        right_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/right/ft_sensor/wrench", 10);
        left_wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/left/ft_sensor/wrench", 10);

    }

private:
    void publish_tf(const std::string &parent_frame,
                    const std::string &child_frame,
                    const std::array<float, 3> &pos,
                    const std::array<float, 4> &quat)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = parent_frame;
        tf_msg.child_frame_id = child_frame;

        tf_msg.transform.translation.x = pos[0];
        tf_msg.transform.translation.y = pos[1];
        tf_msg.transform.translation.z = pos[2];

        tf_msg.transform.rotation.x = quat[1];
        tf_msg.transform.rotation.y = quat[2];
        tf_msg.transform.rotation.z = quat[3];
        tf_msg.transform.rotation.w = quat[0];

        tf_broadcaster_->sendTransform(tf_msg);

         // PoseStamped message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = tf_msg.header;
        pose_msg.pose.position.x = pos[0];
        pose_msg.pose.position.y = pos[1];
        pose_msg.pose.position.z = pos[2];
        pose_msg.pose.orientation.x = quat[1];
        pose_msg.pose.orientation.y = quat[2];
        pose_msg.pose.orientation.z = quat[3];
        pose_msg.pose.orientation.w = quat[0];

        // 发布到对应话题
        if (child_frame == "mcr/base_link")
            mcr_pub_->publish(pose_msg);
        else if (child_frame == "desk/base_link")
            desk_pub_->publish(pose_msg);
    }

    void publish_wrenches(const moying::msg::dds_::MoyingState_ *msg)
    {
        auto now = this->get_clock()->now();

        auto make_wrench_msg = [&](const std::string &frame_id,
                                const std::array<float, 3> &force,
                                const std::array<float, 3> &torque) {
            geometry_msgs::msg::WrenchStamped wrench_msg;
            wrench_msg.header.stamp = now;
            wrench_msg.header.frame_id = frame_id;
            wrench_msg.wrench.force.x = force[0];
            wrench_msg.wrench.force.y = force[1];
            wrench_msg.wrench.force.z = force[2];
            wrench_msg.wrench.torque.x = torque[0];
            wrench_msg.wrench.torque.y = torque[1];
            wrench_msg.wrench.torque.z = torque[2];
            return wrench_msg;
        };

        right_wrench_pub_->publish(make_wrench_msg("right_arm_robotiq_ft_frame_id", msg->right_force(), msg->right_torque()));
        left_wrench_pub_->publish(make_wrench_msg("left_arm_robotiq_ft_frame_id", msg->left_force(), msg->left_torque()));
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    unitree::robot::ChannelSubscriberPtr<moying::msg::dds_::MoyingState_> state_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mor_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mcr_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desk_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr mor_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoyingTfNode>());
    rclcpp::shutdown();
    return 0;
}
