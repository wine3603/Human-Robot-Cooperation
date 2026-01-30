#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/moying/MoyingState_.hpp>
#include <unitree/idl/moying/Moying_Mcr_Cmd_.hpp>

#define TOPIC_MOYINGSTATE "moying/state"
#define TOPIC_MOYINGCMD "moying/mcr_cmd"

using namespace unitree::robot;
using namespace unitree::common;

void MoyingStateHandler(const void *msg)
{
    const moying::msg::dds_::MoyingState_ *s = (const moying::msg::dds_::MoyingState_ *)msg;

    std::cout << "[MoyingState] 前6个关节角度为:" << std::endl;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "  joint_position[" << i << "] = " << s->mcrl_joint_position()[i] << std::endl;
    }
    std::cout << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(1, "lo");

    ChannelSubscriber<moying::msg::dds_::MoyingState_> state_suber(TOPIC_MOYINGSTATE);
    state_suber.InitChannel(MoyingStateHandler);

    ChannelPublisher<moying::msg::dds_::MoyingMcrCmd_> cmd_puber(TOPIC_MOYINGCMD);
    cmd_puber.InitChannel();

    double torque = 10.0;
    int counter = 0;
    const int switch_cycle = 1000;  // 2s at 500Hz

    while (true) {
        moying::msg::dds_::MoyingMcrCmd_ cmd{};

        // 前 18 个关节施加 torque（3x6）
        for (int i = 0; i < 6; ++i) {
            cmd.mcrl_joint_tau()[i] = torque;
        }

        cmd_puber.Write(cmd);

        // 每 2 秒反转一次力方向
        counter++;
        if (counter >= switch_cycle) {
            torque = -torque;
            counter = 0;
        }

        usleep(2000);  // 500Hz
    }

    return 0;
}
