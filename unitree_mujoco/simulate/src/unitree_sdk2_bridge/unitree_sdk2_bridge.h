#ifndef UNITREE_SDK2_BRIDGE_H
#define UNITREE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <cstring>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/moying/MoyingCmd_.hpp>
#include <unitree/idl/moying/Moying_Mor_Cmd_.hpp>
#include <unitree/idl/moying/Moying_Mcr_Cmd_.hpp>
#include <unitree/idl/moying/MoyingState_.hpp>
#include <unitree/idl/pr2/pr2_cmd_.hpp>
#include <unitree/idl/pr2/pr2_state_.hpp>
#include <mujoco/mujoco.h>
#include "../joystick/joystick.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_WIRELESS_CONTROLLER "rt/wirelesscontroller"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20
#define NUM_MOTOR_IDL_HG 35

typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

typedef struct
{
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;

    uint8_t idle[16];
} xRockerBtnDataStruct;

// Defaults to xbox gamepad
struct JoystickId
{

    map<string, int> axis =
        {
            {"LX", 0}, // Left stick axis x
            {"LY", 1}, // Left stick axis y
            {"RX", 3}, // Right stick axis x
            {"RY", 4}, // Right stick axis y
            {"LT", 2}, // Left trigger
            {"RT", 5}, // Right trigger
            {"DX", 6}, // Directional pad x
            {"DY", 7}, // Directional pad y
    };

    map<string, int> button =
        {
            {"X", 2},
            {"Y", 3},
            {"B", 1},
            {"A", 0},
            {"LB", 4},
            {"RB", 5},
            {"SELECT", 6},
            {"START", 7},
    };
};

class UnitreeSdk2Bridge
{
public:
    UnitreeSdk2Bridge(mjModel *model, mjData *data);       //构造函数，传入参数mjModel和mjData
    ~UnitreeSdk2Bridge();

    void LowCmdGoHandler(const void *msg);                //Go系列订阅 LowCmd 消息的回调函数
    void LowCmdHgHandler(const void *msg);                //Hg系列订阅 LowCmd 消息的回调函数
    void Moying_Mor_CmdHandler(const void *msg);          //moying系列订阅 LowCmd 消息的回调函数
    void Moying_Mcr_CmdHandler(const void *msg);          //moying系列订阅 LowCmd 消息的回调函数
    void PublishMoyingState();
    void PR2CmdHandler(const void* msg);
    void PublishPR2State();
    void PublishLowStateGo();
    void PublishLowStateHg();
    void PublishHighState();
    void PublishWirelessController();                      //发布状态（MuJoCo → DDS）

    void Run();                                            //核心运行函数
    void PrintSceneInformation();                          //打印仿真场景信息 

    void CheckSensor();                                    //检查传感器
    void SetupJoystick(string device, string js_type, int bits);  //初始化手柄设备

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_go_suber_;  //通过 ChannelFactory 创建的go通信类成员
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> low_cmd_hg_suber_;  //通过 ChannelFactory 创建的hg通信类成员
    // Moying 相关成员变量
    ChannelSubscriberPtr<moying::msg::dds_::MoyingMorCmd_ > MoyingMorCmd_suber_;
    ChannelSubscriberPtr<moying::msg::dds_::MoyingMcrCmd_ > MoyingMcrCmd_suber_;
    ChannelSubscriberPtr<pr2cmd::msg::dds_::PR2ActuatorCmd > Pr2Cmd_suber_;
    
    moying::msg::dds_::MoyingState_ moying_state_{};
    pr2cmd::msg::dds_::PR2SensorState pr2_state_{};
    // moying::msg::dds_::MoyingCmd_ moying_cmd_;


    unitree_go::msg::dds_::LowState_ low_state_go_{};
    unitree_hg::msg::dds_::LowState_ low_state_hg_{};              //状态消息数据结构 （IDL 结构体）   
    unitree_go::msg::dds_::SportModeState_ high_state_{};
    unitree_go::msg::dds_::WirelessController_ wireless_controller_{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_go_puber_;
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> low_state_hg_puber_;
    ChannelPublisherPtr<moying::msg::dds_::MoyingState_> moying_state_puber_;
    ChannelPublisherPtr<pr2cmd::msg::dds_::PR2SensorState> pr2_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::SportModeState_> high_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> wireless_controller_puber_;

    ThreadPtr lowStatePuberThreadPtr;
    ThreadPtr HighStatePuberThreadPtr;
    ThreadPtr WirelessControllerPuberThreadPtr;
    // 新增 PR2 专用的线程指针，以便和 Moying 并行发布状态
    ThreadPtr pr2StatePuberThreadPtr;

    xKeySwitchUnion dds_keys_ = {};
    xRockerBtnDataStruct wireless_remote_ = {};

    JoystickId js_id_;
    Joystick *js_;
    int max_value_ = (1 << 15); // 16 bits joystick

    mjData *mj_data_;
    mjModel *mj_model_;           // 仿真数据指针

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
    bool have_touch_sensor_ = false;
    int idl_type_ = 0; // 0: unitree_go, 1: unitree_hg

    // [修改] 新增布尔标志位，用于记录特定机器人是否存在 (支持多机共存)
    bool has_moying_ = false;  // 检测到 MOR 或 MCR
    bool has_pr2_ = false;     // 检测到 PR2

private:
    

    void GetWirelessRemote();  //读取值并填充 wireless_remote_ 与 wireless_controller_



};

#endif
