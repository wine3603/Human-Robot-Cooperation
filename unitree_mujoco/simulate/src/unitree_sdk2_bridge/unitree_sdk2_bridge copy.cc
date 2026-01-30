#include "unitree_sdk2_bridge.h"

UnitreeSdk2Bridge::UnitreeSdk2Bridge(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    CheckSensor();

    if (idl_type_ == 0)
    {
        low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        low_cmd_go_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

        low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        low_state_go_puber_->InitChannel();

        lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishLowStateGo, this);
    }
    else if (idl_type_ == 1)
    {
        low_cmd_hg_suber_.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        low_cmd_hg_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowCmdHgHandler, this, placeholders::_1), 1);

        low_state_hg_puber_.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        low_state_hg_puber_->InitChannel();

        lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishLowStateHg, this);
    }
    else if (idl_type_ == 2)
    {
        // Moying 专用控制逻辑
        MoyingMorCmd_suber_.reset(new ChannelSubscriber<moying::msg::dds_::MoyingMorCmd_>("moying/mor_cmd"));
        MoyingMorCmd_suber_->InitChannel(bind(&UnitreeSdk2Bridge::Moying_Mor_CmdHandler, this, placeholders::_1), 1);
        MoyingMcrCmd_suber_.reset(new ChannelSubscriber<moying::msg::dds_::MoyingMcrCmd_>("moying/mcr_cmd"));
        MoyingMcrCmd_suber_->InitChannel(bind(&UnitreeSdk2Bridge::Moying_Mcr_CmdHandler, this, placeholders::_1), 1);

        moying_state_puber_.reset(new ChannelPublisher<moying::msg::dds_::MoyingState_>("moying/state"));
        moying_state_puber_->InitChannel();

        lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishMoyingState, this);
    }

    high_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    high_state_puber_->InitChannel();
    wireless_controller_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::WirelessController_>(TOPIC_WIRELESS_CONTROLLER));
    wireless_controller_puber_->InitChannel();

    HighStatePuberThreadPtr = CreateRecurrentThreadEx("highstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishHighState, this);
    WirelessControllerPuberThreadPtr = CreateRecurrentThreadEx("wirelesscontroller", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishWirelessController, this);
}

UnitreeSdk2Bridge::~UnitreeSdk2Bridge()
{
    delete js_;
}

void UnitreeSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}

void UnitreeSdk2Bridge::LowCmdHgHandler(const void *msg)
{
    const unitree_hg::msg::dds_::LowCmd_ *cmd = (const unitree_hg::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}

void UnitreeSdk2Bridge::Moying_Mor_CmdHandler(const void *msg)
{
    const moying::msg::dds_::MoyingMorCmd_ *cmd = static_cast<const moying::msg::dds_::MoyingMorCmd_ *>(msg);
    if (!mj_data_ || !mj_model_) return;

    int ctrl_index = 0;
    // MOR 机械臂力矩控制 (6个)
    for (int i = 0; i < 6; ++i) {
        mj_data_->ctrl[ctrl_index++] = cmd->mor_joint_tau()[i];
    }

    // MOR 轮子速度控制 (4个)
    for (int i = 0; i < 4; ++i) {
        mj_data_->ctrl[ctrl_index++] = cmd->mor_wheel_velocity()[i];
    }
    // MOR Robotiq 夹爪位置 (1个)
    mj_data_->ctrl[ctrl_index++] = cmd->mor_robotiq_position();
}

void UnitreeSdk2Bridge::Moying_Mcr_CmdHandler(const void *msg)
{
    const moying::msg::dds_::MoyingMcrCmd_ *cmd = static_cast<const moying::msg::dds_::MoyingMcrCmd_ *>(msg);
    if (!mj_data_ || !mj_model_) return;

    int ctrl_index = 11;

    // MCR 左机械臂力矩控制 (6个)
    for (int i = 0; i < 6; ++i) {
        mj_data_->ctrl[ctrl_index++] = cmd->mcrl_joint_tau()[i];
    }
    // MCR 右机械臂力矩控制 (6个)
    for (int i = 0; i < 6; ++i) {
        mj_data_->ctrl[ctrl_index++] = cmd->mcrr_joint_tau()[i];
    }
    // MCR 轮子速度控制 (4个)
    for (int i = 0; i < 4; ++i) {
        mj_data_->ctrl[ctrl_index++] = cmd->mcr_wheel_velocity()[i];
    }
    // MCR 左夹爪位置 (1个)
    mj_data_->ctrl[ctrl_index++] = cmd->mcr_robotiqleft_position();
    // MCR 右夹爪位置 (1个)
    mj_data_->ctrl[ctrl_index++] = cmd->mcr_robotiqright_position();
}


void UnitreeSdk2Bridge::PublishLowStateGo()
{
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            low_state_go_.motor_state()[i].q() = mj_data_->sensordata[i];
            low_state_go_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
            low_state_go_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
        }

        if (have_frame_sensor_)
        {
            low_state_go_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
            low_state_go_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
            low_state_go_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
            low_state_go_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

	    double w = low_state_go_.imu_state().quaternion()[0];
	    double x = low_state_go_.imu_state().quaternion()[1];
	    double y = low_state_go_.imu_state().quaternion()[2];
	    double z = low_state_go_.imu_state().quaternion()[3];

	    low_state_go_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	    low_state_go_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
	    low_state_go_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            low_state_go_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
            low_state_go_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
            low_state_go_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

            low_state_go_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
            low_state_go_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
            low_state_go_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
        }

        if (js_)
        {
            GetWirelessRemote();
            memcpy(&low_state_go_.wireless_remote()[0], &wireless_remote_, 40);
        }

        low_state_go_puber_->Write(low_state_go_);
    }
}

void UnitreeSdk2Bridge::PublishLowStateHg()
{
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            low_state_hg_.motor_state()[i].q() = mj_data_->sensordata[i];
            low_state_hg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
            low_state_hg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
        }

        if (have_frame_sensor_)
        {
            low_state_hg_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
            low_state_hg_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
            low_state_hg_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
            low_state_hg_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

	    double w = low_state_hg_.imu_state().quaternion()[0];
	    double x = low_state_hg_.imu_state().quaternion()[1];
	    double y = low_state_hg_.imu_state().quaternion()[2];
	    double z = low_state_hg_.imu_state().quaternion()[3];

	    low_state_hg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	    low_state_hg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
	    low_state_hg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            low_state_hg_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
            low_state_hg_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
            low_state_hg_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

            low_state_hg_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
            low_state_hg_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
            low_state_hg_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
        }

        if (js_)
        {
            GetWirelessRemote();
            memcpy(&low_state_hg_.wireless_remote()[0], &wireless_remote_, 40);
        }

        low_state_hg_puber_->Write(low_state_hg_);
    }
}

void UnitreeSdk2Bridge::PublishMoyingState()
{
    if (!mj_data_ || !mj_model_) return;

    int sensor_id = 0;

    // MOR joint position / velocity / torque
    for (int i = 0; i < 6; ++i) {
        moying_state_.mor_joint_position()[i] = mj_data_->sensordata[sensor_id++];
    }
    for (int i = 0; i < 6; ++i) {
        moying_state_.mcrl_joint_position()[i] = mj_data_->sensordata[sensor_id++];
    }
    for (int i = 0; i < 6; ++i) {
        moying_state_.mcrr_joint_position()[i] = mj_data_->sensordata[sensor_id++];
    }

    for (int i = 0; i < 6; ++i) {
        moying_state_.mor_joint_velocity()[i] = mj_data_->sensordata[sensor_id++];
    }
    for (int i = 0; i < 6; ++i) {
        moying_state_.mcrl_joint_velocity()[i] = mj_data_->sensordata[sensor_id++];
    }
    for (int i = 0; i < 6; ++i) {
        moying_state_.mcrr_joint_velocity()[i] = mj_data_->sensordata[sensor_id++];
    }

    for (int i = 0; i < 6; ++i) {
        moying_state_.mor_joint_torque()[i] = mj_data_->sensordata[sensor_id++];
    }
    for (int i = 0; i < 6; ++i) {
        moying_state_.mcrl_joint_torque()[i] = mj_data_->sensordata[sensor_id++];
    }
    for (int i = 0; i < 6; ++i) {
        moying_state_.mcrr_joint_torque()[i] = mj_data_->sensordata[sensor_id++];
    }

    // base pos/quat
    for (int i = 0; i < 3; ++i) moying_state_.mor_base_pos()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 4; ++i) moying_state_.mor_base_quat()[i] = mj_data_->sensordata[sensor_id++];

    for (int i = 0; i < 3; ++i) moying_state_.mcr_base_pos()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 4; ++i) moying_state_.mcr_base_quat()[i] = mj_data_->sensordata[sensor_id++];

    for (int i = 0; i < 3; ++i) moying_state_.desk_base_pos()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 4; ++i) moying_state_.desk_base_quat()[i] = mj_data_->sensordata[sensor_id++];

    // double w = moying_state_.desk_base_quat()[0];
	// double x = moying_state_.desk_base_quat()[1];
	// double y = moying_state_.desk_base_quat()[2];
	// double z = moying_state_.desk_base_quat()[3];

    // double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	// double pitch = asin(2 * (w * y - z * x));
	// double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    
    // std::cerr << "desk quaternion is : w= " << w <<", x= "<<x <<", y= "<< y << ", z= "<< z << std::endl;
    // std::cerr << "desk eulor angle is : roll= " << roll <<", pitch= "<< pitch <<", yaw= "<<yaw << std::endl;
   
    // force/torque sensors
    for (int i = 0; i < 3; ++i) moying_state_.mor_force()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 3; ++i) moying_state_.mor_torque()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 3; ++i) moying_state_.right_force()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 3; ++i) moying_state_.right_torque()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 3; ++i) moying_state_.left_force()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 3; ++i) moying_state_.left_torque()[i] = mj_data_->sensordata[sensor_id++];

    for (int i = 0; i < 4; ++i) moying_state_.mor_wheel_velocity()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 4; ++i) moying_state_.mcr_wheel_velocity()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 4; ++i) moying_state_.mor_wheel_position()[i] = mj_data_->sensordata[sensor_id++];
    for (int i = 0; i < 4; ++i) moying_state_.mcr_wheel_position()[i] = mj_data_->sensordata[sensor_id++];

    moying_state_.mcr_robotiqleft_position() = mj_data_->sensordata[sensor_id++];
    moying_state_.mcr_robotiqleft_position() = mj_data_->sensordata[sensor_id++];
    moying_state_.mcr_robotiqright_position() = mj_data_->sensordata[sensor_id++];

    // // Debug check（可选）
    // if (sensor_id != mj_model_->nsensor) {
    //     std::cerr << "[PublishMoyingState] Warning: sensor_id = " << sensor_id
    //               << ", but model has " << mj_model_->nsensor << " sensors." << std::endl;
    // }

    // 发布
    moying_state_puber_->Write(moying_state_);
}


void UnitreeSdk2Bridge::PublishHighState()
{
    if (mj_data_ && have_frame_sensor_)
    {

        high_state_.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
        high_state_.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
        high_state_.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];

        high_state_.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
        high_state_.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
        high_state_.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];

        high_state_puber_->Write(high_state_);
    }
}

void UnitreeSdk2Bridge::PublishWirelessController()
{
    if (js_)
    {
        js_->getState();
        dds_keys_.components.R1 = js_->button_[js_id_.button["RB"]];
        dds_keys_.components.L1 = js_->button_[js_id_.button["LB"]];
        dds_keys_.components.start = js_->button_[js_id_.button["START"]];
        dds_keys_.components.select = js_->button_[js_id_.button["SELECT"]];
        dds_keys_.components.R2 = (js_->axis_[js_id_.axis["RT"]] > 0);
        dds_keys_.components.L2 = (js_->axis_[js_id_.axis["LT"]] > 0);
        dds_keys_.components.F1 = 0;
        dds_keys_.components.F2 = 0;
        dds_keys_.components.A = js_->button_[js_id_.button["A"]];
        dds_keys_.components.B = js_->button_[js_id_.button["B"]];
        dds_keys_.components.X = js_->button_[js_id_.button["X"]];
        dds_keys_.components.Y = js_->button_[js_id_.button["Y"]];
        dds_keys_.components.up = (js_->axis_[js_id_.axis["DY"]] < 0);
        dds_keys_.components.right = (js_->axis_[js_id_.axis["DX"]] > 0);
        dds_keys_.components.down = (js_->axis_[js_id_.axis["DY"]] > 0);
        dds_keys_.components.left = (js_->axis_[js_id_.axis["DX"]] < 0);

        wireless_controller_.lx() = double(js_->axis_[js_id_.axis["LX"]]) / max_value_;
        wireless_controller_.ly() = -double(js_->axis_[js_id_.axis["LY"]]) / max_value_;
        wireless_controller_.rx() = double(js_->axis_[js_id_.axis["RX"]]) / max_value_;
        wireless_controller_.ry() = -double(js_->axis_[js_id_.axis["RY"]]) / max_value_;
        wireless_controller_.keys() = dds_keys_.value;

        wireless_controller_puber_->Write(wireless_controller_);
    }
}

void UnitreeSdk2Bridge::Run()
{
    while (1)
    {
        sleep(2);
    }
}

void UnitreeSdk2Bridge::SetupJoystick(string device, string js_type, int bits)
{
    js_ = new Joystick(device);
    if (!js_->isFound())
    {
        cout << "Error: Joystick open failed." << endl;
        exit(1);
    }

    max_value_ = (1 << (bits - 1));

    if (js_type == "xbox")
    {
        js_id_.axis["LX"] = 0; // Left stick axis x
        js_id_.axis["LY"] = 1; // Left stick axis y
        js_id_.axis["RX"] = 3; // Right stick axis x
        js_id_.axis["RY"] = 4; // Right stick axis y
        js_id_.axis["LT"] = 2; // Left trigger
        js_id_.axis["RT"] = 5; // Right trigger
        js_id_.axis["DX"] = 6; // Directional pad x
        js_id_.axis["DY"] = 7; // Directional pad y

        js_id_.button["X"] = 2;
        js_id_.button["Y"] = 3;
        js_id_.button["B"] = 1;
        js_id_.button["A"] = 0;
        js_id_.button["LB"] = 4;
        js_id_.button["RB"] = 5;
        js_id_.button["SELECT"] = 6;
        js_id_.button["START"] = 7;
    }
    else if (js_type == "switch")
    {
        js_id_.axis["LX"] = 0; // Left stick axis x
        js_id_.axis["LY"] = 1; // Left stick axis y
        js_id_.axis["RX"] = 2; // Right stick axis x
        js_id_.axis["RY"] = 3; // Right stick axis y
        js_id_.axis["LT"] = 5; // Left trigger
        js_id_.axis["RT"] = 4; // Right trigger
        js_id_.axis["DX"] = 6; // Directional pad x
        js_id_.axis["DY"] = 7; // Directional pad y

        js_id_.button["X"] = 3;
        js_id_.button["Y"] = 4;
        js_id_.button["B"] = 1;
        js_id_.button["A"] = 0;
        js_id_.button["LB"] = 6;
        js_id_.button["RB"] = 7;
        js_id_.button["SELECT"] = 10;
        js_id_.button["START"] = 11;
    }
    else
    {
        cout << "Unsupported gamepad." << endl;
    }
}

void UnitreeSdk2Bridge::PrintSceneInformation()
{
    cout << endl;

    cout << "<<------------- Link ------------->> " << endl;
    for (int i = 0; i < mj_model_->nbody; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_BODY, i);
        if (name)
        {
            cout << "link_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Joint ------------->> " << endl;
    for (int i = 0; i < mj_model_->njnt; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
        if (name)
        {
            cout << "joint_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Actuator ------------->> " << endl;
    for (int i = 0; i < mj_model_->nu; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);
        if (name)
        {
            cout << "actuator_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Sensor ------------->> " << endl;
    int index = 0;
    // 多维传感器，输出第一维的index
    for (int i = 0; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (name)
        {
            cout << "sensor_index: " << index << ", "
                 << "name: " << name << ", "
                 << "dim: " << mj_model_->sensor_dim[i]
                 << endl;
        }
        index = index + mj_model_->sensor_dim[i];
    }
    cout << endl;
}

void UnitreeSdk2Bridge::CheckSensor()
{
    num_motor_ = mj_model_->nu;
    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    int my_actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "mor/joint1");
    if (my_actuator_id != -1)
    {
        idl_type_ = 2;
        printf("Get moying robot!\n");
        printf("idl_type_moying is %d !\n",idl_type_);
        return ;
    }

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "imu_quat") == 0)
        {
            have_imu_ = true;
        }
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
        if (strstr(name, "touch") != nullptr)
        {
            have_touch_sensor_ = true;
        }
    }

    if (num_motor_ > NUM_MOTOR_IDL_GO)
    {
        idl_type_ = 1; // unitree_hg
    }
    else
    {
        idl_type_ = 0; // unitree_go
    }
}

void UnitreeSdk2Bridge::GetWirelessRemote()
{
    js_->getState();
    wireless_remote_.btn.components.R1 = js_->button_[js_id_.button["RB"]];
    wireless_remote_.btn.components.L1 = js_->button_[js_id_.button["LB"]];
    wireless_remote_.btn.components.start = js_->button_[js_id_.button["START"]];
    wireless_remote_.btn.components.select = js_->button_[js_id_.button["SELECT"]];
    wireless_remote_.btn.components.R2 = (js_->axis_[js_id_.axis["RT"]] > 0);
    wireless_remote_.btn.components.L2 = (js_->axis_[js_id_.axis["LT"]] > 0);
    wireless_remote_.btn.components.F1 = 0;
    wireless_remote_.btn.components.F2 = 0;
    wireless_remote_.btn.components.A = js_->button_[js_id_.button["A"]];
    wireless_remote_.btn.components.B = js_->button_[js_id_.button["B"]];
    wireless_remote_.btn.components.X = js_->button_[js_id_.button["X"]];
    wireless_remote_.btn.components.Y = js_->button_[js_id_.button["Y"]];
    wireless_remote_.btn.components.up = (js_->axis_[js_id_.axis["DY"]] < 0);
    wireless_remote_.btn.components.right = (js_->axis_[js_id_.axis["DX"]] > 0);
    wireless_remote_.btn.components.down = (js_->axis_[js_id_.axis["DY"]] > 0);
    wireless_remote_.btn.components.left = (js_->axis_[js_id_.axis["DX"]] < 0);

    wireless_remote_.lx = double(js_->axis_[js_id_.axis["LX"]]) / max_value_;
    wireless_remote_.ly = -double(js_->axis_[js_id_.axis["LY"]]) / max_value_;
    wireless_remote_.rx = double(js_->axis_[js_id_.axis["RX"]]) / max_value_;
    wireless_remote_.ry = -double(js_->axis_[js_id_.axis["RY"]]) / max_value_;
}