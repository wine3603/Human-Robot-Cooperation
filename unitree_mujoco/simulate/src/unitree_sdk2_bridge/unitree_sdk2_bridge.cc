#include "unitree_sdk2_bridge.h"


UnitreeSdk2Bridge::UnitreeSdk2Bridge(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    CheckSensor();

    // ============================================================
    // 情况 A: 标准 Unitree 机器狗 (Go1 / B2 / H1)
    // ============================================================
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
    
    // ============================================================
    // 情况 B: 自定义机器人 (并行加载)
    // ============================================================
    
    // 1. 如果有 Moying ，初始化它的通信
    if (has_moying_)
    {
        // Cmd Subscribers
        MoyingMorCmd_suber_.reset(new ChannelSubscriber<moying::msg::dds_::MoyingMorCmd_>("moying/mor_cmd"));
        MoyingMorCmd_suber_->InitChannel(bind(&UnitreeSdk2Bridge::Moying_Mor_CmdHandler, this, placeholders::_1), 1);
        
        MoyingMcrCmd_suber_.reset(new ChannelSubscriber<moying::msg::dds_::MoyingMcrCmd_>("moying/mcr_cmd"));
        MoyingMcrCmd_suber_->InitChannel(bind(&UnitreeSdk2Bridge::Moying_Mcr_CmdHandler, this, placeholders::_1), 1);

        // State Publisher
        moying_state_puber_.reset(new ChannelPublisher<moying::msg::dds_::MoyingState_>("moying/state"));
        moying_state_puber_->InitChannel();

        // Thread: Moying 状态发布线程
        // 注意：这里复用了 lowStatePuberThreadPtr，如果和标准狗不冲突就没问题
        lowStatePuberThreadPtr = CreateRecurrentThreadEx("moying_state", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishMoyingState, this);
        
        printf("[UnitreeBridge] Moying interfaces initialized.\n");
    }

    // 2. 如果有 PR2，初始化它的通信 (注意这里是独立的 if，不是 else if)
    if (has_pr2_)
    {
        // Cmd Subscriber
        Pr2Cmd_suber_.reset(new ChannelSubscriber<pr2cmd::msg::dds_::PR2ActuatorCmd >("pr2/cmd"));
        Pr2Cmd_suber_->InitChannel(bind(&UnitreeSdk2Bridge::PR2CmdHandler, this, placeholders::_1), 1);

        // State Publisher
        pr2_state_puber_.reset(new ChannelPublisher<pr2cmd::msg::dds_::PR2SensorState >("pr2/state"));
        pr2_state_puber_->InitChannel();

        // Thread: PR2 状态发布线程
        // 【关键】必须使用新的线程指针 (pr2StatePuberThreadPtr)，不能覆盖 Moying 的指针！
        // 请确保你在 .h 文件里定义了 RecurrentThread* pr2StatePuberThreadPtr;
        pr2StatePuberThreadPtr = CreateRecurrentThreadEx("pr2_state", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishPR2State, this);
        
        printf("[UnitreeBridge] PR2 interfaces initialized.\n");
    }

    // ============================================================
    // 公共部分 (HighState / Wireless)
    // ============================================================
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

    // --- 静态缓存：只在第一次收到消息时查找 ID ---
    static std::vector<int> arm_ids;
    static std::vector<int> wheel_ids;
    static int gripper_id = -2; // -2 代表未初始化

    if (gripper_id == -2) { 
        // [初始化] 查找 MOR 的 Actuator ID
        std::cout << "[UnitreeBridge] Initializing MOR Actuator IDs..." << std::endl;

        // 1. 关节
        for (int i = 1; i <= 6; ++i) {
            std::string name = "mor/joint" + std::to_string(i);
            int id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
            arm_ids.push_back(id);
            if(id == -1) {
                std::cout << "[UnitreeBridge] MOR Actuator NOT found: " << name << " (If only MCR is loaded, this is normal)" << std::endl;
            }
        }

        // 2. 轮子 (必须是 FL, FR, BL, BR)
        const char* w_names[4] = {"mor/fl_wheel", "mor/fr_wheel", "mor/bl_wheel", "mor/br_wheel"};
        
        for (int i = 0; i < 4; ++i) {
            int id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, w_names[i]);
            wheel_ids.push_back(id);
            if (id != -1) {
                std::cout << "  MOR Wheel " << i << " (" << w_names[i] << ") -> ID " << id << std::endl;
            }
        }

        // 3. 夹爪
        gripper_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "mor/robitiq_joint");
        
        std::cout << "[UnitreeBridge] MOR Cmd IDs initialized." << std::endl;
    }

    // --- 执行控制 (如果 ID 为 -1，说明该机器人不在场景中，直接跳过不写) ---

    // 1. 机械臂力矩
    for (int i = 0; i < 6; ++i) {
        if (arm_ids[i] != -1) mj_data_->ctrl[arm_ids[i]] = cmd->mor_joint_tau()[i];
    }

    // 2. 轮子速度
    for (int i = 0; i < 4; ++i) {
        if (wheel_ids[i] != -1) mj_data_->ctrl[wheel_ids[i]] = cmd->mor_wheel_velocity()[i];
    }

    // 3. 夹爪位置
    if (gripper_id != -1) {
        mj_data_->ctrl[gripper_id] = cmd->mor_robotiq_position();
    }
}



void UnitreeSdk2Bridge::Moying_Mcr_CmdHandler(const void *msg)
{
    const moying::msg::dds_::MoyingMcrCmd_ *cmd = static_cast<const moying::msg::dds_::MoyingMcrCmd_ *>(msg);
    if (!mj_data_ || !mj_model_) return;

    // --- 静态缓存 ---
    static std::vector<int> left_arm_ids;
    static std::vector<int> right_arm_ids;
    static std::vector<int> wheel_ids;
    static int left_grip_id = -2;
    static int right_grip_id = -2;

    if (left_grip_id == -2) { // 初始化
        std::cout << "[UnitreeBridge] Initializing MCR Actuator IDs..." << std::endl;

        // 1. 左臂
        for (int i = 1; i <= 6; ++i) {
            std::string name = "mcr/left_joint" + std::to_string(i);
            int id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
            left_arm_ids.push_back(id);
        }
        // 2. 右臂
        for (int i = 1; i <= 6; ++i) {
            std::string name = "mcr/right_joint" + std::to_string(i);
            int id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
            right_arm_ids.push_back(id);
        }
        
        // 3. 轮子 (修正顺序！必须是 FL, FR, BL, BR)
        // 之前错误的顺序: {"mcr/fr_wheel", "mcr/fl_wheel", ...} 导致左右颠倒
        const char* w_names[4] = {"mcr/fl_wheel", "mcr/fr_wheel", "mcr/bl_wheel", "mcr/br_wheel"};
        for (int i = 0; i < 4; ++i) {
            int id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, w_names[i]);
            if (id != -1) {
                // std::cout << "  Wheel " << i << " (" << w_names[i] << ") -> ID " << id << std::endl;
            }
            wheel_ids.push_back(id);
        }

        // 4. 夹爪
        left_grip_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "mcr/left_robitiq_joint");
        right_grip_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "mcr/right_robitiq_joint");
        
        std::cout << "[UnitreeBridge] MCR Cmd IDs initialized." << std::endl;
    }

    // --- 执行控制 ---

    // 1. 左臂
    for (int i = 0; i < 6; ++i) {
        if (left_arm_ids[i] != -1) mj_data_->ctrl[left_arm_ids[i]] = cmd->mcrl_joint_tau()[i];
    }
    // 2. 右臂
    for (int i = 0; i < 6; ++i) {
        if (right_arm_ids[i] != -1) mj_data_->ctrl[right_arm_ids[i]] = cmd->mcrr_joint_tau()[i];
    }
    // 3. 轮子
    for (int i = 0; i < 4; ++i) {
        if (wheel_ids[i] != -1) mj_data_->ctrl[wheel_ids[i]] = cmd->mcr_wheel_velocity()[i];
    }
    // 4. 夹爪
    if (left_grip_id != -1) mj_data_->ctrl[left_grip_id] = cmd->mcr_robotiqleft_position();
    if (right_grip_id != -1) mj_data_->ctrl[right_grip_id] = cmd->mcr_robotiqright_position();
}



void UnitreeSdk2Bridge::PR2CmdHandler(const void* msg)
{
    const pr2cmd::msg::dds_::PR2ActuatorCmd* cmd =
        static_cast<const pr2cmd::msg::dds_::PR2ActuatorCmd*>(msg);

    if (!mj_data_ || !mj_model_) return;

    // =========================================================
    // 1. 静态缓存 & 初始化逻辑
    // =========================================================
    static bool is_initialized = false;
    static bool pr2_is_loaded = false;      // PR2 是否存在

    // 定义 ID 缓存变量
    static int r_gripper_id = -1, l_gripper_id = -1;
    static int head_pan_id = -1, head_tilt_id = -1, laser_tilt_id = -1;
    static int torso_id = -1;
    static std::vector<int> r_arm_ids;
    static std::vector<int> l_arm_ids;
    static std::vector<int> caster_ids;
    static std::vector<int> wheel_ids;

    if (!is_initialized) {
        is_initialized = true; // 锁住

        // --- A. 核心检测：检查躯干关节是否存在 ---
        // 只要这个找不到，说明整个 PR2 根本没加载
        int check_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "torso_lift_tau");

        if (check_id == -1) {
            // 情况 1: PR2 未加载
            pr2_is_loaded = false;
            std::cout << "[UnitreeBridge] PR2 robot NOT detected in scene. Skipping PR2 control." << std::endl;
        } 
        else {
            // 情况 2: PR2 已加载 -> 正常查找所有 30+ 个 ID
            pr2_is_loaded = true;
            std::cout << "[UnitreeBridge] PR2 robot detected. Initializing IDs..." << std::endl;

            // 辅助 Lambda: 查找并只在 PR2 存在时报错
            auto get_id = [&](const std::string& name) -> int {
                int id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
                if (id == -1) {
                    // 既然已经确认 PR2 在场，如果还找不到某个关节，那就是 XML 写错了，必须报错
                    std::cerr << "[Warning] PR2 Actuator not found: " << name << std::endl;
                }
                return id;
            };

            // --- B. 躯干 ---
            torso_id = check_id; // 刚才已经查过了

            // --- C. 夹爪 & 头部 ---
            r_gripper_id  = get_id("r_gripper_pos");
            l_gripper_id  = get_id("l_gripper_pos");
            head_pan_id   = get_id("head_pan_pos");
            head_tilt_id  = get_id("head_tilt_pos");
            laser_tilt_id = get_id("laser_tilt_pos");

            // --- D. 机械臂 (7 DoF) ---
            std::vector<std::string> arm_suffixes = {
                "shoulder_pan_tau", "shoulder_lift_tau", "upper_arm_roll_tau", "elbow_flex_tau", 
                "forearm_roll_tau", "wrist_flex_tau", "wrist_roll_tau"
            };
            for (const auto& suffix : arm_suffixes) {
                r_arm_ids.push_back(get_id("r_" + suffix));
                l_arm_ids.push_back(get_id("l_" + suffix));
            }

            // --- E. 舵轮 (Steer) FL, FR, BL, BR ---
            caster_ids.push_back(get_id("fl_caster_steer"));
            caster_ids.push_back(get_id("fr_caster_steer"));
            caster_ids.push_back(get_id("bl_caster_steer"));
            caster_ids.push_back(get_id("br_caster_steer"));

            // --- F. 轮速 (Wheel Velocity) ---
            wheel_ids.push_back(get_id("fl_caster_l_wheel_vel"));
            wheel_ids.push_back(get_id("fl_caster_r_wheel_vel"));
            wheel_ids.push_back(get_id("fr_caster_l_wheel_vel"));
            wheel_ids.push_back(get_id("fr_caster_r_wheel_vel"));
            wheel_ids.push_back(get_id("bl_caster_l_wheel_vel"));
            wheel_ids.push_back(get_id("bl_caster_r_wheel_vel"));
            wheel_ids.push_back(get_id("br_caster_l_wheel_vel"));
            wheel_ids.push_back(get_id("br_caster_r_wheel_vel"));

            std::cout << "[UnitreeBridge] PR2 Actuator IDs cached successfully." << std::endl;
        }
    }

    // --- 2. 快速拦截 ---
    // 如果没有 PR2，直接返回，不再执行后续复杂的赋值
    if (!pr2_is_loaded) return;


    // =========================================================
    // 3. 执行写入 (只有 PR2 在场时才会执行到这里)
    // =========================================================

    // 1. Gripper + Head
    if (r_gripper_id != -1) mj_data_->ctrl[r_gripper_id] = cmd->r_gripper_pos();
    if (l_gripper_id != -1) mj_data_->ctrl[l_gripper_id] = cmd->l_gripper_pos();
    if (head_pan_id  != -1) mj_data_->ctrl[head_pan_id]  = cmd->head_pan_pos();
    if (head_tilt_id != -1) mj_data_->ctrl[head_tilt_id] = cmd->head_tilt_pos();
    if (laser_tilt_id!= -1) mj_data_->ctrl[laser_tilt_id]= cmd->laser_tilt_pos();

    // 2. Caster Steer (4个)
    if (caster_ids[0] != -1) mj_data_->ctrl[caster_ids[0]] = cmd->fl_caster_steer();
    if (caster_ids[1] != -1) mj_data_->ctrl[caster_ids[1]] = cmd->fr_caster_steer();
    if (caster_ids[2] != -1) mj_data_->ctrl[caster_ids[2]] = cmd->bl_caster_steer();
    if (caster_ids[3] != -1) mj_data_->ctrl[caster_ids[3]] = cmd->br_caster_steer();

    // 3. Wheel Velocity (8个)
    if (wheel_ids[0] != -1) mj_data_->ctrl[wheel_ids[0]] = cmd->fl_caster_l_wheel_vel();
    if (wheel_ids[1] != -1) mj_data_->ctrl[wheel_ids[1]] = cmd->fl_caster_r_wheel_vel();
    if (wheel_ids[2] != -1) mj_data_->ctrl[wheel_ids[2]] = cmd->fr_caster_l_wheel_vel();
    if (wheel_ids[3] != -1) mj_data_->ctrl[wheel_ids[3]] = cmd->fr_caster_r_wheel_vel();
    if (wheel_ids[4] != -1) mj_data_->ctrl[wheel_ids[4]] = cmd->bl_caster_l_wheel_vel();
    if (wheel_ids[5] != -1) mj_data_->ctrl[wheel_ids[5]] = cmd->bl_caster_r_wheel_vel();
    if (wheel_ids[6] != -1) mj_data_->ctrl[wheel_ids[6]] = cmd->br_caster_l_wheel_vel();
    if (wheel_ids[7] != -1) mj_data_->ctrl[wheel_ids[7]] = cmd->br_caster_r_wheel_vel();

    // 4. Torso
    if (torso_id != -1) mj_data_->ctrl[torso_id] = cmd->torso_lift_tau();

    // 5. Arms (Right & Left)
    for (int i = 0; i < 7; ++i) {
        if (r_arm_ids[i] != -1) mj_data_->ctrl[r_arm_ids[i]] = cmd->r_arm_tau()[i];
        if (l_arm_ids[i] != -1) mj_data_->ctrl[l_arm_ids[i]] = cmd->l_arm_tau()[i];
    }
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

    // lambda helper: 通过名字查找传感器数据的辅助函数
    auto get_sensor_data = [&](const std::string& name, int offset = 0) -> double {
        int id = mj_name2id(mj_model_, mjOBJ_SENSOR, name.c_str());
        if (id == -1) {
            // 如果找不到名字，打印一次错误信息（避免刷屏），返回0
            static bool printed_err = false;
            if (!printed_err) {
                // std::cerr << "[Error] Sensor not found: " << name << std::endl;
                // printed_err = true; // Uncomment to print only once
            }
            return 0.0;
        }
        return mj_data_->sensordata[mj_model_->sensor_adr[id] + offset];
    };

    // --------------------------------------------------------------
    // 1. 读取 MOR 机械臂状态 (Joint Position, Velocity, Torque)
    // --------------------------------------------------------------
    for (int i = 0; i < 6; ++i) {
        std::string j_name = "mor/joint" + std::to_string(i + 1);
        moying_state_.mor_joint_position()[i] = get_sensor_data(j_name + "_p");
        moying_state_.mor_joint_velocity()[i] = get_sensor_data(j_name + "_v");
        moying_state_.mor_joint_torque()[i]   = get_sensor_data(j_name + "_t");
    }

    // --------------------------------------------------------------
    // 2. 读取 MCR 左臂状态
    // --------------------------------------------------------------
    for (int i = 0; i < 6; ++i) {
        std::string j_name = "mcr/left_joint" + std::to_string(i + 1);
        moying_state_.mcrl_joint_position()[i] = get_sensor_data(j_name + "_p");
        moying_state_.mcrl_joint_velocity()[i] = get_sensor_data(j_name + "_v");
        moying_state_.mcrl_joint_torque()[i]   = get_sensor_data(j_name + "_t");
    }

    // --------------------------------------------------------------
    // 3. 读取 MCR 右臂状态
    // --------------------------------------------------------------
    for (int i = 0; i < 6; ++i) {
        std::string j_name = "mcr/right_joint" + std::to_string(i + 1);
        moying_state_.mcrr_joint_position()[i] = get_sensor_data(j_name + "_p");
        moying_state_.mcrr_joint_velocity()[i] = get_sensor_data(j_name + "_v");
        moying_state_.mcrr_joint_torque()[i]   = get_sensor_data(j_name + "_t");
    }

    // --------------------------------------------------------------
    // 4. 读取基座位姿 (Base Pos/Quat)
    // --------------------------------------------------------------
    // MOR Base
    for (int i = 0; i < 3; ++i) moying_state_.mor_base_pos()[i]  = get_sensor_data("mor/base_pos", i);
    for (int i = 0; i < 4; ++i) moying_state_.mor_base_quat()[i] = get_sensor_data("mor/base_quat", i);

    // MCR Base
    for (int i = 0; i < 3; ++i) moying_state_.mcr_base_pos()[i]  = get_sensor_data("mcr/base_pos", i);
    for (int i = 0; i < 4; ++i) moying_state_.mcr_base_quat()[i] = get_sensor_data("mcr/base_quat", i);

    // Desk Base (注意：如果在 scene.xml 里没有补回 desk 传感器，这里会读到 0，但不会报错)
    for (int i = 0; i < 3; ++i) moying_state_.desk_base_pos()[i]  = get_sensor_data("desk/base_pos", i);
    for (int i = 0; i < 4; ++i) moying_state_.desk_base_quat()[i] = get_sensor_data("desk/base_quat", i);

    // --------------------------------------------------------------
    // 5. 读取力/力矩传感器 (Force/Torque)
    // --------------------------------------------------------------
    for (int i = 0; i < 3; ++i) moying_state_.mor_force()[i]    = get_sensor_data("mor/force", i);
    for (int i = 0; i < 3; ++i) moying_state_.mor_torque()[i]   = get_sensor_data("mor/torque", i);
    
    for (int i = 0; i < 3; ++i) moying_state_.right_force()[i]  = get_sensor_data("mcr/right_force", i);
    for (int i = 0; i < 3; ++i) moying_state_.right_torque()[i] = get_sensor_data("mcr/right_torque", i);
    
    for (int i = 0; i < 3; ++i) moying_state_.left_force()[i]   = get_sensor_data("mcr/left_force", i);
    for (int i = 0; i < 3; ++i) moying_state_.left_torque()[i]  = get_sensor_data("mcr/left_torque", i);

    // --------------------------------------------------------------
    // 6. 读取轮子状态 (Wheels)
    // --------------------------------------------------------------
    // 这里为了防止名字拼写错误，最好一个个写，或者确保 xml 命名极度规范
    // MOR Wheels
    moying_state_.mor_wheel_velocity()[0] = get_sensor_data("mor/fr_wheel_v");
    moying_state_.mor_wheel_velocity()[1] = get_sensor_data("mor/fl_wheel_v");
    moying_state_.mor_wheel_velocity()[2] = get_sensor_data("mor/br_wheel_v");
    moying_state_.mor_wheel_velocity()[3] = get_sensor_data("mor/bl_wheel_v");

    moying_state_.mor_wheel_position()[0] = get_sensor_data("mor/fr_wheel_p");
    moying_state_.mor_wheel_position()[1] = get_sensor_data("mor/fl_wheel_p");
    moying_state_.mor_wheel_position()[2] = get_sensor_data("mor/br_wheel_p");
    moying_state_.mor_wheel_position()[3] = get_sensor_data("mor/bl_wheel_p");

    // MCR Wheels
    moying_state_.mcr_wheel_velocity()[0] = get_sensor_data("mcr/fr_wheel_v");
    moying_state_.mcr_wheel_velocity()[1] = get_sensor_data("mcr/fl_wheel_v");
    moying_state_.mcr_wheel_velocity()[2] = get_sensor_data("mcr/br_wheel_v");
    moying_state_.mcr_wheel_velocity()[3] = get_sensor_data("mcr/bl_wheel_v");

    moying_state_.mcr_wheel_position()[0] = get_sensor_data("mcr/fr_wheel_p");
    moying_state_.mcr_wheel_position()[1] = get_sensor_data("mcr/fl_wheel_p");
    moying_state_.mcr_wheel_position()[2] = get_sensor_data("mcr/br_wheel_p");
    moying_state_.mcr_wheel_position()[3] = get_sensor_data("mcr/bl_wheel_p");

    // --------------------------------------------------------------
    // 7. 读取夹爪状态 (Grippers)
    // --------------------------------------------------------------
    moying_state_.mor_robotiq_position() = get_sensor_data("mor/robitiq_joint");
    moying_state_.mcr_robotiqleft_position() = get_sensor_data("mcr/left_robitiq_joint");
    moying_state_.mcr_robotiqright_position() = get_sensor_data("mcr/right_robitiq_joint");




    // 发布消息
    moying_state_puber_->Write(moying_state_);
}




void UnitreeSdk2Bridge::PublishPR2State()
{
    if (!mj_data_ || !mj_model_) return;

    // --- 1. 辅助函数：通过名字找数据，找不到返回 0.0 ---
    auto get_sensor_data = [&](const std::string& name, int offset = 0) -> double {
        int id = mj_name2id(mj_model_, mjOBJ_SENSOR, name.c_str());
        if (id == -1) {
            // [可选] 调试时打开，平时注释掉以免刷屏
            // std::cerr << "[Warning] PR2 Sensor not found: " << name << std::endl;
            return 0.0;
        }
        return mj_data_->sensordata[mj_model_->sensor_adr[id] + offset];
    };

    // --- 2. 躯干升降 (Torso) ---
    pr2_state_.torso_lift_q()  = get_sensor_data("torso_lift_q");
    pr2_state_.torso_lift_dq() = get_sensor_data("torso_lift_dq");

    // --- 3. 机械臂 (Arms) ---
    // 定义关节后缀列表，确保顺序与 PR2 运动学链一致
    // 顺序: 肩旋转 -> 肩升降 -> 上臂旋转 -> 肘弯曲 -> 前臂旋转 -> 腕弯曲 -> 腕旋转
    const std::vector<std::string> arm_suffixes = {
        "shoulder_pan", "shoulder_lift", "upper_arm_roll", "elbow_flex", 
        "forearm_roll", "wrist_flex", "wrist_roll"
    };

    for (int i = 0; i < 7; ++i) {
        // 右臂 (r_)
        std::string r_name = "r_" + arm_suffixes[i];
        pr2_state_.r_arm_q()[i]  = get_sensor_data(r_name + "_q");
        pr2_state_.r_arm_dq()[i] = get_sensor_data(r_name + "_dq");

        // 左臂 (l_)
        std::string l_name = "l_" + arm_suffixes[i];
        pr2_state_.l_arm_q()[i]  = get_sensor_data(l_name + "_q");
        pr2_state_.l_arm_dq()[i] = get_sensor_data(l_name + "_dq");
    }

    // --- 4. 移动底盘 (Casters & Wheels) ---
    // 定义轮子方位前缀，顺序通常是: FL, FR, BL, BR
    const std::vector<std::string> caster_prefixes = {"fl", "fr", "bl", "br"};

    for (int i = 0; i < 4; ++i) {
        std::string prefix = caster_prefixes[i] + "_caster"; // e.g. "fl_caster"

        // 舵轮转向 (Rotation)
        pr2_state_.caster_rotation_q()[i] = get_sensor_data(prefix + "_rotation_q");

        // 左轮 (L Wheel)
        pr2_state_.caster_l_wheel_q()[i]  = get_sensor_data(prefix + "_l_wheel_q");
        pr2_state_.caster_l_wheel_dq()[i] = get_sensor_data(prefix + "_l_wheel_dq");

        // 右轮 (R Wheel)
        pr2_state_.caster_r_wheel_q()[i]  = get_sensor_data(prefix + "_r_wheel_q");
        pr2_state_.caster_r_wheel_dq()[i] = get_sensor_data(prefix + "_r_wheel_dq");
    }

    // --- 5. 基座与环境 (Base & Desk) ---
    // 使用 offset 读取多维数据 (pos=3, quat=4)
    
    // PR2 本体基座真值
    for (int i = 0; i < 3; ++i) pr2_state_.pr2_base_pos()[i]  = get_sensor_data("pr2/base_pos", i);
    for (int i = 0; i < 4; ++i) pr2_state_.pr2_base_quat()[i] = get_sensor_data("pr2/base_quat", i);

    // 桌子位置 (注意：如果在 scene.xml 里没加载桌子，这里会读到 0，不会报错)
    for (int i = 0; i < 3; ++i) pr2_state_.desk_base_pos()[i]  = get_sensor_data("desk/base_pos", i);
    for (int i = 0; i < 4; ++i) pr2_state_.desk_base_quat()[i] = get_sensor_data("desk/base_quat", i);

    // --- 6. 力/力矩传感器 (Force/Torque) ---
    for (int i = 0; i < 3; ++i) pr2_state_.right_force()[i]  = get_sensor_data("pr2/right_force", i);
    for (int i = 0; i < 3; ++i) pr2_state_.right_torque()[i] = get_sensor_data("pr2/right_torque", i);
    for (int i = 0; i < 3; ++i) pr2_state_.left_force()[i]   = get_sensor_data("pr2/left_force", i);
    for (int i = 0; i < 3; ++i) pr2_state_.left_torque()[i]  = get_sensor_data("pr2/left_torque", i);

    // 发送数据
    pr2_state_puber_->Write(pr2_state_);
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

    // 初始化标志位
    has_moying_ = false;
    has_pr2_ = false;
    idl_type_ = 0; // 默认为 unitree_go，如果有自定义机器人则覆盖

    // --------------------------------------------------------
    // 1. 检查 Moying (MOR / MCR)
    // --------------------------------------------------------
    int mor_check = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "mor/joint1");
    int mcr_check = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "mcr/left_joint1");

    if (mor_check != -1 || mcr_check != -1)
    {
        has_moying_ = true;
        printf("Get moying robot! (Detected: %s)\n", (mor_check != -1) ? "MOR/Both" : "MCR");
    }

    // --------------------------------------------------------
    // 2. 检查 PR2
    // --------------------------------------------------------
    // 注意：不要用 return 提前退出，继续检查！
    int pr2_check = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "torso_lift_tau"); // 建议用 torso，更稳定
    if (pr2_check == -1) pr2_check = mj_name2id(mj_model_, mjOBJ_ACTUATOR, "laser_tilt_pos"); // 兼容你之前的写法

    if (pr2_check != -1)
    {
        has_pr2_ = true;
        printf("Get pr2 robot!\n");
    }

    // --------------------------------------------------------
    // 3. 决策逻辑
    // --------------------------------------------------------
    // 如果检测到了任意自定义机器人，就不再作为标准 Unitree 狗处理
    if (has_moying_ || has_pr2_)
    {
        idl_type_ = 99; // 设置为一个特殊值，表示"自定义混合模式"
        return; 
    }

    // --------------------------------------------------------
    // 4. 标准 Unitree 狗检测 (只有没检测到上面的机器人时才跑这里)
    // --------------------------------------------------------
    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "imu_quat") == 0) have_imu_ = true;
        if (strcmp(name, "frame_pos") == 0) have_frame_sensor_ = true;
        if (strstr(name, "touch") != nullptr) have_touch_sensor_ = true;
    }

    if (num_motor_ > NUM_MOTOR_IDL_GO) idl_type_ = 1; // unitree_hg
    else idl_type_ = 0; // unitree_go
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