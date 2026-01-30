#include <vector>
#include <iostream>
#include <iomanip>

#include <soem/ethercat.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "baichuan_fieldbus_ethercat/ethercat_master.h"

// 分配和映射PDO，由硬件决定，此处是大族机械臂的EtherCAT通信协议。
int customCofiguration(uint16_t slave)
{
    int ret = 0;
    uint16_t map_1c12[3] = {0x0002, 0x1600, 0x1610};
    uint16_t map_1c13[3] = {0x0002, 0x1A00, 0x1A10};
    ret += ec_SDOwrite(slave, 0x1C12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    ret += ec_SDOwrite(slave, 0x1C13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

    return 0;
}

class EtherCATMasterTestNode : public rclcpp::Node
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(EtherCATMasterTestNode)

    EtherCATMasterTestNode() : Node("baichuan_fieldbus_ethercat_test_node", rclcpp::NodeOptions{}),
            ecat_master_()
    {
        ecat_master_.setCustomConfigurationHook(customCofiguration);

        this->declare_parameter<std::string>("if_name", "");
        this->get_parameter("if_name", if_name_);
        this->declare_parameter<int>("period_ms", 100);
        this->get_parameter("period_ms", period_ms_);

        if (!if_name_.empty())
        {
            // 连接机械臂。
            if (ecat_master_.init(if_name_))
            {
                ecat_master_.startWorking(1000);
                RCLCPP_INFO(get_logger(), "EtherCAT master started working via interface '%s'.", if_name_.c_str());
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to initialize EtherCAT master via interface '%s'.", if_name_.c_str());
                return;
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Parameter 'if_name' is empty.");
            return;
        }

        timer_ = this->create_wall_timer(std::chrono::duration<long double, std::milli>(period_ms_),
                std::bind(&EtherCATMasterTestNode::timer_callback, this));
    }

private:
    baichuan::fieldbus::EtherCATMaster ecat_master_;
    std::string if_name_;
    int period_ms_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        if (ecat_master_.isWorking())
        {
            std::cout << "===== IO Map Data =====" << std::endl;
            std::vector<uint8_t> data = ecat_master_.readIOMap(0, 384);
            for (std::size_t idx_i = 0; idx_i < 24; ++idx_i)
            {
                for (std::size_t idx_j = 0; idx_j < 16; ++idx_j)
                {
                    std::cout << std::setw(2) << std::setfill('0') << std::hex <<
                            (int)data[16*idx_i+idx_j] << " ";
                }

                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    EtherCATMasterTestNode::SharedPtr ecat_test_node = std::make_shared<EtherCATMasterTestNode>();
    rclcpp::spin(ecat_test_node);
    rclcpp::shutdown();
    return 0;
}
