#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "moying_mor_hardware/moying_mor_ethercat_driver.hpp"

class MoyingBaseDriverTester: public rclcpp::Node
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(MoyingBaseDriverTester)

    MoyingBaseDriverTester (): Node("test_moying_ethercat_chassis_driver", rclcpp::NodeOptions{}), 
            moying_mor_chassis_()
    {
    }

    bool connect(const std::string & if_name)
    {
        nlohmann::json json;
        json["if_name"] = if_name;
        json["directions"] = {-1.0, 1.0,-1.0,1.0};
        json["slave_no"] = {1,2,3,4};
        json["reduction_ratios"] = {21,21,21,21};
        moying_mor_chassis_.configure(json.dump());
        if (moying_mor_chassis_.connect())
        {
            std::cout << "[info] Connect to Moying diff-drive vehicle." << std::endl;
            if (moying_mor_chassis_.enable())
            {
                std::cout << "[info] Enable Moying diff-drive vehicle." << std::endl;
                return true;
            }
            else
            {
                std::cout << "[error] Failed to enable Moying diff-drive vehicle." << std::endl;
                return false;
            }
        }
        else
        {
            std::cout << "[error] Failed to connect to Moying base." << std::endl;
            return false;
        }
    }

    void setWheelVelocities(const Eigen::VectorXd & target_velocities)
    {
        Eigen::VectorXd wheel_commands = Eigen::VectorXd::Zero(4);
        wheel_commands[0] = target_velocities[0];
        wheel_commands[1] = target_velocities[1];
        wheel_commands[2] = target_velocities[2];
        wheel_commands[3] = target_velocities[3];
        moying_mor_chassis_.setWheelCommands(wheel_commands);
    }

    Eigen::VectorXd getWheelVelocities()
    {
        Eigen::VectorXd wheel_states = moying_mor_chassis_.getWheelStates();
        Eigen::VectorXd wheel_velocities = Eigen::VectorXd::Zero(4);
        wheel_velocities[0] = wheel_states[0];
        wheel_velocities[1] = wheel_states[1];
        wheel_velocities[2] = wheel_states[2];
        wheel_velocities[3] = wheel_states[3];
        return wheel_velocities;
    }

private:
    moying_mor_ethercat_driver::MoyingMorEthercatDriver moying_mor_chassis_;

};

// 终端交互线程。
void commandLineInteraction(MoyingBaseDriverTester::SharedPtr driver_test_node);

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    MoyingBaseDriverTester::SharedPtr driver_test_node = std::make_shared<MoyingBaseDriverTester>();

    std::thread cli_thread(std::bind(commandLineInteraction, driver_test_node));
    cli_thread.detach();

    rclcpp::spin(driver_test_node);
    rclcpp::shutdown();
    return 0;
}
void commandLineInteraction(MoyingBaseDriverTester::SharedPtr driver_tester_node)
{
    while (rclcpp::ok())
    {
        std::cout << "[In]: ";
        std::string str_input;
        std::getline(std::cin, str_input);

        try
        {
            nlohmann::json json_input = nlohmann::json::parse(str_input);
            if ((!json_input.contains("command")) || (!json_input["command"].is_string()))
            {
                std::cout << "[Out]: Wrong input, cannot find 'command'." << std::endl;
                continue;
            }

            if (std::strcmp(json_input["command"].get<std::string>().c_str(), "connect") == 0)
            {
                if (!json_input.contains("if_name"))
                {
                    std::cout << "[Out]: Wrong 'connect' command, cannot find 'if_name'." << std::endl;
                    continue;
                }

                std::string if_name = json_input["if_name"];

                if (driver_tester_node->connect(if_name))
                    std::cout << "[Out]: Connect to Moying diff-drive vehicle." << std::endl;
                else
                    std::cout << "[Out]: Failed to connect to Moying diff-drive vehicle." << std::endl;

                continue;
            }
            else if (std::strcmp(json_input["command"].get<std::string>().c_str(), "set_wheel_velocities") == 0)
            {
                if (!json_input.contains("target_velocities"))
                {
                    std::cout << "[Out]: Wrong 'set_wheel_velocities' command, cannot find 'target_velocities'." << std::endl;
                    continue;
                }

                Eigen::VectorXd target_velocities = Eigen::VectorXd::Zero(4);
                for (int idx = 0; idx < target_velocities.size(); ++idx)
                    target_velocities[idx] = json_input["target_velocities"][idx];

                driver_tester_node->setWheelVelocities(target_velocities);

                continue;
            }
            else if (std::strcmp(json_input["command"].get<std::string>().c_str(), "get_wheel_velocities") == 0)
            {
                Eigen::VectorXd wheel_velocities = driver_tester_node->getWheelVelocities();
                std::cout << "[Out]: Wheel velocities: " << wheel_velocities.transpose() << std::endl;

                continue;
            }
            else
            {
                std::cout << "[Out]: Unsupported command." << std::endl;
                continue;
            }
        }
        catch (...)
        {
            std::cout << "[Out]: Cannot parse the input." << std::endl;
        }
    }
}