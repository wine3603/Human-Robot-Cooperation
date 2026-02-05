// Copyright (c) 2024, 
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "robotiq_ft_sensor_hardware_zombie/robotiq_ft_sensor_hardware_zombie.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotiq_ft_sensor_hardware_zombie
{
hardware_interface::CallbackReturn RobotiqFTSensorHardwareZombie::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // parameters
  use_fake_mode_ =
      info_.hardware_parameters["use_fake_mode"] == "True" || info_.hardware_parameters["use_fake_mode"] == "true";
  max_retries_ = std::stoi(info_.hardware_parameters["max_retries"]);
  read_rate_ = std::stoi(info_.hardware_parameters["read_rate"]);
  ftdi_id_ = info_.hardware_parameters["ftdi_id"];
  if (ftdi_id_.size() == 1)
  {
    ftdi_id_ = "";
  }
  if (info_.sensors.size() != 1)
  {
    RCLCPP_ERROR(logger_, "The Robotiq FT driver only supports one sensor. Number of sensors specified: %zu",
                 info_.sensors.size());
    return CallbackReturn::ERROR;
  }
  sensor_ = info_.sensors[0];
  if (sensor_.state_interfaces.size() != 6)
  {
    RCLCPP_ERROR(logger_, "The Robotiq FT driver expects 6 state interfaces. Number of state interfaces specified: %zu",
                 sensor_.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Parameter : use_fake_mode -> %d", use_fake_mode_);
  RCLCPP_INFO(logger_, "Parameter : max_retries -> %d", max_retries_);
  RCLCPP_INFO(logger_, "Parameter : read_rate -> %d", read_rate_);
  RCLCPP_INFO(logger_, "Parameter : ftdi_id -> %s", ftdi_id_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}
void RobotiqFTSensorHardwareZombie::read_background()
{
  if (!use_fake_mode_)
  {
    ret_ = sensor_state_machine();
    if (ret_ == -1)
    {
      wait_for_other_connection();
    }

    if (rq_sensor_get_current_state() == RQ_STATE_RUN)
    {
      strcpy(bufStream_, "");
      // auto msgStream = get_data();

      if (rq_state_got_new_message())
      {
        std::array<double, 6> sensor_reading_background{};
        sensor_reading_background[0] = rq_state_get_received_data(0);
        sensor_reading_background[1] = rq_state_get_received_data(1);
        sensor_reading_background[2] = rq_state_get_received_data(2);
        sensor_reading_background[3] = rq_state_get_received_data(3);
        sensor_reading_background[4] = rq_state_get_received_data(4);
        sensor_reading_background[5] = rq_state_get_received_data(5);

        sensor_readings_.writeFromNonRT(sensor_reading_background);
      }
    }
  }
}

hardware_interface::CallbackReturn RobotiqFTSensorHardwareZombie::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotiqFTSensorHardwareZombie::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(logger_, "Exporting State Interfaces");

  for (uint j = 0; j < sensor_.state_interfaces.size(); ++j)
  {
    RCLCPP_INFO(logger_, "Sensor %s state %s", sensor_.name.c_str(), sensor_.state_interfaces[j].name.c_str());
    state_interfaces.emplace_back(sensor_.name, sensor_.state_interfaces[j].name, &hw_sensor_states_[j]);
  }

  return state_interfaces;
}


hardware_interface::CallbackReturn RobotiqFTSensorHardwareZombie::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating ...please wait...");
  if (!use_fake_mode_)
  {
    // Connect
    if (!ftdi_id_.empty())
    {
      RCLCPP_INFO(logger_, "Trying to connect to a sensor at /dev/%s", ftdi_id_.c_str());
    }
    else
    {
      RCLCPP_INFO(logger_, "No device filename specified. Will attempt to discover Robotiq force torque sensor.");
    }
    // Connect
    // If we can't initialize, we return an error
    ret_ = sensor_state_machine();
    if (ret_ == -1)
    {
      wait_for_other_connection();
    }
    // Reads basic info on the sensor
    ret_ = sensor_state_machine();
    if (ret_ == -1)
    {
      wait_for_other_connection();
    }
    // Starts the stream
    ret_ = sensor_state_machine();
    if (ret_ == -1)
    {
      wait_for_other_connection();
    }
  }

  for (auto& value : hw_sensor_states_)
  {
    value = std::numeric_limits<double>::quiet_NaN();
  }
  sensor_readings_.writeFromNonRT(hw_sensor_states_);

  //=== ASYNC NODE
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=robotiq_ft_hardware_internal_" + info_.name });
  async_node_ = rclcpp::Node::make_shared("_", options);
  srv_zero_fts_ = async_node_->create_service<std_srvs::srv::Trigger>(
      "~/io_and_status_controller/zero_ftsensor",
      std::bind(&RobotiqFTSensorHardwareZombie::set_zero, this, std::placeholders::_1, std::placeholders::_2));

  timer_ = async_node_->create_wall_timer(std::chrono::milliseconds(read_rate_),
                                          std::bind(&RobotiqFTSensorHardwareZombie::read_background, this));

  node_thread_ = std::make_unique<std::thread>([&]() {
    executor_.add_node(async_node_);
    executor_.spin();
    executor_.remove_node(async_node_);
  });

  RCLCPP_INFO(logger_, "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn RobotiqFTSensorHardwareZombie::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating ...please wait...");
  //
  executor_.cancel();
  node_thread_->join();
  node_thread_.reset();
  srv_zero_fts_.reset();
  async_node_.reset();
  //
  // TODO DEACTIVATE RQ SENSOR
  //
  RCLCPP_INFO(logger_, "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqFTSensorHardwareZombie::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_mode_)
  {
    hw_sensor_states_[0] = 0.0;
    hw_sensor_states_[1] = 0.0;
    hw_sensor_states_[2] = 0.0;
    hw_sensor_states_[3] = 0.0;
    hw_sensor_states_[4] = 0.0;
    hw_sensor_states_[5] = 0.0;
  }
  else
  {
    hw_sensor_states_ = *(sensor_readings_.readFromRT());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robotiq_ft_sensor_hardware_zombie

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robotiq_ft_sensor_hardware_zombie::RobotiqFTSensorHardwareZombie, hardware_interface::SensorInterface)
