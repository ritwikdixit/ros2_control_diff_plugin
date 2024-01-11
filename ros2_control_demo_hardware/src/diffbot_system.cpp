// Copyright 2021 ros2_control Development Team
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

#include "ros2_control_demo_hardware/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{
hardware_interface::return_type DiffBotSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");
  arduino_.sendEmptyMsg();
  arduino_.setPidValues(30, 20, 0, 100);
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{
  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write()
{
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.setMotorValues(
    l_wheel_.cmd / l_wheel_.rads_per_count, r_wheel_.cmd / r_wheel_.rads_per_count);

  return return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)
