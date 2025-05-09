// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#pragma once

#include <realtime_tools/realtime_buffer.h>

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "gazebo_ros2_control/gazebo_system_interface.hpp"

namespace usr_gazebo_ros2_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
class UsrGazeboSystemPrivate;

// These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class GazeboBridge : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Documentation Inherited
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh, gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info, sdf::ElementPtr sdf) override;

private:
  void registerJoints(
    const hardware_interface::HardwareInfo & hardware_info, gazebo::physics::ModelPtr parent_model);

  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info, gazebo::physics::ModelPtr parent_model);

  int parseJointIndex(const std::string & jointName);

  /// \brief Private data class
  std::unique_ptr<UsrGazeboSystemPrivate> dataPtr_;
};

}  // namespace usr_gazebo_ros2_control
