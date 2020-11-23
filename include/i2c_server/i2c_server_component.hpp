// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef COMPOSITION__I2C_SERVER_COMPONENT_HPP_
#define COMPOSITION__I2C_SERVER_COMPONENT_HPP_

#include "i2c_server/visibility_control.h"
#include "i2c_interfaces/srv/i2c_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace i2c_server
{

class I2CServer : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit I2CServer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<i2c_interfaces::srv::I2cCommand>::SharedPtr i2c_service;
  
  bool selectSlave(int addr);

  uint8_t write_buffer[256];
  uint8_t received_buffer[256];
  
  int fd = -1;
};

}  // namespace i2c_server

#endif  // COMPOSITION__I2C_SERVER_COMPONENT_HPP_
