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

#ifndef COMPOSITION__I2C_CLIENT_COMPONENT_HPP_
#define COMPOSITION__I2C_CLIENT_COMPONENT_HPP_

#include "i2c_server/visibility_control.h"
#include "i2c_interfaces/srv/i2c_read.hpp"
#include "i2c_interfaces/srv/i2c_write.hpp"
#include "rclcpp/rclcpp.hpp"

namespace i2c_client
{

class I2CClient : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit I2CClient(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<i2c_interfaces::srv::I2cRead>::SharedPtr client_read;
  rclcpp::Client<i2c_interfaces::srv::I2cWrite>::SharedPtr client_write;
  
  bool selectSlave(int fd, int addr);

  void on_timer();
  
  int fd = -1;

  int value;
};

}  // namespace i2c_client

#endif  // COMPOSITION__I2C_CLIENT_COMPONENT_HPP_
