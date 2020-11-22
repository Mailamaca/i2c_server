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

//ref: https://www.kernel.org/doc/Documentation/i2c/smbus-protocol
//ref: https://i2c.info/i2c-bus-specification


#include "i2c_server/i2c_client_component.hpp"

#include <cinttypes>
#include <iostream>
#include <cstring>
#include <memory>

extern "C" {
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <sys/ioctl.h>
  #include <stdio.h>
  #include <stdlib.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <unistd.h>
}

using namespace std::chrono_literals;

namespace i2c_client
{

I2CClient::I2CClient(const rclcpp::NodeOptions & options)
: Node("I2CClient", options)
{

  client_command = create_client<i2c_interfaces::srv::I2cCommand>("i2c_command");
  timer_ = create_wall_timer(10ms, std::bind(&I2CClient::on_timer, this));

  
}

void I2CClient::on_timer()
{
  if (!client_command->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = 0x09;
  request->reg = 3;
  request->length = 2;
  request->write = false;

  using ServiceResponseFuture =
    rclcpp::Client<i2c_interfaces::srv::I2cCommand>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {

      //RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get()->ok);

      uint8_t *buffer = future.get()->data_received.data();
      int16_t newValue;
      std::memcpy(&newValue, buffer, sizeof(newValue));
      if (newValue-value==1) {
        if (newValue % 1000 == 0) {
          RCLCPP_INFO(this->get_logger(), "newValue: %d",newValue);
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "value: %d, newValue: %d",value,newValue);
      }
      value = newValue;

      std::vector<uint8_t> vectData = future.get()->data_received;
      for (int i=0; i < vectData.size(); i++) {
        //RCLCPP_INFO(this->get_logger(), "bytes read: [%d] = %d",i,vectData.at(i));
      } 

    };
  auto future_result = client_command->async_send_request(request, response_received_callback);
}

}  // namespace i2c_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(i2c_client::I2CClient)

