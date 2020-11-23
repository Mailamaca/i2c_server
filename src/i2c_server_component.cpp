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


#include "i2c_server/i2c_server_component.hpp"

#include <cinttypes>
#include <iostream>
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


namespace i2c_server
{

I2CServer::I2CServer(const rclcpp::NodeOptions & options)
: Node("I2CServer", options)
{
  auto handle_i2c_command =
    [this](
    const std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request,
    std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response
    ) -> void
    {
      /*RCLCPP_INFO(this->get_logger(),
        "Incoming request 'command': \
        [slave: %d, \
        register: %d, \
        length: %d, \
        write_cmd: %b",
        request->slave,
        request->reg,
        request->length,
        request->write);
      
      if (request->write && request->length > 1) {
        RCLCPP_INFO(this->get_logger(),
        "[data0: %d, \
        data1: %d]",
        request->data_to_send.at(0),
        request->data_to_send.at(1));
      }*/
      
      // request data
      uint8_t slave = request->slave;
      uint8_t reg = request->reg;
      uint8_t length = request->length;
      bool write_cmd = request->write;
      std::vector<uint8_t> data_to_send = request->data_to_send;

      // select slave
      if (!selectSlave(slave)) {
        RCLCPP_WARN(
          this->get_logger(),
          "error on selectSlave");
        response->ok = false;
        return;      
      }

      if (write_cmd) {
        // write data
        int wlength = length+1;
        write_buffer[0] = reg;
        for (int i=0; i<length; i++){
          write_buffer[i+1] = data_to_send.at(i);
        }     

        /*if (request->write && request->length > 1) {
          RCLCPP_INFO(this->get_logger(),
          "[data0: %d, \
          data1: %d,\
          data2: %d]",
          write_buffer[0],
          write_buffer[1],
          write_buffer[2]);
        }*/

        if (write(fd, write_buffer, wlength) != wlength) {
          RCLCPP_WARN(
            this->get_logger(),
            "error on write");
          response->ok = false;
          return; 
        }
        // ok
        response->ok = true;

      } else {
        // Send the register to read from
        if (write(fd, &reg, 1) != 1) {
          RCLCPP_WARN(
            this->get_logger(),
            "error on write");
          response->ok = false;
          return;
        }
        // read data
        if (read(fd, received_buffer, length) != length) {
          RCLCPP_WARN(
            this->get_logger(),
            "error on read");
          response->ok = false;
          return;
        }
        // copy data received to vector
        std::vector<uint8_t> vectData;
        for (int i=0; i < length; i++) {
          vectData.push_back(received_buffer[i]);
          //RCLCPP_INFO(this->get_logger(), "bytes read: [%d] = %d",i,data_received[i]);
        }
        // ok
        response->data_received = vectData;
        response->ok = true;
      } 
    };  

  i2c_service = create_service<i2c_interfaces::srv::I2cCommand>("i2c_command", handle_i2c_command);
  
}

bool I2CServer::selectSlave(int addr) {
    
    // initialize i2c if not initialized
    if (fd == -1) {
      int bus = 1;
      char device[16];
      sprintf(device, "/dev/i2c-%d", bus);
      if ((fd = open(device, O_RDWR)) < 0) {
        RCLCPP_INFO(this->get_logger(), "error on open: %d", fd);
        // ERROR Open port for reading and writing
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "open ok, fd=%d", fd);
    }
  
    // open slave
    int s;
    s = ioctl(fd, I2C_SLAVE, addr);
    if (s == -1) {
      RCLCPP_INFO(this->get_logger(), "error on ioctl: %d", s);
       // ERROR! no slave
       return false;
    }
    //RCLCPP_INFO(this->get_logger(), "ioctl ok, s= %d", s);
    return true;
}

}  // namespace i2c_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(i2c_server::I2CServer)

