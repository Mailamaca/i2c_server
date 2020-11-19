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
  auto handle_i2c_read =
    [this](
    const std::shared_ptr<i2c_interfaces::srv::I2cRead::Request> request,
    std::shared_ptr<i2c_interfaces::srv::I2cRead::Response> response
    ) -> void
    {
      RCLCPP_INFO(
        this->get_logger(), "Incoming request 'read': [slave: %d, register: %d]",
        request->slave, request->reg);
      
      // request data
      uint8_t slave = request->slave;
      uint8_t reg = request->reg;
      uint8_t length = request->length;
      
      // response data
      uint8_t* buffer = new uint8_t[length];
      bool ok = false;
      
      // select slave
      if (selectSlave(fd, slave)) {
        // Send the register to read from
        if (write(fd, &reg, 1) == 1) {
          // read data
          if (read(fd, buffer, length) == length) {           
            ok = true;
          }
        }      
      }
      
      std::vector<uint8_t> vectData;
      for (int i=0; i < length; i++) {
        vectData.push_back(buffer[i]);
      }      
      
            
      // populate response
      response->ok = ok;
      response->data = vectData;
    };
    
  auto handle_i2c_write =
    [this](
    const std::shared_ptr<i2c_interfaces::srv::I2cWrite::Request> request,
    std::shared_ptr<i2c_interfaces::srv::I2cWrite::Response> response
    ) -> void
    {
      RCLCPP_INFO(
        this->get_logger(), "Incoming request 'write': [slave: %d, register: %d]",
        request->slave, request->reg);
      
      // request data
      uint8_t slave = request->slave;
      uint8_t reg = request->reg;
      uint8_t length = request->length;
      uint8_t *buffer = request->data.data();
            
      // response data
      bool ok = false;
      
      // select slave
      if (selectSlave(fd, slave)) {
        // write data
        uint8_t* write_buffer = new uint8_t[length+2];
        write_buffer[0] = reg;
        for (int i=0; i<length; i++){
          write_buffer[i+1] = buffer[i];
        }         
        if (write(fd, write_buffer, length+1) == 1) {
          ok = true;
        }      
      }
      
      // populate response
      response->ok = ok;
    }; 

  srv_read = create_service<i2c_interfaces::srv::I2cRead>("i2c_read", handle_i2c_read);
  srv_write = create_service<i2c_interfaces::srv::I2cWrite>("i2c_write", handle_i2c_write);
  
}

bool I2CServer::selectSlave(int fd, int addr) {
    
    // initialize i2c if not initialized
    if (fd == -1) {
      int bus = 1;
      char device[16];
      sprintf(device, "/dev/i2c-%d", bus);
      if ((fd = open(device, O_RDWR)) < 0) {
        // ERROR Open port for reading and writing
        return false;
      }
    }
  
    // open slave
    int s;
    s = ioctl(fd, I2C_SLAVE, addr);
    if (s == -1) {
       // ERROR! no slave
       return false;
    }
    return true;
}

}  // namespace i2c_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(i2c_server::I2CServer)

