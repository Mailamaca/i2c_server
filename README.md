# encoders_interface
ROS Node to interact with encoders

## Initialize ROS2 package

1. Create ros2 C++ package

    ```bash
    ros2 pkg create --build-type ament_cmake --node-name encoders_interface encoders_interface
    ```

2. Build package

    ```bash
    colcon build --packages-select encoders_interface
    ```

3. Source the setup file

    ```bash
    . install/setup.bash
    ```

4. Use the package

    ```bash
    ros2 run encoders_interface encoders_interface
    ```

5. Customize package.xml


## I2C

it needs wiringPi installed in /usr/local/lib

1. git clone https://github.com/WiringPi/WiringPi
2. cd WiringPi
3. sudo ./build

## Hardware

This package interact with an ESP32 board installed on the Maile Vehicle.
The communication is made on I2C where the board has been setup as a SLAVE with ID:0x09.
After each request the boards response with 10 bytes, 5x uint16_t values representing the 5 "delta tic" of each encoder from the last call.
the encoders order is:
1. spur gear motor encoder (high gear ratio = 1:5.22, low gear ratio  = 1:14.45)
2. front right wheel encoder
3. rear right wheel encoder
4. rear left wheel encoder
5. front left wheel encoder

**NOTES**
- wheel radious = 0.559 meter
- tick for wheel / round = 12
- tick at spur gear / round = 15



# Copyright and License

This software contains code licensed as described in LICENSE.