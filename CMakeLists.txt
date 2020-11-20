cmake_minimum_required(VERSION 3.5)
project(i2c_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(i2c_interfaces REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(i2c_server_component SHARED
  src/i2c_server_component.cpp)
target_compile_definitions(i2c_server_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(i2c_server_component
  "i2c_interfaces"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(i2c_server_component "i2c_server::I2CServer")
set(node_plugins "${node_plugins}i2c_server::I2CServer;$<TARGET_FILE:i2c_server_component>\n")

add_library(i2c_client_component SHARED
  src/i2c_client_component.cpp)
target_compile_definitions(i2c_client_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(i2c_client_component
  "i2c_interfaces"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(i2c_client_component "i2c_client::I2CClient")
set(node_plugins "${node_plugins}i2c_client::I2CClient;$<TARGET_FILE:i2c_client_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(linktime_composition
  src/linktime_composition.cpp)
set(libs
  i2c_server_component)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()

target_link_libraries(linktime_composition ${libs})
ament_target_dependencies(linktime_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")

add_executable(dlopen_composition
  src/dlopen_composition.cpp)
ament_target_dependencies(dlopen_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")

install(TARGETS
  i2c_server_component
  i2c_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  linktime_composition
  dlopen_composition
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
