cmake_minimum_required(VERSION 3.8)
project(uma_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) # add quando criando interfaces
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MinhaMsg.msg"
  DEPENDENCIES builtin_interfaces # ref para msgs de outros pacotes
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
