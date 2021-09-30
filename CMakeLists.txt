cmake_minimum_required(VERSION 3.5)
project(monitoring)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

## Include messages
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(PkgConfig REQUIRED)

include_directories(include/${PROJECT_NAME})



# this builds a component and a node
# it assumes that the class, component, source name are the same, the node will be ${component_name}_node
macro(build_component_and_node component_name)
  
  add_library(${component_name} SHARED src/${component_name}.cpp)
  target_link_libraries(${component_name})
  #target_include_directories(${component_name})
  ament_target_dependencies(${component_name}
    rclcpp
    rclcpp_components
    sensor_msgs
    rcl_interfaces
  )
  rclcpp_components_register_nodes(${component_name} PLUGIN "monitoring::${component_name}" EXECUTABLE ${component_name})

  add_executable(${component_name}_node src/${component_name}.cpp )
  target_link_libraries(${component_name}_node)
  #target_include_directories(${component_name}_node)
  ament_target_dependencies(
    ${component_name}_node
    rclcpp
    rclcpp_components
    sensor_msgs
    rcl_interfaces
  )

endmacro()

build_component_and_node(monitor)


install(TARGETS
  monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY config
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
