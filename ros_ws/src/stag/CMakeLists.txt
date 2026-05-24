cmake_minimum_required(VERSION 3.8)
project(stag)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphNode.msg"
  "msg/GraphEdge.msg"
  "msg/TerrainGraph.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_python_install_package(stag_core)

install(PROGRAMS
  scripts/stag_node
  scripts/demo_costmap_publisher
  scripts/benchmark_topology_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(topology_core test/test_topology_core.py)
  ament_add_pytest_test(stag_node_integration test/test_stag_node_integration.py TIMEOUT 10)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
