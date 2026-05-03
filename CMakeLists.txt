cmake_minimum_required(VERSION 3.8)
project(my_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

install(DIRECTORY
  urdf launch worlds config maps scripts
  DESTINATION share/${PROJECT_NAME}
)

# ← THIS BLOCK is what makes ros2 run work
install(PROGRAMS
  scripts/astar_planner.py
  scripts/autonomous_explorer.py
  scripts/goal_navigator.py
  scripts/easy_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()