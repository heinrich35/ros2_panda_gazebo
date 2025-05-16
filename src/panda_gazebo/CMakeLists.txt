cmake_minimum_required(VERSION 3.8)
project(panda_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(
  PROGRAMS
  
    #scripts/ik_executor.py
    #scripts/cartesian_keyboard_control.py
    scripts/initial_pose_pub.py
  DESTINATION lib/${PROJECT_NAME}
)
