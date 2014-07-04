cmake_minimum_required(VERSION 2.8.3)
project(speed_limit)

find_package(catkin REQUIRED actionlib_msgs)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES
    Disable.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

