cmake_minimum_required(VERSION 3.0.2)
project(computer_vision_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)


add_message_files(
  FILES
  PosOriObjectsArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS
  scripts/create_tf.py
  scripts/main_script.py
  scripts/robot_control.py
  scripts/robot_control_2.py
  scripts/get_poitcloud_topic.py
  scripts/generate_obj_tf.py
  # scripts/get_poitcloud_topic_old.py
  scripts/pointcloud_publisher_topic.py
  scripts/get_tfs.py
  scripts/robot_test.py
  scripts/tests.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# add_executable(exc_name src/cpp_file_name.cpp)
# target_link_libraries(
#   exc_name
#   ${catkin_LIBRARIES}
#   ${POCO_LIBS}
#   ${RWS_LIBS}
# )

