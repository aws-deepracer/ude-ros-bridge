cmake_minimum_required(VERSION 3.7.2)
project(ude_ros_server)

find_package(catkin REQUIRED COMPONENTS
    rospy
    ude
    std_msgs
    ude_ros_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS ude rospy std_msgs ude_ros_msgs
)

catkin_install_python(
  PROGRAMS ude_ros_server/ude_ros_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
