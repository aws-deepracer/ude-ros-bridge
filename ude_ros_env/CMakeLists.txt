cmake_minimum_required(VERSION 3.7.2)
project(ude_ros_env)

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

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
