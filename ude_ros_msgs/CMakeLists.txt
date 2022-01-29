cmake_minimum_required(VERSION 3.0.2)
project(ude_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  UDESideChannelData.msg
  )

add_service_files(DIRECTORY srv FILES
  UDECloseSrv.srv
  UDEResetSrv.srv
  UDEStepSrv.srv
  UDEObservationSpaceSrv.srv
  UDEActionSpaceSrv.srv
  UDESideChannelSrv.srv
  SetUDEServerConfig.srv
  GetUDEServerConfig.srv
  )

generate_messages(DEPENDENCIES
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  )
