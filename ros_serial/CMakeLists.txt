cmake_minimum_required(VERSION 2.8.3)
project(ros_serial)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation serial tf sensor_msgs nav_msgs nmea_msgs )#gnss)

add_message_files(
    FILES
    serial.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ros_serial
    CATKIN_DEPENDS roscpp message_runtime serial std_msgs rospy tf sensor_msgs nav_msgs
    DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_serial src/ros_serial.cpp)
add_dependencies(ros_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_serial ${catkin_LIBRARIES})



#############
## Install ##
#############
install(TARGETS ros_serial
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# 添加目录

#install(DIRECTORY launch
#            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
# install(DIRECTORY mesh
#            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
# install(DIRECTORY rviz
#            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )

#install(FILES
#   launch/scout_base.launch 
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )



