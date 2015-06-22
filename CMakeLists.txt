cmake_minimum_required(VERSION 2.8.3)
project(outdoor_bot)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  tf
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  std_msgs
  cv_bridge
  image_transport
  image_view
  image_geometry
  image_proc
  sensor_msgs
  geometry_msgs
  message_generation
  #OpenCV
)   

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
	 autoMove_msg.msg
    digcams_custom.msg
	 dirAnt_msg.msg
	 mainTargetsCommand_msg.msg
	 mainTargets_msg.msg
	 movement_msg.msg
	 NavTargets_msg.msg
	 navTargetsCommand_msg.msg
	 ObstacleDetectorFeedback_msg.msg
	 Obstacle_msg.msg
	 ObstacleParams_msg.msg
	 pmotor_msg.msg
	 radar_msg.msg
	 servo_msg.msg
	 webcams_custom.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   accelerometers_service.srv
   autoMove_service.srv
   digcams_service.srv
   dirAnt_service.srv
   encoders_service.srv
   mainTargets_service.srv
   NavTargets_service.srv
   obstacle_service.srv
   radar_service.srv
   servos_service.srv
   setPose_service.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   move_base_msgs
   nav_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
##INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS 
 include
 /home/dbarry/Dropbox/ros/catkin_ws/devel/include/outdoor_bot
 /home/dbarry/Dropbox/ros/catkin_ws/src/outdoor_bot/include
 ${OpenCV_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 
 LIBRARIES
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}
 
 CATKIN_DEPENDS 
 message_runtime 
 std_msgs
 sensor_msgs
 geometry_msgs
 actionlib_msgs
 rosconsole

  
 DEPENDS 
 rosconsole 
 tf 
 roscpp 
 rospy 
 cv_bridge 
 image_transport

)

###########
## Build ##
###########

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	/home/dbarry/Dropbox/ros/catkin_ws/devel/include/outdoor_bot
	/home/dbarry/Dropbox/ros/catkin_ws/src/outdoor_bot/include
)


add_library(rcm src/rcmRadar/rcm.c src/rcmRadar/rcmIf.c src/rcmRadar/radar.cpp)
add_library(fbfsm src/FBFSM/FBFSM.cpp)
add_library(outdoorbot_nav src/navigation/NavUtils.cpp src/navigation/ObstacleDetector.cpp src/navigation/WallFollower.cpp src/navigation/DirAntFollower.cpp)

add_dependencies(rcm ${PROJECT_NAME}_gencpp)
add_dependencies(fbfsm ${PROJECT_NAME}_gencpp)
add_dependencies(outdoorbot_nav ${PROJECT_NAME}_gencpp)


add_executable(robotPose_node nodes/robotPose_node.cpp)
add_executable(serialComm_node nodes/serialComm_node.cpp)
add_executable(movement_node nodes/movement_node.cpp)
add_executable(keyboardController_node nodes/keyboardController_node.cpp)
add_executable(baseController_node nodes/baseController_node.cpp)
add_executable(pololu_motor_control nodes/pololu_motor_control.cpp)
add_executable(digcams nodes/digcams.cpp)
add_executable(webcams nodes/webcams.cpp)
add_executable(imagecv nodes/imagecv.cpp)
add_executable(NavTargets nodes/NavTargets.cpp)
add_executable(mainTargets nodes/mainTargets.cpp)
add_executable(autonomous_node nodes/autonomous_node.cpp)
add_executable(autonomous_node_obstacle_testing nodes/autonomous_node_obstacle_testing.cpp)
add_executable(radar_node nodes/radar_node.cpp)
add_executable(dirAnt_node nodes/dirAnt_node.cpp)
add_executable(userCommands_node nodes/userCommands_node.cpp)
add_executable(obstacle_node nodes/obstacle_node.cpp)
add_executable(obstacles_service_node nodes/obstacles_service_node.cpp)
add_executable(obstacle_tester_node nodes/obstacle_tester_node.cpp)
add_executable(obstacle_avoider_tester_node nodes/obstacle_avoider_tester_node.cpp)
add_executable(republish_scan_node nodes/republish_scan_node.cpp)
#add_executable(simple_blobs src/simple_blobs.cpp)
#add_executable(blob_detector src/blob_detector.cpp)
#add_executable(radar src/rcmRadar/rcm.c src/rcmRadar/rcmIf.c src/rcmRadar/rcmSampleApp.c)
#add_executable(radar src/rcmRadar/rcmSampleApp.c)
#add_executable(odomTester_node nodes/odomTester_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(outdoor_bot_catkin_node outdoor_bot_catkin_generate_messages_cpp)

add_dependencies(robotPose_node ${PROJECT_NAME}_gencpp)
add_dependencies(serialComm_node ${PROJECT_NAME}_gencpp)
add_dependencies(movement_node ${PROJECT_NAME}_gencpp)
add_dependencies(keyboardController_node ${PROJECT_NAME}_gencpp)
add_dependencies(baseController_node ${PROJECT_NAME}_gencpp)
add_dependencies(pololu_motor_control ${PROJECT_NAME}_gencpp)
add_dependencies(digcams ${PROJECT_NAME}_gencpp)
add_dependencies(webcams ${PROJECT_NAME}_gencpp)
add_dependencies(imagecv ${PROJECT_NAME}_gencpp)
add_dependencies(NavTargets ${PROJECT_NAME}_gencpp)
add_dependencies(mainTargets ${PROJECT_NAME}_gencpp)
add_dependencies(autonomous_node ${PROJECT_NAME}_gencpp)
add_dependencies(autonomous_node_obstacle_testing ${PROJECT_NAME}_gencpp)
add_dependencies(radar_node ${PROJECT_NAME}_gencpp)
add_dependencies(dirAnt_node ${PROJECT_NAME}_gencpp)
add_dependencies(userCommands_node ${PROJECT_NAME}_gencpp)
add_dependencies(obstacle_node ${PROJECT_NAME}_gencpp)
add_dependencies(obstacles_service_node ${PROJECT_NAME}_gencpp)
add_dependencies(obstacle_tester_node ${PROJECT_NAME}_gencpp)
add_dependencies(obstacle_avoider_tester_node ${PROJECT_NAME}_gencpp)
add_dependencies(republish_scan_node ${PROJECT_NAME}_gencpp)

target_link_libraries(serialComm_node ${catkin_LIBRARIES})
target_link_libraries(digcams ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(webcams ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(imagecv ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(NavTargets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mainTargets ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(autonomous_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(autonomous_node_obstacle_testing ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(simple_blobs ${OpenCV_LIBRARIES})
#target_link_libraries(blob_detector ${OpenCV_LIBRARIES})
target_link_libraries(digcams gphoto2 gphoto2_port ${catkin_LIBRARIES})
target_link_libraries(robotPose_node rcm ${catkin_LIBRARIES})
target_link_libraries(movement_node rcm ${catkin_LIBRARIES})
target_link_libraries(radar_node rcm ${catkin_LIBRARIES})
target_link_libraries(dirAnt_node ${catkin_LIBRARIES})
target_link_libraries(baseController_node ${catkin_LIBRARIES})
target_link_libraries(obstacle_tester_node ${catkin_LIBRARIES})
target_link_libraries(obstacle_node ${catkin_LIBRARIES})
target_link_libraries(republish_scan_node ${catkin_LIBRARIES})
target_link_libraries(dirAnt_node ${catkin_LIBRARIES})
target_link_libraries(keyboardController_node ${catkin_LIBRARIES})
target_link_libraries(autonomous_node fbfsm ${catkin_LIBRARIES})
target_link_libraries(userCommands_node ${catkin_LIBRARIES})
target_link_libraries(autonomous_node outdoorbot_nav ${catkin_LIBRARIES})
target_link_libraries(obstacles_service_node outdoorbot_nav ${catkin_LIBRARIES})
target_link_libraries(obstacles_service_node fbfsm ${catkin_LIBRARIES})
target_link_libraries(obstacle_avoider_tester_node outdoorbot_nav ${catkin_LIBRARIES})
target_link_libraries(obstacle_avoider_tester_node fbfsm ${catkin_LIBRARIES})
target_link_libraries(autonomous_node_obstacle_testing fbfsm ${catkin_LIBRARIES})
target_link_libraries(autonomous_node_obstacle_testing outdoorbot_nav ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS outdoor_bot_catkin outdoor_bot_catkin_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_outdoor_bot_catkin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
