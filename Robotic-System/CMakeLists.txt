cmake_minimum_required(VERSION 2.8.3)
project(ro_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  OpenCV
  roscpp
  rostime
  message_filters
  message_generation
  serial
  cv_bridge
  pcl_ros
  pcl_msgs
  tf_conversions
  tf
  image_transport
  rospy
  std_msgs
  shape_msgs
  moveit_msgs
  sensor_msgs
  kinect2_bridge
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
  actionlib_msgs
  actionlib
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

add_action_files(
   DIRECTORY action
   FILES 
   control.action
   
 )

add_action_files(
   DIRECTORY action
   FILES 
   task.action
   
 )
    

add_message_files(
   FILES
   Double.msg
   RoiList.msg
   serial.msg
   hexNum.msg
)

 generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs sensor_msgs
 )

catkin_package(
CATKIN_DEPENDS 
    message_runtime
    serial
    std_msgs
)

catkin_package(
  LIBRARIES
  actionlib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS}   
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
)

 add_executable(pic_recognize 
 include/pic_recognize.h
 include/RoiList.h
 src/pic_recognize_tie.cpp
 src/main.cpp
 )


 target_link_libraries(pic_recognize 
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 ${kinect2_bridge_LIBRARIES}
 )

 add_dependencies(
    pic_recognize
    ${ro_control_EXPORTED_TARGETS}
 )

 add_executable(irim 
 include/device_controler.h
 include/obstable_manager.h
 include/robot_config.h
 include/robot_control.h
 include/decision_maker.h
 include/data_Input.h
 include/offset.h
 
 src/device_controler.cpp
 src/decision_maker.cpp
 src/robot_control.cpp
 src/obstable_manager.cpp
 src/IRIM.cpp

 )

 target_link_libraries(irim 
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    irim
    ${ro_control_EXPORTED_TARGETS}
 )

 add_executable(orange_calibration
 include/device_controler.h 
 include/obstable_manager.h
 include/decision_maker.h
 include/robot_config.h
 include/robot_control.h
 include/offset.h
 src/robot_control.cpp
 src/device_controler.cpp
 src/decision_maker.cpp
 src/obstable_manager.cpp
 src/orange_calibration.cpp
 )

 target_link_libraries(orange_calibration
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    orange_calibration
    ${ro_control_EXPORTED_TARGETS}
 )

 add_executable(orange_calibration2
 include/device_controler.h 
 include/obstable_manager.h
 include/decision_maker.h
 include/robot_config.h
 include/robot_control.h
 include/offset.h
 src/robot_control.cpp
 src/device_controler.cpp
 src/decision_maker.cpp
 src/obstable_manager.cpp
 src/orange_calibration2.cpp
 )

 target_link_libraries(orange_calibration2
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    orange_calibration2
    ${ro_control_EXPORTED_TARGETS}
 )

 add_executable(impact
 include/device_controler.h 
 include/obstable_manager.h
 include/decision_maker.h
 include/robot_config.h
 include/robot_control.h
 include/offset.h
 src/robot_control.cpp
 src/device_controler.cpp
 src/decision_maker.cpp
 src/obstable_manager.cpp
 src/impact.cpp
 )

 target_link_libraries(impact
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    impact
    ${ro_control_EXPORTED_TARGETS}
 )

  add_executable(vale_open
 include/device_controler.h 
 include/obstable_manager.h
 include/decision_maker.h
 include/robot_config.h
 include/robot_control.h
 include/offset.h
 src/robot_control.cpp
 src/device_controler.cpp
 src/decision_maker.cpp
 src/obstable_manager.cpp
 src/vale_open.cpp
 )

 target_link_libraries(vale_open
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    vale_open
    ${ro_control_EXPORTED_TARGETS}
 )

 add_executable(check_lift
 include/device_controler.h
 include/obstable_manager.h
 include/robot_config.h
 include/robot_control.h
 include/decision_maker.h
 include/data_Input.h
 include/offset.h
 
 src/device_controler.cpp
 src/decision_maker.cpp
 src/robot_control.cpp
 src/obstable_manager.cpp
 src/check_lift.cpp

 )

 target_link_libraries(check_lift
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    check_lift
    ${ro_control_EXPORTED_TARGETS}
 )

 add_executable(task_publisher
 iros/Task.h
 iros/Map.h
 iros/Drawer.cpp
 iros/main.cpp
 iros/Task.cpp
 )

 target_link_libraries(task_publisher
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    task_publisher
    ${ro_control_EXPORTED_TARGETS}
 )


 add_executable(task_receiver
 include/device_controler.h
 include/obstable_manager.h
 include/robot_config.h
 include/robot_control.h
 include/decision_maker.h
 include/data_Input.h
 include/offset.h
 
 src/device_controler.cpp
 src/decision_maker.cpp
 src/robot_control.cpp
 src/obstable_manager.cpp
 src/task_receiver.cpp

 )

 target_link_libraries(task_receiver
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${PCL_LIBRARIES}
 )

 add_dependencies(
    task_receiver
    ${ro_control_EXPORTED_TARGETS}
 )