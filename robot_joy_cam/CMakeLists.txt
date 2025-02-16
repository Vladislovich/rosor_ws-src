cmake_minimum_required(VERSION 3.0.2)
project(robot_joy_cam)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(joystick src/joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES})

add_executable(rob_return src/rob_return.cpp)
target_link_libraries(rob_return ${catkin_LIBRARIES})

add_executable(image_push src/image_push.cpp)
target_link_libraries(image_push ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_pop src/image_pop.cpp)
target_link_libraries(image_pop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(forward_wall src/forward_wall.cpp)
target_link_libraries(forward_wall ${catkin_LIBRARIES})
