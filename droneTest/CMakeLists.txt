cmake_minimum_required(VERSION 2.8.3)
project(droneTest)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  rospy
  message_generation
)

add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  Num_srv.srv
)

generate_messages(
   DEPENDENCIES
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

#${OpenCV_LIBS})
find_package(OpenCV)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(droneTest src/droneTest.cpp)
# Do not just add this line to your CMakeLists.txt, modify the existing line
target_link_libraries(
droneTest ${catkin_LIBRARIES} 
${OpenCV_LIBRARIES}
)

add_dependencies(droneTest droneTest_generate_messages_cpp)


