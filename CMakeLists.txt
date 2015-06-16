cmake_minimum_required(VERSION 2.8.3)
project(orb2bowdb)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLib REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS})
add_executable(builder src/builder_node.cc)
target_link_libraries(builder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${DLib_LIBS} ${DBoW2_LIBRARIES})
