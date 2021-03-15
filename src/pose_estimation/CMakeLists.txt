cmake_minimum_required(VERSION 3.0.2)
project(pose_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  cv_bridge
  message_generation

)


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})



#add_service_files(
#  FILES
#)



generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)



catkin_package(
  INCLUDE_DIRS include
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)





add_executable(visualize_data src/definations.cpp src/visualize_data.cpp)
target_link_libraries(visualize_data ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(visualize_data pose_estimation_generate_messages_cpp)



add_executable(get_selected_indices src/get_selected_indices.cpp)
target_link_libraries(get_selected_indices ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(get_selected_indices pose_estimation_generate_messages_cpp)


