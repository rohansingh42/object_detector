cmake_minimum_required(VERSION 2.8.3)
project(object_detector)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp roslaunch std_msgs geometry_msgs sensor_msgs nav_msgs message_generation)
find_package(Eigen3 REQUIRED)

add_message_files(FILES Point.msg Circle.msg)
generate_messages(DEPENDENCIES std_msgs object_detector)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detector
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_library(ExtractFeatures src/ExtractFeatures.cpp)
target_link_libraries(ExtractFeatures ${catkin_LIBRARIES} ${Eigen3_INCLUDE_DIRS} Eigen3::Eigen)
add_dependencies(ExtractFeatures ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(TrackFeature src/TrackFeature.cpp src/kalman.cpp)
target_link_libraries(TrackFeature ${catkin_LIBRARIES} ${Eigen3_INCLUDE_DIRS} Eigen3::Eigen)
add_dependencies(TrackFeature ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


add_executable(extract_features_node src/nodes/ExtractFeaturesNode.cpp)
target_link_libraries(extract_features_node ExtractFeatures)

add_executable(track_feature_node src/nodes/TrackFeatureNode.cpp)
target_link_libraries(track_feature_node TrackFeature)

install(TARGETS ExtractFeatures TrackFeature
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


install(TARGETS extract_features_node track_feature_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

