cmake_minimum_required(VERSION 3.8)
project(insta360_cv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(insta360_geometry REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# --- Core library (Eigen + OpenCV + geometry, no rclcpp, no ROS types) ---
add_library(insta360_cv_core
  src/aruco_detector.cpp
  src/marker_pose_estimator.cpp
)
target_include_directories(insta360_cv_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${insta360_geometry_INCLUDE_DIRS}
)
target_link_libraries(insta360_cv_core PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${insta360_geometry_LIBRARIES}
)
install(TARGETS insta360_cv_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# --- ROS packages / node ---
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(aruco_detector_node src/ros/aruco_detector_node.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aruco_detector_node
  ${cpp_typesupport_target}
  insta360_cv_core
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${cv_bridge_TARGETS}
)
install(TARGETS aruco_detector_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_range test/test_pose_range.cpp)
  target_link_libraries(test_pose_range insta360_cv_core)
  ament_add_gtest(test_aruco_detector test/test_aruco_detector.cpp)
  target_link_libraries(test_aruco_detector insta360_cv_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(insta360_cv_core)
ament_export_dependencies(Eigen3 OpenCV insta360_geometry rosidl_default_runtime)
ament_package()
