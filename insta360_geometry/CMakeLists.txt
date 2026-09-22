cmake_minimum_required(VERSION 3.8)
project(insta360_geometry)

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

# --- Core library (no rclcpp) ---
add_library(insta360_geometry_core
  src/equidistant_model.cpp
  src/fisheye_camera.cpp
)
target_include_directories(insta360_geometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(insta360_geometry_core PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

install(TARGETS insta360_geometry_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lens_params test/test_lens_params.cpp)
  target_link_libraries(test_lens_params insta360_geometry_core)
  ament_add_gtest(test_fisheye_model test/test_fisheye_model.cpp)
  target_link_libraries(test_fisheye_model insta360_geometry_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(insta360_geometry_core)
ament_export_dependencies(Eigen3 OpenCV)
ament_package()
