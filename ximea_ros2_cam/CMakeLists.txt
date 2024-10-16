cmake_minimum_required(VERSION 3.5)
project(ximea_ros2_cam)

set(XIMEA_INCLUDE_PATH /usr/include)
set(XIMEA_LIBRARY m3api)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ximea_ros2_cam_node src/ximea_ros2_cam_node.cpp src/ximea_ros2_cam.cpp)
ament_target_dependencies(ximea_ros2_cam_node 
  "rclcpp"
  "image_transport"
  "sensor_msgs"
  "camera_calibration_parsers"
  "camera_info_manager"
  "rosbag2_cpp"
  "cv_bridge")

target_include_directories(ximea_ros2_cam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${XIMEA_INCLUDE_PATH}
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(ximea_ros2_cam_node
  ${XIMEA_LIBRARY}
  ${OpenCV_LIBRARIES}
)

install(TARGETS ximea_ros2_cam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(MAX_LINE_LENGTH "120")
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_DOCS)
  find_package(ament_cmake_doxygen REQUIRED)
  ament_doxygen(DOC_TITLE ${PROJECT_NAME})
endif()

ament_package()
