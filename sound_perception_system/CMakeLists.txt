cmake_minimum_required(VERSION 3.8)
project(sound_perception_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sound_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sound_msgs
  message_filters
)

# Source and header files
set(SOURCES
  src/${PROJECT_NAME}/SoundDetectionNode.cpp
)

set(HEADERS
  include/${PROJECT_NAME}/SoundDetectionNode.hpp
)

# Build library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(sound_detection_main src/sound_detection_main.cpp)
ament_target_dependencies(sound_detection_main ${dependencies})
target_link_libraries(sound_detection_main ${PROJECT_NAME})

install(TARGETS
  sound_detection_main
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
