cmake_minimum_required(VERSION 3.8)
project(ros2neuro_acquisition_eegdev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ros2neuro_msgs REQUIRED)
find_package(ros2neuro_data REQUIRED)
find_package(ros2neuro_acquisition REQUIRED)
find_library(EEGDEV_LIBRARIES eegdev REQUIRED)
find_path(EEGDEV_INCLUDE_DIRS eegdev.h REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(ros2neuro_acquisition plugins.xml)

# ---- add the library for the EGDDevice
add_library(${PROJECT_NAME} 
  src/EGDDevice.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EEGDEV_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${EEGDEV_LIBRARIES}
)
ament_target_dependencies(
  ${PROJECT_NAME}
  "ros2neuro_msgs"
  "pluginlib"
  "ros2neuro_data"
  "ros2neuro_acquisition"
  "rclcpp"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "EGDDEVICE_BUILDING_LIBRARY")

# ---- install and export the library
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# ---- to remove warnings
if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME}  PRIVATE -Wno-unused-parameter)
    target_compile_options(${PROJECT_NAME}  PRIVATE -Wno-parentheses)
    target_compile_options(${PROJECT_NAME}  PRIVATE -Wno-unused-variable)
    target_compile_options(${PROJECT_NAME}  PRIVATE -Wno-sign-compare) 
endif()

ament_package()
