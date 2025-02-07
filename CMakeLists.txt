cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)
get_project_name("package.xml" extracted_name extracted_version)
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
set(CMAKE_BUILD_TYPE Release)

# define ROS2 dependencies
set(ROS2_DEPENDENCIES
  rclcpp
  tf2_ros
  zed_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(human_model REQUIRED)

# find all ROS 2 dependencies
foreach(Dependency IN LISTS ROS2_DEPENDENCIES)
  find_package(${Dependency} REQUIRED)
endforeach()

### ${PROJECT_NAME} LIBRARY ###
# declare the ${PROJECT_NAME} target library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/human_kinematics_publisher.cpp
)
  
# link the ${PROJECT_NAME} target against the required libraries (pure C++ libraries)
target_link_libraries(${PROJECT_NAME} PUBLIC
  human_model::human_model
)

# link the ${PROJECT_NAME} target against the required libraries (ROS2 libraries)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS2_DEPENDENCIES})

# define the include directories for the ${PROJECT_NAME} target
# (the ament_target_dependencies function is used to handle
#  the include directories for the ROS 2 dependencies,
#  so you don't need to manually specify them
#  in the target_include_directories section)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# define the ${PROJECT_NAME} target as an alias to the actual target
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# define the C++ standard to use for the ${PROJECT_NAME} target
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
### ${PROJECT_NAME} LIBRARY ###


### human_kinematics_node EXECUTABLE ###
# declare the human_kinematics_node target executable
add_executable(human_kinematics_node
  src/human_kinematics_node.cpp
)

# link the human_kinematics_node target against the required libraries
target_link_libraries(human_kinematics_node PUBLIC
  rclcpp::rclcpp
  ${PROJECT_NAME}
)

# define the include directories for the human_kinematics_node target
target_include_directories(human_kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# define the C++ standard to use for the human_kinematics_node target
target_compile_features(human_kinematics_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


### INSTALLATION ###
# Install the include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the library target
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Export the dependencies of the library
ament_export_dependencies(${ROS2_DEPENDENCIES})

# Install the executable
install(
  TARGETS human_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the export so that other CMake projects can find the library
install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION "share/${PROJECT_NAME}/cmake"
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
)

# Create a package configuration file and version file + install them
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION "share/${PROJECT_NAME}/cmake"
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION "share/${PROJECT_NAME}/cmake"
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

### INSTALLATION ###


### TESTING ###
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
### TESTING ###


ament_package()
