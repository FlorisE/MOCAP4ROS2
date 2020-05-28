cmake_minimum_required(VERSION 3.5)
project(configuration_from_mocap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)
find_package(mocap4ros_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(marker_viz_srvs REQUIRED)

set(dependencies rclcpp mocap4ros_msgs tf2_msgs tf2_eigen Eigen3 marker_viz_srvs tinyxml_vendor TinyXML urdf)

add_library(marker_lib src/marker_lib.cpp)

target_include_directories(marker_lib
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
     ${TinyXML_EXPORTED_TARGETS})

ament_target_dependencies(marker_lib ${dependencies})
target_link_libraries(marker_lib ${TinyXML_LIBRARIES})

add_library(marker_file_exporter src/marker_file_exporter.cpp)

target_include_directories(marker_file_exporter
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>)

ament_target_dependencies(marker_file_exporter ${dependencies})

add_executable(marker_file_exporter_main src/marker_file_exporter_main.cpp)

target_include_directories(marker_file_exporter_main
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>)

ament_target_dependencies(marker_file_exporter_main ${dependencies})
target_link_libraries(marker_file_exporter_main marker_file_exporter ${TinyXML_LIBRARIES})

add_library(marker_file_updater src/marker_file_updater.cpp)

target_include_directories(marker_file_updater
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
     ${TinyXML_EXPORTED_TARGETS})

ament_target_dependencies(marker_file_updater ${dependencies})
target_link_libraries(marker_file_updater marker_lib ${TinyXML_LIBRARIES})

add_executable(marker_file_updater_main src/marker_file_updater_main.cpp)

target_include_directories(marker_file_updater_main
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
     ${TinyXML_EXPORTED_TARGETS})

ament_target_dependencies(marker_file_updater_main ${dependencies})
target_link_libraries(marker_file_updater_main marker_file_updater ${TinyXML_LIBRARIES})

add_library(estimator src/estimator.cpp)

target_include_directories(estimator
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
     ${TinyXML_EXPORTED_TARGETS})

ament_target_dependencies(estimator ${dependencies})

target_link_libraries(estimator marker_lib ${TinyXML_LIBRARIES})

add_executable(estimator_main src/estimator_main.cpp)

target_include_directories(estimator_main
  PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
     ${TinyXML_EXPORTED_TARGETS})

ament_target_dependencies(estimator_main ${dependencies})

target_link_libraries(estimator_main estimator ${TinyXML_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
