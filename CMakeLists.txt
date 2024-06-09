cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${EIGEN3_LIBRARY_DIRS}
)

ament_auto_add_library(ekf_localization_component SHARED
 src/ekf_localization_component.cpp)
target_compile_definitions(ekf_localization_component
  PRIVATE "AFV_EKF_LOCALIZATION_COMPONENT_BUILDING_DLL")
ament_auto_add_executable(ekf_localization_node
 src/ekf_localization_node.cpp
)

rclcpp_components_register_nodes(ekf_localization_component
"ekf_localization::ExtendedKalmanFilterComponent")

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()