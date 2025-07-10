cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUID_TYPE "Debug")
project(collison_visualize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
# ros2的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# eigen依赖
find_package(Eigen3 REQUIRED)

# 设置fcl依赖
set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH OFF)
set(FCL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/fcl/include")
set(FCL_LIBRARIES_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/fcl/build/lib")
set(LIBCCD_LIBRARIES_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/build/libccd/lib")
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARIES_DIRS})
link_directories(${LIBCCD_LIBRARIES_DIRS})

include_directories(include)
include_directories(${Eigen3_INCLUDE_DIRS})
add_executable(main_node src/main.cpp src/manipulator.cpp)
target_link_libraries(main_node "libccd.so" "libfcl.so")

ament_target_dependencies(
  main_node 
  rclcpp
  visualization_msgs
  tf2_geometry_msgs
)

###################################安装到install文件夹下###################################安
# 1. 编译出来的lib
install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 一些自定义的自己的目录需要手动安装
# 安装urdf目录
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装launch目录
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
