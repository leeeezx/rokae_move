cmake_minimum_required(VERSION 3.8)
project(rokae_move)

# set(SHARED "SHARED")

# 设定C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译输出信息为懒惰模式
set(CMAKE_INSTALL_MESSAGE LAZY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

link_libraries(stdc++fs)  # 链接filesystem库

# ************* 以下内容为手动添加部分 *****************
include_directories(include) # 添加项目中的include文件夹路径。指定头文件（.h 或 .hpp）的搜索路径。
include_directories(external)# 添加项目中的external文件夹路径
link_directories(lib)# 添加项目中的lib文件夹路径。指定库文件（.a 或 .so）的搜索路径。
# ************* 以下内容为手动添加结束 *****************

# find dependencies
find_package(Eigen3 REQUIRED) 
find_package(tf2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_visual_tools REQUIRED)

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



# # ********************添加可执行文件
# add_executable(rokae_force src/rokae_force.cpp)
# ament_target_dependencies(rokae_force  "moveit_ros_planning_interface"  rclcpp std_msgs Eigen3)
# # ament_target_dependencies(rokae_force  "moveit_ros_planning_interface" "moveit_visual_tools" rclcpp std_msgs Eigen3)

# add_executable(rokae_force_impedance src/rokae_force_impedance.cpp)
# ament_target_dependencies(rokae_force_impedance  "moveit_ros_planning_interface"  rclcpp std_msgs Eigen3)

add_executable(rokae_move_node 
  src/rokae_move_node.cpp
  src/trajectory_generator.cpp)

ament_target_dependencies(rokae_move_node  
  "moveit_ros_planning_interface"  
  rclcpp 
  std_msgs Eigen3
  tf2)

# 链接外部库：  xCoreSDK xMateModel
target_link_libraries(rokae_move_node
  ${SHARED} 
  xCoreSDK
  xMateModel
)


install(
  DIRECTORY 
    lib/
    external/
    include/
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  # rokae_force
  # rokae_force_impedance
  rokae_move_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()