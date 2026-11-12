cmake_minimum_required(VERSION 3.8)
project(robot_ctrl)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The robot type and the platform (ROS2 + Mujoco simulation) used by the controller
add_definitions(-DROBOT_TYPE_Go1 -DCOMPILE_WITH_ROS -DCOMPILE_WITH_SIMULATION)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)

# jsoncpp: used to parse the cmd json file in the "mycmd" folder
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

include_directories(include)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  ament_index_cpp
)

file(GLOB_RECURSE ROBOT_CTRL_SRC
  "src/*/*.cpp"
  "src/*/*.cc"
)

add_executable(robot_ctrl src/main.cpp ${ROBOT_CTRL_SRC})

ament_target_dependencies(robot_ctrl ${DEPENDENCY_LIST})
target_link_libraries(robot_ctrl PkgConfig::JSONCPP Threads::Threads)

# ros2 run 只会在 lib/<package_name>/ 下查找可执行文件，
# 所以必须安装到 lib/${PROJECT_NAME}，不能使用默认的 bin/
install(TARGETS
        robot_ctrl
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY mycmd
        DESTINATION share/${PROJECT_NAME})



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

ament_package()
