cmake_minimum_required(VERSION 3.8)
project(himloco_sim2sim)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/himloco_policy scripts/go2w_mujoco scripts/export_policy
  scripts/keyboard_control
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch policies DESTINATION share/${PROJECT_NAME})
ament_package()
