cmake_minimum_required(VERSION 3.8)
project(substation_sim)

# 仅保留 ament_cmake，移除对 gazebo_ros 等缺失包的编译时搜索
find_package(ament_cmake REQUIRED)

# 安装所有资源文件夹到 share 目录
install(DIRECTORY
  launch
  worlds
  models
  maps
  rviz
  scripts
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
