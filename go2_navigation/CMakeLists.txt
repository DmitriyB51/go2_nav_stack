cmake_minimum_required(VERSION 3.16)
project(go2_navigation)

find_package(ament_cmake REQUIRED)

# C++ узлов нет — только launch, конфиги, карты и python-скрипты
install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# исполняемыми, чтобы работал `ros2 run go2_navigation <script>.py`
install(PROGRAMS
  scripts/bag_to_grid.py
  scripts/goal_to_planner.py
  scripts/goal_to_controller.py
  scripts/simple_controller.py
  scripts/costmap_health.py
  scripts/plot_trajectory.py
  scripts/odom_to_tf.py
  scripts/pcd_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
