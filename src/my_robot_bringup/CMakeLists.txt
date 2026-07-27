cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(DIRECTORY launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/square_move.py
  scripts/imu_dead_reckoning.py
  scripts/odom_to_path.py
  scripts/plot_trajectories.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
