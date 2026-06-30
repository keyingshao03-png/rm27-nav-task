cmake_minimum_required(VERSION 3.8)
project(sentry_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/odom_tf_publisher.py
  scripts/moving_obstacle.py
  scripts/simple_nav.py
  scripts/check_nav2_status.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
