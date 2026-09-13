cmake_minimum_required(VERSION 3.16)
project(fly_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# brain_activity_monitor.py: a plain rclpy+Tkinter script, not a full Python
# ament package (fly_simulation is ament_cmake) - installing it as a program
# is enough to make `ros2 run fly_simulation brain_activity_monitor.py` work.
install(
  PROGRAMS scripts/brain_activity_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# moving_target.py (M6): same plain-rclpy-script idiom as
# brain_activity_monitor.py above - see that install()'s own comment.
install(
  PROGRAMS scripts/moving_target.py
  DESTINATION lib/${PROJECT_NAME}
)

# house_navigator.py (M7): same plain-rclpy-script idiom as
# moving_target.py above - see that install()'s own comment.
install(
  PROGRAMS scripts/house_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
