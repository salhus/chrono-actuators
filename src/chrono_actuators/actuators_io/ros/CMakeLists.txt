message(STATUS "chrono-actuators ROS scaffold enabled (no ROS code in v0.1 scaffold)")

add_library(Chrono_actuators_ros_stub INTERFACE)
add_library(Chrono::actuators_ros_stub ALIAS Chrono_actuators_ros_stub)

target_link_libraries(Chrono_actuators_ros_stub INTERFACE Chrono::actuators)
