message(STATUS "chrono-actuators HIL scaffold enabled (no hardware code in v0.1 scaffold)")

add_library(Chrono_actuators_hil_stub INTERFACE)
add_library(Chrono::actuators_hil_stub ALIAS Chrono_actuators_hil_stub)

target_link_libraries(Chrono_actuators_hil_stub INTERFACE Chrono::actuators)
