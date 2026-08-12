function(add_act_demo demo_name)
  add_executable(${demo_name} ${demo_name}.cpp)
  target_link_libraries(${demo_name} PRIVATE Chrono::actuators)
endfunction()

add_act_demo(demo_ACT_linear_damper)
add_act_demo(demo_ACT_electric)
add_act_demo(demo_ACT_shaft)
add_act_demo(demo_ACT_hil_modes)
