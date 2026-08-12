# Tests that require only the model layer (no Chrono dependency)
add_executable(test_model_layer
  test_model_layer.cpp
)
target_include_directories(test_model_layer PRIVATE
  ${CMAKE_SOURCE_DIR}/src
)
# Intentionally NO Chrono link — verifies Chrono-free compilation.
add_test(NAME chrono_free_model_layer COMMAND test_model_layer)

# Tests that require ChActuatorIO (seqlock, watchdog, engage gate, clamps)
add_executable(test_actuator_io
  test_actuator_io.cpp
)
target_include_directories(test_actuator_io PRIVATE
  ${CMAKE_SOURCE_DIR}/src
)
add_test(NAME actuator_io COMMAND test_actuator_io)

# Integration tests that require Chrono
add_executable(test_chrono_binding
  test_chrono_binding.cpp
)
target_link_libraries(test_chrono_binding PRIVATE Chrono::actuators)
add_test(NAME chrono_binding COMMAND test_chrono_binding)
