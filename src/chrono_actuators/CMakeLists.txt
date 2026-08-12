include(CMakePackageConfigHelpers)

# Default to Release when the user has not set a build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (default: Release)" FORCE)
  message(STATUS "chrono-actuators: CMAKE_BUILD_TYPE not set — defaulting to Release")
endif()

# ---------------------------------------------------------------------------
# Model layer (no Chrono dependency — C++ stdlib only)
# ---------------------------------------------------------------------------
set(CHRONO_ACTUATORS_MODEL_HEADERS
  models/ActuatorModel.h
  models/Telemetry.h
  models/Envelope.h
  models/LinearDamperModel.h
  models/ReactiveDamperModel.h
  models/ElectricActuatorModel.h
)

# ---------------------------------------------------------------------------
# Hardware edge
# ---------------------------------------------------------------------------
set(CHRONO_ACTUATORS_IO_HEADERS
  ChActuatorIO.h
)

# ---------------------------------------------------------------------------
# Chrono binding layer
# ---------------------------------------------------------------------------
set(CHRONO_ACTUATORS_CHRONO_HEADERS
  chrono/ChActuatorFunctorTSDA.h
  chrono/ChActuatorFunctorRSDA.h
  chrono/ChActuatorFunction.h
  chrono/ChActuatorShaft.h
  chrono/ChActuatorDynamics.h
)

set(CHRONO_ACTUATORS_CHRONO_SOURCES
  chrono/ChActuatorDynamics.cpp
)

set(CHRONO_ACTUATORS_ALL_HEADERS
  ${CHRONO_ACTUATORS_MODEL_HEADERS}
  ${CHRONO_ACTUATORS_IO_HEADERS}
  ${CHRONO_ACTUATORS_CHRONO_HEADERS}
  ChApiActuators.h
  ChConfigActuators.h.in
)

add_library(Chrono_actuators
  ${CHRONO_ACTUATORS_CHRONO_SOURCES}
  ${CHRONO_ACTUATORS_ALL_HEADERS}
)
add_library(Chrono::actuators ALIAS Chrono_actuators)

# Resolve Chrono core target (handles both installed and in-build tree)
set(CHRONO_ACTUATORS_CHRONO_CORE_TARGET "")
if(TARGET Chrono_core)
  set(CHRONO_ACTUATORS_CHRONO_CORE_TARGET Chrono_core)
elseif(TARGET Chrono::Chrono_core)
  set(CHRONO_ACTUATORS_CHRONO_CORE_TARGET Chrono::Chrono_core)
elseif(TARGET Chrono::Chrono)
  set(CHRONO_ACTUATORS_CHRONO_CORE_TARGET Chrono::Chrono)
endif()

if(CHRONO_ACTUATORS_CHRONO_CORE_TARGET STREQUAL "")
  message(FATAL_ERROR
    "Could not locate Chrono core target "
    "(tried Chrono_core, Chrono::Chrono_core, Chrono::Chrono)")
endif()

# No -latomic: only std::atomic<uint32_t> is used (lock-free everywhere).
target_link_libraries(Chrono_actuators PUBLIC ${CHRONO_ACTUATORS_CHRONO_CORE_TARGET})
target_compile_definitions(Chrono_actuators PRIVATE CH_API_COMPILE_ACTUATORS)

target_include_directories(Chrono_actuators
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(CHRONO_ACTUATORS_HAS_ROS ${CHRONO_ACTUATORS_ENABLE_ROS})
set(CHRONO_ACTUATORS_HAS_HIL ${CHRONO_ACTUATORS_ENABLE_HIL})

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigActuators.h.in
  ${PROJECT_BINARY_DIR}/chrono_actuators/ChConfigActuators.h
  @ONLY
)

install(FILES ${PROJECT_BINARY_DIR}/chrono_actuators/ChConfigActuators.h
        DESTINATION include/chrono_actuators)

install(TARGETS Chrono_actuators
        EXPORT ChronoActuatorsTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_actuators)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_actuators
        FILES_MATCHING PATTERN "*.h")

set(CHRONO_ACTUATORS_CMAKE_INSTALL_DIR lib/cmake/ChronoActuators)

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/ChronoActuatorsConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/ChronoActuatorsConfig.cmake.in
  ${PROJECT_BINARY_DIR}/ChronoActuatorsConfig.cmake
  INSTALL_DESTINATION ${CHRONO_ACTUATORS_CMAKE_INSTALL_DIR}
)

install(EXPORT ChronoActuatorsTargets
        FILE ChronoActuatorsTargets.cmake
        NAMESPACE Chrono::
        DESTINATION ${CHRONO_ACTUATORS_CMAKE_INSTALL_DIR})

install(FILES
          ${PROJECT_BINARY_DIR}/ChronoActuatorsConfig.cmake
          ${PROJECT_BINARY_DIR}/ChronoActuatorsConfigVersion.cmake
        DESTINATION ${CHRONO_ACTUATORS_CMAKE_INSTALL_DIR})

if(CHRONO_ACTUATORS_ENABLE_ROS)
  find_package(chrono_ros QUIET)
  if(chrono_ros_FOUND)
    add_subdirectory(ros)
  else()
    message(STATUS "chrono-actuators: chrono_ros not found — ROS handler skipped")
  endif()
endif()
