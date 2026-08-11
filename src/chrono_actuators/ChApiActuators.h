#ifndef CH_ACTUATORS_API_H
#define CH_ACTUATORS_API_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

#if defined(CH_API_COMPILE_ACTUATORS)
#define ChApiActuators ChApiEXPORT
#else
#define ChApiActuators ChApiIMPORT
#endif

/**
 * @defgroup actuators Chrono::Actuators module
 * @brief Standalone actuator-modeling scaffold for Chrono.
 *
 * This module establishes the architecture and interfaces for actuator-augmented
 * multibody simulation while intentionally leaving model implementations for
 * follow-up PRs.
 */

namespace chrono {
namespace actuators {}
}  // namespace chrono

#endif
