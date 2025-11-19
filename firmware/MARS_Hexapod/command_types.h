// command_types.h — Shared command enum for the command processor
// Placed in a header so Arduino's auto-prototype generation sees the type
// before emitting function prototypes that return CommandType.

#ifndef MARS_COMMAND_TYPES_H
#define MARS_COMMAND_TYPES_H

#include <stdint.h>

enum CommandType : uint8_t {
  CMD_HELP = 0, CMD_STATUS, CMD_REBOOT, CMD_ENABLE, CMD_DISABLE,
  CMD_FK, CMD_LEGS, CMD_SERVOS, CMD_LEG, CMD_SERVO,
  CMD_RAW, CMD_RAW3, CMD_FOOT, CMD_FEET, CMD_MODE,
  CMD_I, CMD_T, CMD_TEST, CMD_STAND, CMD_SAFETY,
  CMD_HOME, CMD_SAVEHOME, CMD_OFFSET, CMD_LOG,
  CMD_LOOP,
  CMD_TUCK, CMD_PID, CMD_IMP, CMD_COMP, CMD_EST,
  CMD_LIMITS, CMD_CONFIG,
  CMD_UNKNOWN
};

#endif // MARS_COMMAND_TYPES_H
