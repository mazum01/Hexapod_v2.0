// command_helpers.h — Small shared helpers for command parsing and masks
#ifndef MARS_COMMAND_HELPERS_H
#define MARS_COMMAND_HELPERS_H

#include <stdint.h>
#include <string.h>

// Extern globals used by bitmask helpers (declared in main TU)
extern volatile uint8_t  g_leg_enabled_mask;
extern volatile uint32_t g_joint_enabled_mask;

// Token -> index helpers
static inline int legIndexFromToken(const char* t) {
  if (!t) return -1;
  if (!strcmp(t, "LF")) return 0;
  if (!strcmp(t, "LM")) return 1;
  if (!strcmp(t, "LR")) return 2;
  if (!strcmp(t, "RF")) return 3;
  if (!strcmp(t, "RM")) return 4;
  if (!strcmp(t, "RR")) return 5;
  return -1;
}
static inline int jointIndexFromToken(const char* t) {
  if (!t) return -1;
  if (!strcmp(t, "COXA") || !strcmp(t, "coxa")) return 0;
  if (!strcmp(t, "FEMUR") || !strcmp(t, "femur")) return 1;
  if (!strcmp(t, "TIBIA") || !strcmp(t, "tibia")) return 2;
  return -1;
}

// Enable mask queries
static inline bool leg_enabled_mask_get(uint8_t leg) {
  return ((g_leg_enabled_mask >> leg) & 1u) != 0;
}
static inline bool joint_enabled_mask_get(uint8_t leg, uint8_t joint) {
  uint8_t idx = (uint8_t)(leg * 3u + joint);
  return ((g_joint_enabled_mask >> idx) & 1u) != 0;
}

// LX-16A unit conversions: 1 unit ≈ 0.24° = 24 centideg
static inline int16_t units_to_cd(int units) {
  return (int16_t)(units * 24);
}
static inline int16_t cd_to_units_round(int cd) {
  if (cd < -32768) cd = -32768; if (cd > 32767) cd = 32767;
  // round to nearest
  int n = (cd >= 0) ? ((cd + 12) / 24) : ((cd - 12) / 24);
  return (int16_t)n;
}

#endif // MARS_COMMAND_HELPERS_H
