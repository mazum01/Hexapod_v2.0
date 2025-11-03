#pragma once
#include <stdint.h>

// Robot attributes: link dimensions and morphology
// Keep this header tiny; included by firmware .ino files.

// Link lengths (mm)
#define COXA_LENGTH_MM   41.70f
#define FEMUR_LENGTH_MM  80.00f
#define TIBIA_LENGTH_MM 133.78f

// Aliases for IK routines that expect these names
#ifndef COXA_LENGTH
#define COXA_LENGTH COXA_LENGTH_MM
#endif
#ifndef FEMUR_LENGTH
#define FEMUR_LENGTH FEMUR_LENGTH_MM
#endif
#ifndef TIBIA_LENGTH
#define TIBIA_LENGTH TIBIA_LENGTH_MM
#endif

namespace Robot {


// -----------------------------------------------------------------------------
// Splash banner (moved from .ino)
// -----------------------------------------------------------------------------
inline const char SPLASH_BANNER[] =
R"(+----------------------------------+
|  __  __    _    ____  ____       |
| |  \/  |  / \  |  _ \/ ___|      |
| | |\/| | / _ \ | |_) \___ \      |
| | |  | |/ ___ \|  _ < ___) |     |
| |_|  |_/_/   \_\_| \_\____/      |
|                                  |
+----------------------------------+
)";

// Canonical leg count used for arrays in this header
static constexpr uint8_t NUM_LEGS_CFG = 6;

// Coxa origin offsets in the body frame (mm), canonical leg order: LF, LM, LR, RF, RM, RR
static constexpr float COXA_OFFSET[NUM_LEGS_CFG][2] = {
  {-65.5959f,  88.163f}, // LF
  {-86.0000f,   0.000f}, // LM
  {-65.5959f, -88.163f}, // LR
  { 65.5959f,  88.163f}, // RF
  { 86.0000f,   0.000f}, // RM
  { 65.5959f, -88.163f}  // RR
};

// UART mapping summary and hardware pin mappings (canonical LF,LM,LR,RF,RM,RR)
static constexpr const char* UART_MAP_SUMMARY = "LF=Serial8 LM=Serial3 LR=Serial5 RF=Serial7 RM=Serial6 RR=Serial2";
// TX pin mapping (derived from board wiring)
static constexpr int SERIAL_TX_PINS_CAN[NUM_LEGS_CFG] = {34, 14, 21, 28, 25, 8};
// 74HC126 buffer OE pins per leg (HIGH=TX drive, LOW=RX high-Z)
static constexpr int BUFFER_ENABLE_PINS[NUM_LEGS_CFG] = {36, 16, 22, 32, 9, 6};
// Default servo bus baud rate
static constexpr uint32_t SERVO_BAUD_DEFAULT = 115200u;
}
