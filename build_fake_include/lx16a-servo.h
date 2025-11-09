// Minimal stub of lx16a-servo library for host-side compilation/testing.
// Real firmware links against actual library providing full implementations.
#pragma once
#include <stdint.h>

// Command constant used in firmware send path
#ifndef LX16A_SERVO_MOVE_TIME_WRITE
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#endif

class LX16ABus {
public:
	void begin(void* serial, int txPin, int oePin) { (void)serial; (void)txPin; (void)oePin; }
	int write(uint8_t cmd, const uint8_t* params, int len, uint8_t id) { (void)cmd; (void)params; (void)len; (void)id; return 0; }
};

class LX16AServo {
public:
	LX16AServo(LX16ABus* bus, uint8_t id) { (void)bus; (void)id; }
	void enable() {}
	void disable() {}
	int32_t pos_read() { return 12000; } // centidegrees mid-range
};

// Hardware angle offset helpers (stubbed)
inline int angle_offset_read(uint8_t id) { (void)id; return 0; }
inline bool angle_offset_adjust(uint8_t id, int8_t delta) { (void)id; (void)delta; return true; }
inline bool angle_offset_write(uint8_t id) { (void)id; return true; }
