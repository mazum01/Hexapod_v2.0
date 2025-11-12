// Minimal stub of lx16a-servo library for host-side compilation/testing.
// Real firmware links against actual library providing full implementations.
#pragma once
#include <stdint.h>

// Command constants minimally used by firmware
#ifndef LX16A_SERVO_MOVE_TIME_WRITE
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#endif
#ifndef LX16A_SERVO_ANGLE_OFFSET_ADJUST
#define LX16A_SERVO_ANGLE_OFFSET_ADJUST 17
#endif
#ifndef LX16A_SERVO_ANGLE_OFFSET_WRITE
#define LX16A_SERVO_ANGLE_OFFSET_WRITE 18
#endif
#ifndef LX16A_SERVO_ANGLE_OFFSET_READ
#define LX16A_SERVO_ANGLE_OFFSET_READ 19
#endif

class LX16ABus {
public:
	void begin(void* serial, int txPin, int oePin) { (void)serial; (void)txPin; (void)oePin; }
	int write(uint8_t cmd, const uint8_t* params, int len, uint8_t id) { (void)cmd; (void)params; (void)len; (void)id; return 0; }
	bool read(uint8_t cmd, uint8_t* params, int len, uint8_t id) { (void)cmd; (void)id; for (int i=0;i<len;++i) params[i]=0; return true; }
};

class LX16AServo {
public:
	uint8_t _id;
	LX16AServo(LX16ABus* bus, uint8_t id) { (void)bus; _id = id; }
	void enable() {}
	void disable() {}
	int32_t pos_read() { return 12000; } // centidegrees mid-range
	// Minimal offset API for host builds
	void angle_offset_adjust(int16_t /*cd*/) {}
	void angle_offset_save() {}
	int16_t read_angle_offset() { return 0; }
};

