#pragma once
#include <stdint.h>
#include <stddef.h>
// Minimal Arduino.h stubs for syntax checking
using uint8_t = ::uint8_t;
using uint16_t = ::uint16_t;
using uint32_t = ::uint32_t;
using int16_t = ::int16_t;
using int32_t = ::int32_t;

class HardwareSerial {
public:
  void begin(unsigned long) {}
  size_t write(const uint8_t*, size_t) { return 0; }
  size_t write(uint8_t) { return 0; }
  int available();
  int read();
  void print(const char* s);
  void print(int v);
  void println(char c);
  void flush();
};

extern HardwareSerial Serial;

#define F(x) (x)
#define HIGH 0x1
#define LOW  0x0

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline unsigned long millis() { return 0; }
inline void delay(unsigned long) {}

