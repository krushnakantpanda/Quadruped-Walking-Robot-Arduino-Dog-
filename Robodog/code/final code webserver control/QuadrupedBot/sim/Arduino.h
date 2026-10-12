#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

class IPAddress;

typedef uint8_t byte;
typedef uint16_t word;

#define PI 3.1415926535897932384626433832795f
#define F(x) x
#define PROGMEM

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void yield();

class String {
public:
  String() {}
  String(const char* s) {}
  String(char c) {}
  String(int v) {}
  String(unsigned int v) {}
  String(long v) {}
  String(unsigned long v) {}
  String(float v, int dec) {}
  String(double v, int dec) {}
  const char* c_str() const { return ""; }
  int length() const { return 0; }
  bool operator==(const char* s) const { return false; }
  String& operator+=(const char* s) { return *this; }
  String& operator+=(const String& s) { return *this; }
  String& operator+=(char c) { return *this; }
  void reserve(size_t n) {}
};
String operator+(const String& a, const String& b);

// Arduino's constrain is a macro (works across mixed numeric types)
#define constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class SerialClass {
public:
  void begin(long baud) {}
  void print(const char* s) {}
  void print(int v) {}
  void print(unsigned int v) {}
  void print(float v) {}
  void print(double v) {}
  void println() {}
  void println(const char* s) {}
  void println(int v) {}
  void println(unsigned int v) {}
  void println(float v) {}
  void println(double v) {}
  void println(const IPAddress& ip) {}
};
extern SerialClass Serial;
