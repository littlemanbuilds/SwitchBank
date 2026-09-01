#pragma once
#include <cstdint>
#include <cstddef>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define BIN 2
inline std::uint32_t millis() { return 0u; }
inline void delay(unsigned long) {}
inline void pinMode(std::uint8_t, int) {}
inline int digitalRead(std::uint8_t) { return HIGH; }
struct SerialStub {
    void begin(unsigned long) {}
    template <typename T> void print(const T&) {}
    template <typename T> void print(const T&, int) {}
    template <typename T> void println(const T&) {}
    void println() {}
};
static SerialStub Serial;
