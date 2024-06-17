#pragma once

#include <ch32v10x.h>

#include <cstddef>

class Display {
   public:
    Display() = default;
    virtual void refresh() = 0;
    virtual void clear() = 0;
    virtual void print(char *str) = 0;
    virtual void printNumber(int16_t number, bool zeroPadding = false) = 0;
    virtual void printTemperature(int16_t temperature) = 0;
    virtual void printTime(uint8_t hour, uint8_t min) = 0;
    virtual void printRaw(std::byte *raw) = 0;
    virtual void setAll() = 0;
    virtual void allOff() = 0;
};