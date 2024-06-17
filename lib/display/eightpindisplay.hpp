#pragma once

#include <ch32v10x.h>

#include <cstddef>

#include "asciimap.hpp"
#include "display.hpp"

#define DEFAULT_CLOCK_PRINT_DELAY 500

class EightPinDisplay : public Display {
   private:
    GPIO_TypeDef *PIN_0_PORT;
    uint8_t PIN_0_PIN;
    GPIO_TypeDef *PIN_1_PORT;
    uint8_t PIN_1_PIN;
    GPIO_TypeDef *PIN_2_PORT;
    uint8_t PIN_2_PIN;
    GPIO_TypeDef *PIN_3_PORT;
    uint8_t PIN_3_PIN;
    GPIO_TypeDef *PIN_4_PORT;
    uint8_t PIN_4_PIN;
    GPIO_TypeDef *PIN_5_PORT;
    uint8_t PIN_5_PIN;
    GPIO_TypeDef *PIN_6_PORT;
    uint8_t PIN_6_PIN;
    GPIO_TypeDef *PIN_7_PORT;
    uint8_t PIN_7_PIN;
    
    std::byte digits[4];
    uint8_t current_step;
    AsciiMap asciiMap;
    bool colon;

    void all_input();

   public:
    EightPinDisplay(GPIO_TypeDef *PIN_0_PORT, uint8_t PIN_0_PIN,
                    GPIO_TypeDef *PIN_1_PORT, uint8_t PIN_1_PIN,
                    GPIO_TypeDef *PIN_2_PORT, uint8_t PIN_2_PIN,
                    GPIO_TypeDef *PIN_3_PORT, uint8_t PIN_3_PIN,
                    GPIO_TypeDef *PIN_4_PORT, uint8_t PIN_4_PIN,
                    GPIO_TypeDef *PIN_5_PORT, uint8_t PIN_5_PIN,
                    GPIO_TypeDef *PIN_6_PORT, uint8_t PIN_6_PIN,
                    GPIO_TypeDef *PIN_7_PORT, uint8_t PIN_7_PIN);
    void refresh() override;
    void clear() override;
    void print(char *str) override;
    void printNumber(int16_t number, bool zeroPadding = false) override;
    void printTemperature(int16_t temperature) override;
    void printTime(uint8_t hour, uint8_t min) override;
    void printRaw(std::byte *raw) override;
    void setAll() override;
    void allOff() override;
};