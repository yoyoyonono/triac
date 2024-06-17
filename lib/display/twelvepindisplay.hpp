#pragma once

#include <ch32v10x.h>

#include <cstddef>

#include "asciimap.hpp"
#include "display.hpp"

#define DEFAULT_CLOCK_PRINT_DELAY 500

enum display_mode {
    COMMON_CATHODE,
    COMMON_ANODE
};

class TwelvePinDisplay : public Display {
   private:
    GPIO_TypeDef *SEG_A_PORT;
    uint8_t SEG_A_PIN;
    GPIO_TypeDef *SEG_B_PORT;
    uint8_t SEG_B_PIN;
    GPIO_TypeDef *SEG_C_PORT;
    uint8_t SEG_C_PIN;
    GPIO_TypeDef *SEG_D_PORT;
    uint8_t SEG_D_PIN;
    GPIO_TypeDef *SEG_E_PORT;
    uint8_t SEG_E_PIN;
    GPIO_TypeDef *SEG_F_PORT;
    uint8_t SEG_F_PIN;
    GPIO_TypeDef *SEG_G_PORT;
    uint8_t SEG_G_PIN;
    GPIO_TypeDef *SEG_DP_PORT;
    uint8_t SEG_DP_PIN;
    GPIO_TypeDef *SEG_DIG1_PORT;
    uint8_t SEG_DIG1_PIN;
    GPIO_TypeDef *SEG_DIG2_PORT;
    uint8_t SEG_DIG2_PIN;
    GPIO_TypeDef *SEG_DIG3_PORT;
    uint8_t SEG_DIG3_PIN;
    GPIO_TypeDef *SEG_DIG4_PORT;
    uint8_t SEG_DIG4_PIN;
    display_mode mode;
    std::byte digits[4];
    uint8_t current_digit;
    AsciiMap asciiMap;

   public:
    TwelvePinDisplay(GPIO_TypeDef *SEG_A_PORT, uint8_t SEG_A_PIN,
                     GPIO_TypeDef *SEG_B_PORT, uint8_t SEG_B_PIN,
                     GPIO_TypeDef *SEG_C_PORT, uint8_t SEG_C_PIN,
                     GPIO_TypeDef *SEG_D_PORT, uint8_t SEG_D_PIN,
                     GPIO_TypeDef *SEG_E_PORT, uint8_t SEG_E_PIN,
                     GPIO_TypeDef *SEG_F_PORT, uint8_t SEG_F_PIN,
                     GPIO_TypeDef *SEG_G_PORT, uint8_t SEG_G_PIN,
                     GPIO_TypeDef *SEG_DP_PORT, uint8_t SEG_DP_PIN,
                     GPIO_TypeDef *SEG_DIG1_PORT, uint8_t SEG_DIG1_PIN,
                     GPIO_TypeDef *SEG_DIG2_PORT, uint8_t SEG_DIG2_PIN,
                     GPIO_TypeDef *SEG_DIG3_PORT, uint8_t SEG_DIG3_PIN,
                     GPIO_TypeDef *SEG_DIG4_PORT, uint8_t SEG_DIG4_PIN,
                     display_mode mode);
    void refresh() override;  // call frequently to refresh display
    void clear() override;
    void print(char *str) override;
    void printNumber(int16_t number, bool zeroPadding = false) override;
    void printTemperature(int16_t temperature) override;
    void printTime(uint8_t hour, uint8_t min) override;
    void printRaw(std::byte *raw) override;
    void setAll() override;
    void allOff() override;
};