#pragma once

#include <ch32v10x.h>

#define TKEY_CR    ADC1->CTLR1
#define TKEY_CH    ADC1->RSQR3
#define TKEY_SR    ADC1->RDATAR

class TouchButton {
    public:
        uint8_t channel;
        uint16_t threshold;
        TouchButton(uint8_t channel, uint16_t threshold);
        uint16_t read();
        bool is_pressed();
};