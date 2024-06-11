#pragma once

#include <ch32v10x.h>

#define TKEY_CR ADC1->CTLR1
#define TKEY_CH ADC1->RSQR3
#define TKEY_SR ADC1->RDATAR

class TouchButton {
   public:
    uint8_t channel;
    TouchButton(uint8_t channel);
    uint32_t read();
    bool is_pressed();
    uint32_t callibrate();
    uint32_t normal_value_average;

   private:
    bool last_read;
    uint32_t last_value;
    uint32_t history[100];
};