#include <ch32v10x.h>
#include <debug.h>
#include "tkey.hpp"
#include "stdlib.h"

TouchButton::TouchButton(uint8_t channel, uint16_t threshold) : channel(channel), threshold(threshold) {

}

uint16_t TouchButton::read() {
    u16 val;

    TKEY_CH = channel; // TouchKey Channel

    while(!(TKEY_CR & 0x08000000))
        ;
    val = (u16)TKEY_SR;

    return val; 
}

bool TouchButton::is_pressed() {
    return read() < threshold;
}
