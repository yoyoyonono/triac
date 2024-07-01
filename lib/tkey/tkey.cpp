#include "tkey.hpp"

#include <ch32v10x.h>
#include <debug.h>

#include "stdlib.h"
#include "util.hpp"

TouchButton::TouchButton(uint8_t channel) : channel(channel) {
    last_read = false;
    last_read_value = 0;
}

uint32_t TouchButton::read() {
    u32 val;

    TKEY_CH = channel;  // TouchKey Channel

    while (!(TKEY_CR & 0x08000000));
    val = (u32)TKEY_SR;

    last_read_value = val;

    return val;
}

bool TouchButton::is_pressed() {
    uint32_t new_value = read();

    // calculate average
    uint32_t average = 0;
    for (uint8_t i = 0; i < 100; i++) {
        average += history[i];
    }
    average /= 100;

    if (abs((int16_t)average - (int16_t)new_value) > 100) {
        delay(20);
        return true;
    } else {
        // rotate history
        for (uint8_t i = 0; i < 99; i++) {
            history[i] = history[i + 1];
        }
        history[99] = new_value;
        return false;
    }
}

uint32_t TouchButton::callibrate() {
    for (uint8_t i = 0; i < 100; i++) {
        normal_value_average += read();
    }
    normal_value_average /= 100;
    for (uint8_t i = 0; i < 100; i++) {
        history[i] = normal_value_average;
    }
    return normal_value_average;
}