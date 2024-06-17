#include "eightpindisplay.hpp"

#include <cstddef>

#include "asciimap.hpp"
#include "util.hpp"

EightPinDisplay::EightPinDisplay(GPIO_TypeDef *PIN_0_PORT, uint8_t PIN_0_PIN,
                                 GPIO_TypeDef *PIN_1_PORT, uint8_t PIN_1_PIN,
                                 GPIO_TypeDef *PIN_2_PORT, uint8_t PIN_2_PIN,
                                 GPIO_TypeDef *PIN_3_PORT, uint8_t PIN_3_PIN,
                                 GPIO_TypeDef *PIN_4_PORT, uint8_t PIN_4_PIN,
                                 GPIO_TypeDef *PIN_5_PORT, uint8_t PIN_5_PIN,
                                 GPIO_TypeDef *PIN_6_PORT, uint8_t PIN_6_PIN,
                                 GPIO_TypeDef *PIN_7_PORT, uint8_t PIN_7_PIN)
    : PIN_0_PORT(PIN_0_PORT), PIN_0_PIN(PIN_0_PIN), PIN_1_PORT(PIN_1_PORT), PIN_1_PIN(PIN_1_PIN), PIN_2_PORT(PIN_2_PORT), PIN_2_PIN(PIN_2_PIN), PIN_3_PORT(PIN_3_PORT), PIN_3_PIN(PIN_3_PIN), PIN_4_PORT(PIN_4_PORT), PIN_4_PIN(PIN_4_PIN), PIN_5_PORT(PIN_5_PORT), PIN_5_PIN(PIN_5_PIN), PIN_6_PORT(PIN_6_PORT), PIN_6_PIN(PIN_6_PIN), PIN_7_PORT(PIN_7_PORT), PIN_7_PIN(PIN_7_PIN) {
    digits[0] = std::byte(0);
    digits[1] = std::byte(0);
    digits[2] = std::byte(0);
    digits[3] = std::byte(0);

    current_step = 0;

    asciiMap = AsciiMap();
}

void EightPinDisplay::all_input() {
    pinMode(PIN_0_PORT, PIN_0_PIN, INPUT);
    pinMode(PIN_1_PORT, PIN_1_PIN, INPUT);
    pinMode(PIN_2_PORT, PIN_2_PIN, INPUT);
    pinMode(PIN_3_PORT, PIN_3_PIN, INPUT);
    pinMode(PIN_4_PORT, PIN_4_PIN, INPUT);
    pinMode(PIN_5_PORT, PIN_5_PIN, INPUT);
    pinMode(PIN_6_PORT, PIN_6_PIN, INPUT);
    pinMode(PIN_7_PORT, PIN_7_PIN, INPUT);
}

void EightPinDisplay::refresh() {
    all_input();
    switch (current_step >> 2) {
        case 0:
            pinMode(PIN_0_PORT, PIN_0_PIN, OUTPUT);
            digitalWrite(PIN_0_PORT, PIN_0_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[0] & std::byte(0b00010000)) == std::byte(0b00010000)) {
                        pinMode(PIN_4_PORT, PIN_4_PIN, OUTPUT);
                        digitalWrite(PIN_4_PORT, PIN_4_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[1] & std::byte(0b00010000)) == std::byte(0b00010000)) {
                        pinMode(PIN_5_PORT, PIN_5_PIN, OUTPUT);
                        digitalWrite(PIN_5_PORT, PIN_5_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[2] & std::byte(0b00010000)) == std::byte(0b00010000)) {
                        pinMode(PIN_6_PORT, PIN_6_PIN, OUTPUT);
                        digitalWrite(PIN_6_PORT, PIN_6_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[3] & std::byte(0b00010000)) == std::byte(0b00010000)) {
                        pinMode(PIN_7_PORT, PIN_7_PIN, OUTPUT);
                        digitalWrite(PIN_7_PORT, PIN_7_PIN, LOW);
                    }
                    break;
            }
            break;
        case 1:
            pinMode(PIN_1_PORT, PIN_1_PIN, OUTPUT);
            digitalWrite(PIN_1_PORT, PIN_1_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[0] & std::byte(0b00100000)) == std::byte(0b00100000)) {
                        pinMode(PIN_4_PORT, PIN_4_PIN, OUTPUT);
                        digitalWrite(PIN_4_PORT, PIN_4_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[1] & std::byte(0b00100000)) == std::byte(0b00100000)) {
                        pinMode(PIN_5_PORT, PIN_5_PIN, OUTPUT);
                        digitalWrite(PIN_5_PORT, PIN_5_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[2] & std::byte(0b00100000)) == std::byte(0b00100000)) {
                        pinMode(PIN_6_PORT, PIN_6_PIN, OUTPUT);
                        digitalWrite(PIN_6_PORT, PIN_6_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[3] & std::byte(0b00100000)) == std::byte(0b00100000)) {
                        pinMode(PIN_7_PORT, PIN_7_PIN, OUTPUT);
                        digitalWrite(PIN_7_PORT, PIN_7_PIN, LOW);
                    }
                    break;
            }
            break;
        case 2:
            pinMode(PIN_2_PORT, PIN_2_PIN, OUTPUT);
            digitalWrite(PIN_2_PORT, PIN_2_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[0] & std::byte(0b01000000)) == std::byte(0b01000000)) {
                        pinMode(PIN_4_PORT, PIN_4_PIN, OUTPUT);
                        digitalWrite(PIN_4_PORT, PIN_4_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[1] & std::byte(0b01000000)) == std::byte(0b01000000)) {
                        pinMode(PIN_5_PORT, PIN_5_PIN, OUTPUT);
                        digitalWrite(PIN_5_PORT, PIN_5_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[2] & std::byte(0b01000000)) == std::byte(0b01000000)) {
                        pinMode(PIN_6_PORT, PIN_6_PIN, OUTPUT);
                        digitalWrite(PIN_6_PORT, PIN_6_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[3] & std::byte(0b01000000)) == std::byte(0b01000000)) {
                        pinMode(PIN_7_PORT, PIN_7_PIN, OUTPUT);
                        digitalWrite(PIN_7_PORT, PIN_7_PIN, LOW);
                    }
                    break;
            }
            break;
        case 3:
            pinMode(PIN_3_PORT, PIN_3_PIN, OUTPUT);
            digitalWrite(PIN_3_PORT, PIN_3_PIN, HIGH);
            if ((digits[1] & std::byte(0b10000000)) == std::byte(0b10000000)) {
                pinMode(PIN_5_PORT, PIN_5_PIN, OUTPUT);
                digitalWrite(PIN_5_PORT, PIN_5_PIN, LOW);
            }
            if (colon) {
                pinMode(PIN_7_PORT, PIN_7_PIN, OUTPUT);
                digitalWrite(PIN_7_PORT, PIN_7_PIN, LOW);
            }
            break;
        case 4:
            pinMode(PIN_4_PORT, PIN_4_PIN, OUTPUT);
            digitalWrite(PIN_4_PORT, PIN_4_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[0] & std::byte(0b00000001)) == std::byte(0b00000001)) {
                        pinMode(PIN_0_PORT, PIN_0_PIN, OUTPUT);
                        digitalWrite(PIN_0_PORT, PIN_0_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[0] & std::byte(0b00000010)) == std::byte(0b00000010)) {
                        pinMode(PIN_1_PORT, PIN_1_PIN, OUTPUT);
                        digitalWrite(PIN_1_PORT, PIN_1_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[0] & std::byte(0b00000100)) == std::byte(0b00000100)) {
                        pinMode(PIN_2_PORT, PIN_2_PIN, OUTPUT);
                        digitalWrite(PIN_2_PORT, PIN_2_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[0] & std::byte(0b00001000)) == std::byte(0b00001000)) {
                        pinMode(PIN_3_PORT, PIN_3_PIN, OUTPUT);
                        digitalWrite(PIN_3_PORT, PIN_3_PIN, LOW);
                    }
                    break;
            }
            break;
        case 5:
            pinMode(PIN_5_PORT, PIN_5_PIN, OUTPUT);
            digitalWrite(PIN_5_PORT, PIN_5_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[1] & std::byte(0b00000001)) == std::byte(0b00000001)) {
                        pinMode(PIN_0_PORT, PIN_0_PIN, OUTPUT);
                        digitalWrite(PIN_0_PORT, PIN_0_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[1] & std::byte(0b00000010)) == std::byte(0b00000010)) {
                        pinMode(PIN_1_PORT, PIN_1_PIN, OUTPUT);
                        digitalWrite(PIN_1_PORT, PIN_1_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[1] & std::byte(0b00000100)) == std::byte(0b00000100)) {
                        pinMode(PIN_2_PORT, PIN_2_PIN, OUTPUT);
                        digitalWrite(PIN_2_PORT, PIN_2_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[1] & std::byte(0b00001000)) == std::byte(0b00001000)) {
                        pinMode(PIN_3_PORT, PIN_3_PIN, OUTPUT);
                        digitalWrite(PIN_3_PORT, PIN_3_PIN, LOW);
                    }
                    break;
            }
            break;
        case 6:
            pinMode(PIN_6_PORT, PIN_6_PIN, OUTPUT);
            digitalWrite(PIN_6_PORT, PIN_6_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[2] & std::byte(0b00000001)) == std::byte(0b00000001)) {
                        pinMode(PIN_0_PORT, PIN_0_PIN, OUTPUT);
                        digitalWrite(PIN_0_PORT, PIN_0_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[2] & std::byte(0b00000010)) == std::byte(0b00000010)) {
                        pinMode(PIN_1_PORT, PIN_1_PIN, OUTPUT);
                        digitalWrite(PIN_1_PORT, PIN_1_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[2] & std::byte(0b00000100)) == std::byte(0b00000100)) {
                        pinMode(PIN_2_PORT, PIN_2_PIN, OUTPUT);
                        digitalWrite(PIN_2_PORT, PIN_2_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[2] & std::byte(0b00001000)) == std::byte(0b00001000)) {
                        pinMode(PIN_3_PORT, PIN_3_PIN, OUTPUT);
                        digitalWrite(PIN_3_PORT, PIN_3_PIN, LOW);
                    }
                    break;
            }
            break;
        case 7:
            pinMode(PIN_7_PORT, PIN_7_PIN, OUTPUT);
            digitalWrite(PIN_7_PORT, PIN_7_PIN, HIGH);
            switch (current_step & 3) {
                case 0:
                    if ((digits[3] & std::byte(0b00000001)) == std::byte(0b00000001)) {
                        pinMode(PIN_0_PORT, PIN_0_PIN, OUTPUT);
                        digitalWrite(PIN_0_PORT, PIN_0_PIN, LOW);
                    }
                    break;
                case 1:
                    if ((digits[3] & std::byte(0b00000010)) == std::byte(0b00000010)) {
                        pinMode(PIN_1_PORT, PIN_1_PIN, OUTPUT);
                        digitalWrite(PIN_1_PORT, PIN_1_PIN, LOW);
                    }
                    break;
                case 2:
                    if ((digits[3] & std::byte(0b00000100)) == std::byte(0b00000100)) {
                        pinMode(PIN_2_PORT, PIN_2_PIN, OUTPUT);
                        digitalWrite(PIN_2_PORT, PIN_2_PIN, LOW);
                    }
                    break;
                case 3:
                    if ((digits[3] & std::byte(0b00001000)) == std::byte(0b00001000)) {
                        pinMode(PIN_3_PORT, PIN_3_PIN, OUTPUT);
                        digitalWrite(PIN_3_PORT, PIN_3_PIN, LOW);
                    }
                    break;
            }
            break;
    }

    current_step++;
    if (current_step == 32) {
        current_step = 0;
    }
}

void EightPinDisplay::clear() {
    digits[0] = std::byte(0);
    digits[1] = std::byte(0);
    digits[2] = std::byte(0);
    digits[3] = std::byte(0);
}

void EightPinDisplay::print(char *str) {
    colon = false;
    for (int i = 0; i < 4; i++) {
        digits[i] = std::byte(asciiMap.map[str[i] - 32]);
    }
}

void EightPinDisplay::printNumber(int16_t number, bool zeroPadding) {
    char str[4];
    if (number < 0) {
        number = -number;
    }
    if (number > 9999) {
        number = 9999;
    }
    str[0] = (number / 1000) + 48;
    str[1] = ((number % 1000) / 100) + 48;
    str[2] = ((number % 100) / 10) + 48;
    str[3] = (number % 10) + 48;
    if (!zeroPadding) {
        if (str[0] == 48) {
            str[0] = 32;
        }
        if (str[0] == 48 && str[1] == 48) {
            str[1] = 32;
        }
        if (str[0] == 48 && str[1] == 48 && str[2] == 48) {
            str[2] = 32;
        }
    }
    print(str);
}

void EightPinDisplay::printTemperature(int16_t temperature) {
    colon = false;
    printNumber(temperature * 10, true);
    digits[3] = std::byte(0b01100011);
}

void EightPinDisplay::printTime(uint8_t hour, uint8_t min) {
    printNumber(hour * 100 + min, true);
    colon = true;
}

void EightPinDisplay::printRaw(std::byte *raw) {
    for (int i = 0; i < 4; i++) {
        digits[i] = raw[i];
    }
}

void EightPinDisplay::setAll() {
    digits[0] = std::byte(0b11111111);
    digits[1] = std::byte(0b11111111);
    digits[2] = std::byte(0b11111111);
    digits[3] = std::byte(0b11111111);
    colon = true;
}

void EightPinDisplay::allOff() {
    digits[0] = std::byte(0);
    digits[1] = std::byte(0);
    digits[2] = std::byte(0);
    digits[3] = std::byte(0);
    colon = false;
}