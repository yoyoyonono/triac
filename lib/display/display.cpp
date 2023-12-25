#include "display.hpp"

#include <cstddef>

#include "asciimap.hpp"
#include "util.hpp"

Display::Display(GPIO_TypeDef *SEG_A_PORT, uint8_t SEG_A_PIN,
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
                 display_mode mode) {
    this->SEG_A_PORT = SEG_A_PORT;
    this->SEG_A_PIN = SEG_A_PIN;
    this->SEG_B_PORT = SEG_B_PORT;
    this->SEG_B_PIN = SEG_B_PIN;
    this->SEG_C_PORT = SEG_C_PORT;
    this->SEG_C_PIN = SEG_C_PIN;
    this->SEG_D_PORT = SEG_D_PORT;
    this->SEG_D_PIN = SEG_D_PIN;
    this->SEG_E_PORT = SEG_E_PORT;
    this->SEG_E_PIN = SEG_E_PIN;
    this->SEG_F_PORT = SEG_F_PORT;
    this->SEG_F_PIN = SEG_F_PIN;
    this->SEG_G_PORT = SEG_G_PORT;
    this->SEG_G_PIN = SEG_G_PIN;
    this->SEG_DP_PORT = SEG_DP_PORT;
    this->SEG_DP_PIN = SEG_DP_PIN;
    this->SEG_DIG1_PORT = SEG_DIG1_PORT;
    this->SEG_DIG1_PIN = SEG_DIG1_PIN;
    this->SEG_DIG2_PORT = SEG_DIG2_PORT;
    this->SEG_DIG2_PIN = SEG_DIG2_PIN;
    this->SEG_DIG3_PORT = SEG_DIG3_PORT;
    this->SEG_DIG3_PIN = SEG_DIG3_PIN;
    this->SEG_DIG4_PORT = SEG_DIG4_PORT;
    this->SEG_DIG4_PIN = SEG_DIG4_PIN;
    this->mode = mode;

    // Set up GPIOs
    pinMode(SEG_A_PORT, SEG_A_PIN, OUTPUT);
    pinMode(SEG_B_PORT, SEG_B_PIN, OUTPUT);
    pinMode(SEG_C_PORT, SEG_C_PIN, OUTPUT);
    pinMode(SEG_D_PORT, SEG_D_PIN, OUTPUT);
    pinMode(SEG_E_PORT, SEG_E_PIN, OUTPUT);
    pinMode(SEG_F_PORT, SEG_F_PIN, OUTPUT);
    pinMode(SEG_G_PORT, SEG_G_PIN, OUTPUT);
    pinMode(SEG_DP_PORT, SEG_DP_PIN, OUTPUT);
    pinMode(SEG_DIG1_PORT, SEG_DIG1_PIN, OUTPUT);
    pinMode(SEG_DIG2_PORT, SEG_DIG2_PIN, OUTPUT);
    pinMode(SEG_DIG3_PORT, SEG_DIG3_PIN, OUTPUT);
    pinMode(SEG_DIG4_PORT, SEG_DIG4_PIN, OUTPUT);

    digits[0] = std::byte(0);
    digits[1] = std::byte(0);
    digits[2] = std::byte(0);
    digits[3] = std::byte(0);

    current_digit = 0;

    asciiMap = AsciiMap();
}

void Display::refresh() {
    if (mode == COMMON_CATHODE) {
        // clear anodes

        digitalWrite(SEG_A_PORT, SEG_A_PIN, LOW);
        digitalWrite(SEG_B_PORT, SEG_B_PIN, LOW);
        digitalWrite(SEG_C_PORT, SEG_C_PIN, LOW);
        digitalWrite(SEG_D_PORT, SEG_D_PIN, LOW);
        digitalWrite(SEG_E_PORT, SEG_E_PIN, LOW);
        digitalWrite(SEG_F_PORT, SEG_F_PIN, LOW);
        digitalWrite(SEG_G_PORT, SEG_G_PIN, LOW);
        digitalWrite(SEG_DP_PORT, SEG_DP_PIN, LOW);

        // set cathodes
        switch (current_digit) {
            case 0:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, LOW);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, HIGH);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, HIGH);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, HIGH);
                break;

            case 1:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, HIGH);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, LOW);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, HIGH);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, HIGH);
                break;

            case 2:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, HIGH);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, HIGH);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, LOW);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, HIGH);
                break;

            case 3:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, HIGH);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, HIGH);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, HIGH);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, LOW);
                break;
        }

        // bit order is dp g f e d c b a

        digitalWrite(SEG_A_PORT, SEG_A_PIN, (digits[current_digit] & std::byte(0b00000001)) == std::byte(0b00000001));
        digitalWrite(SEG_B_PORT, SEG_B_PIN, (digits[current_digit] & std::byte(0b00000010)) == std::byte(0b00000010));
        digitalWrite(SEG_C_PORT, SEG_C_PIN, (digits[current_digit] & std::byte(0b00000100)) == std::byte(0b00000100));
        digitalWrite(SEG_D_PORT, SEG_D_PIN, (digits[current_digit] & std::byte(0b00001000)) == std::byte(0b00001000));
        digitalWrite(SEG_E_PORT, SEG_E_PIN, (digits[current_digit] & std::byte(0b00010000)) == std::byte(0b00010000));
        digitalWrite(SEG_F_PORT, SEG_F_PIN, (digits[current_digit] & std::byte(0b00100000)) == std::byte(0b00100000));
        digitalWrite(SEG_G_PORT, SEG_G_PIN, (digits[current_digit] & std::byte(0b01000000)) == std::byte(0b01000000));
        digitalWrite(SEG_DP_PORT, SEG_DP_PIN, (digits[current_digit] & std::byte(0b10000000)) == std::byte(0b10000000));

    } else {
        switch (current_digit) {
            case 0:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, HIGH);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, LOW);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, LOW);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, LOW);
                break;

            case 1:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, LOW);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, HIGH);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, LOW);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, LOW);
                break;

            case 2:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, LOW);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, LOW);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, HIGH);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, LOW);
                break;

            case 3:
                digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, LOW);
                digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, LOW);
                digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, LOW);
                digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, HIGH);
                break;
        }

        // bit order is dp g f e d c b a

        digitalWrite(SEG_A_PORT, SEG_A_PIN, (digits[current_digit] & std::byte(0b00000001)) != std::byte(0b00000001));
        digitalWrite(SEG_B_PORT, SEG_B_PIN, (digits[current_digit] & std::byte(0b00000010)) != std::byte(0b00000010));
        digitalWrite(SEG_C_PORT, SEG_C_PIN, (digits[current_digit] & std::byte(0b00000100)) != std::byte(0b00000100));
        digitalWrite(SEG_D_PORT, SEG_D_PIN, (digits[current_digit] & std::byte(0b00001000)) != std::byte(0b00001000));
        digitalWrite(SEG_E_PORT, SEG_E_PIN, (digits[current_digit] & std::byte(0b00010000)) != std::byte(0b00010000));
        digitalWrite(SEG_F_PORT, SEG_F_PIN, (digits[current_digit] & std::byte(0b00100000)) != std::byte(0b00100000));
        digitalWrite(SEG_G_PORT, SEG_G_PIN, (digits[current_digit] & std::byte(0b01000000)) != std::byte(0b01000000));
        digitalWrite(SEG_DP_PORT, SEG_DP_PIN, (digits[current_digit] & std::byte(0b10000000)) != std::byte(0b10000000));
    }
    // increment digit
    current_digit++;
    if (current_digit > 3) {
        current_digit = 0;
    }
}

void Display::clear() {
    digits[0] = std::byte(0);
    digits[1] = std::byte(0);
    digits[2] = std::byte(0);
    digits[3] = std::byte(0);
    for (int i = 0; i < 4; i++) {
        refresh();
    }
}

void Display::print(char *str) {
    // set byte values based on ascii lookup
    for (int i = 0; i < 4; i++) {
        digits[i] = std::byte(asciiMap.map[str[i] - 32]);
    }
}

void Display::printNumber(int16_t number, bool zeroPadding) {
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

void Display::printTemperature(int16_t temperature) {
    printNumber(temperature * 10);
    digits[3] = std::byte(0b01100011);  // degree symbol
}

void Display::printTime(uint8_t hour, uint8_t min) {
    printNumber(hour * 100 + min, true);
    digits[1] |= std::byte(0b10000000);  // colon
}

void Display::printRaw(std::byte *raw) {
    for (int i = 0; i < 4; i++) {
        digits[i] = raw[i];
    }
}

void Display::setAll() {
    digits[0] = std::byte(0b11111111);
    digits[1] = std::byte(0b11111111);
    digits[2] = std::byte(0b11111111);
    digits[3] = std::byte(0b11111111);
}

void Display::allOff() {
    if (mode == COMMON_CATHODE) {
        digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, HIGH);
        digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, HIGH);
        digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, HIGH);
        digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, HIGH);
    } else {
        digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, LOW);
        digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, LOW);
        digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, LOW);
        digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, LOW);
    }
}