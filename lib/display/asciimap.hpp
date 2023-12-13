#pragma once

#include <ch32v10x.h>


/* Prints raw (encoded) bytes to the display
*         A
*       ___
*  * F |   | B
* X     -G-
*  * E |   | C
*       ___
*        D
* Bit:      76543210
* Segment:  XGFEDCBA
*
* For example to print an H, you would set bits BCEFG, this gives B01110110 in binary or 118 in decimal or 0x76 in HEX.
* Bit 7 (X) only applies to the second digit and sets the colon
*/

// ASCII MAPPINGS
#define DISPLAY_CHAR_SPACE       0b00000000 // 32  (ASCII)
#define DISPLAY_CHAR_EXC         0b00000110
#define DISPLAY_CHAR_D_QUOTE     0b00100010
#define DISPLAY_CHAR_POUND       0b01110110
#define DISPLAY_CHAR_DOLLAR      0b01101101
#define DISPLAY_CHAR_PERC        0b00100100
#define DISPLAY_CHAR_AMP         0b01111111
#define DISPLAY_CHAR_S_QUOTE     0b00100000
#define DISPLAY_CHAR_L_BRACKET   0b00111001
#define DISPLAY_CHAR_R_BRACKET   0b00001111
#define DISPLAY_CHAR_STAR        0b01011100
#define DISPLAY_CHAR_PLUS        0b01010000
#define DISPLAY_CHAR_COMMA       0b00010000
#define DISPLAY_CHAR_MIN         0b01000000
#define DISPLAY_CHAR_DOT         0b10000000
#define DISPLAY_CHAR_F_SLASH     0b00000110
#define DISPLAY_CHAR_0           0b00111111   // 48
#define DISPLAY_CHAR_1           0b00000110
#define DISPLAY_CHAR_2           0b01011011
#define DISPLAY_CHAR_3           0b01001111
#define DISPLAY_CHAR_4           0b01100110
#define DISPLAY_CHAR_5           0b01101101
#define DISPLAY_CHAR_6           0b01111101
#define DISPLAY_CHAR_7           0b00000111
#define DISPLAY_CHAR_8           0b01111111
#define DISPLAY_CHAR_9           0b01101111
#define DISPLAY_CHAR_COLON       0b10000000
#define DISPLAY_CHAR_S_COLON     0b00110000
#define DISPLAY_CHAR_LESS        0b01011000
#define DISPLAY_CHAR_EQUAL       0b01001000
#define DISPLAY_CHAR_GREAT       0b01001100
#define DISPLAY_CHAR_QUEST       0b01010011
#define DISPLAY_CHAR_AT          0b01011111
#define DISPLAY_CHAR_A           0b01110111 // 65  (ASCII)
#define DISPLAY_CHAR_B           0b01111111
#define DISPLAY_CHAR_C           0b00111001
#define DISPLAY_CHAR_D           DISPLAY_CHAR_d
#define DISPLAY_CHAR_E           0b01111001
#define DISPLAY_CHAR_F           0b01110001
#define DISPLAY_CHAR_G           0b00111101
#define DISPLAY_CHAR_H           0b01110110
#define DISPLAY_CHAR_I           0b00000110
#define DISPLAY_CHAR_J           0b00001110
#define DISPLAY_CHAR_K           0b01110101
#define DISPLAY_CHAR_L           0b00111000
#define DISPLAY_CHAR_M           0b00010101
#define DISPLAY_CHAR_N           0b00110111
#define DISPLAY_CHAR_O           0b00111111
#define DISPLAY_CHAR_P           0b01110011
#define DISPLAY_CHAR_Q           0b01100111
#define DISPLAY_CHAR_R           0b00110011
#define DISPLAY_CHAR_S           0b01101101
#define DISPLAY_CHAR_T           DISPLAY_CHAR_t
#define DISPLAY_CHAR_U           0b00111110
#define DISPLAY_CHAR_V           0b00011100
#define DISPLAY_CHAR_W           0b00101010
#define DISPLAY_CHAR_X           DISPLAY_CHAR_H
#define DISPLAY_CHAR_Y           0b01101110
#define DISPLAY_CHAR_Z           0b01011011
#define DISPLAY_CHAR_L_S_BRACKET 0b00111001 // 91 (ASCII)
#define DISPLAY_CHAR_B_SLASH     0b00110000
#define DISPLAY_CHAR_R_S_BRACKET 0b00001111
#define DISPLAY_CHAR_A_CIRCUM    0b00010011
#define DISPLAY_CHAR_UNDERSCORE  0b00001000
#define DISPLAY_CHAR_A_GRAVE     0b00010000
#define DISPLAY_CHAR_a           0b01011111 // 97 (ASCII)
#define DISPLAY_CHAR_b           0b01111100
#define DISPLAY_CHAR_c           0b01011000
#define DISPLAY_CHAR_d           0b01011110
#define DISPLAY_CHAR_e           0b01111011
#define DISPLAY_CHAR_f           DISPLAY_CHAR_F
#define DISPLAY_CHAR_g           0b01101111
#define DISPLAY_CHAR_h           0b01110100
#define DISPLAY_CHAR_i           0b00000100
#define DISPLAY_CHAR_j           0b00001100
#define DISPLAY_CHAR_k           DISPLAY_CHAR_K
#define DISPLAY_CHAR_l           0b00110000
#define DISPLAY_CHAR_m           DISPLAY_CHAR_M
#define DISPLAY_CHAR_n           0b01010100
#define DISPLAY_CHAR_o           0b01011100
#define DISPLAY_CHAR_p           DISPLAY_CHAR_P
#define DISPLAY_CHAR_q           DISPLAY_CHAR_Q
#define DISPLAY_CHAR_r           0b01010000
#define DISPLAY_CHAR_s           DISPLAY_CHAR_S
#define DISPLAY_CHAR_t           0b01111000
#define DISPLAY_CHAR_u           0b00011100
#define DISPLAY_CHAR_v           0b00011100
#define DISPLAY_CHAR_w           DISPLAY_CHAR_W
#define DISPLAY_CHAR_x           DISPLAY_CHAR_X
#define DISPLAY_CHAR_y           0b01100110
#define DISPLAY_CHAR_z           DISPLAY_CHAR_Z
#define DISPLAY_CHAR_L_ACCON     0b01111001 // 123 (ASCII)
#define DISPLAY_CHAR_BAR         0b00000110
#define DISPLAY_CHAR_R_ACCON     0b01001111
#define DISPLAY_CHAR_TILDE       0b01000000 // 126 (ASCII)

class AsciiMap {
public:
    const static uint8_t map[96];
};
// static const uint8_t asciiMap[96];
