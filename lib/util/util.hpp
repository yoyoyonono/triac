#pragma once

#include <ch32v10x.h>

#define OUTPUT GPIO_Mode_Out_PP
#define OUTPUT_OPEN_DRAIN GPIO_Mode_Out_OD
#define INPUT GPIO_Mode_IN_FLOATING
#define INPUT_PULLUP GPIO_Mode_IPU
#define INPUT_PULLDOWN GPIO_Mode_IPD
#define INPUT_ANALOG GPIO_Mode_AIN

#define HIGH Bit_SET
#define LOW Bit_RESET

#define delay(x) Delay_Ms(x)
#define delayMicroseconds(x) Delay_Us(x)

void pinMode(GPIO_TypeDef *GPIOx, uint8_t PIN, GPIOMode_TypeDef MODE, GPIOSpeed_TypeDef SPEED);
void pinMode(GPIO_TypeDef *GPIOx, uint8_t PIN, GPIOMode_TypeDef MODE);

void digitalWrite(GPIO_TypeDef *GPIOx, uint8_t PIN, BitAction VAL);
void digitalWrite(GPIO_TypeDef *GPIOx, uint8_t PIN, bool VAL);