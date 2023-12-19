#include <ch32v10x.h>
#include "util.hpp"

void pinMode(GPIO_TypeDef *GPIOx, uint8_t PIN, GPIOMode_TypeDef MODE, GPIOSpeed_TypeDef SPEED) {
    if (GPIOx == GPIOA) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    }

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = 1 << PIN;
    GPIO_InitStructure.GPIO_Mode = MODE;
    if (MODE == INPUT_ANALOG) {

    }
    else {
        GPIO_InitStructure.GPIO_Speed = SPEED;
    }
    GPIO_Init(GPIOx, &GPIO_InitStructure);        
    
}

void pinMode(GPIO_TypeDef *GPIOx, uint8_t PIN, GPIOMode_TypeDef MODE) {
    pinMode(GPIOx, PIN, MODE, GPIO_Speed_50MHz);
}

void digitalWrite(GPIO_TypeDef *GPIOx, uint8_t PIN, BitAction VAL) {
    GPIO_WriteBit(GPIOx, 1 << PIN, VAL);
}

void digitalWrite(GPIO_TypeDef *GPIOx, uint8_t PIN, bool VAL) {
    digitalWrite(GPIOx, PIN, VAL ? HIGH : LOW);
}

uint8_t digitalRead(GPIO_TypeDef *GPIOx, uint8_t PIN) {
    return GPIO_ReadInputDataBit(GPIOx, 1 << PIN);
}

uint16_t analogRead(ADC_TypeDef *ADCx, uint8_t CHANNEL) {
    ADC_RegularChannelConfig(ADCx, CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);

    while (!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC));

    return ADC_GetConversionValue(ADCx);
}