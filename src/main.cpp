#include <debug.h>
#include <ch32v10x.h>
#include <string.h>
#include "util.hpp"
#include "ch32v10x_it.h"
#include "tkey.hpp"
#include "display.hpp"

//#define LOG_INTERRUPT
//#define LOG_BUTTONVALS
//#define LOG_SWITCHES
//#define LOG_ALPHA

#define KEY1_PORT GPIOA
#define KEY1_PIN 0
#define KEY2_PORT GPIOA
#define KEY2_PIN 1
#define KEY3_PORT GPIOA
#define KEY3_PIN 2
#define KEY4_PORT GPIOA
#define KEY4_PIN 3
#define KEY5_PORT GPIOA
#define KEY5_PIN 4
#define KEY6_PORT GPIOA
#define KEY6_PIN 5

#define SEG_A_PORT GPIOC
#define SEG_A_PIN 8
#define SEG_B_PORT GPIOB
#define SEG_B_PIN 14
#define SEG_C_PORT GPIOC
#define SEG_C_PIN 4
#define SEG_D_PORT GPIOB
#define SEG_D_PIN 0
#define SEG_E_PORT GPIOB
#define SEG_E_PIN 1
#define SEG_F_PORT GPIOC
#define SEG_F_PIN 7
#define SEG_G_PORT GPIOA
#define SEG_G_PIN 7
#define SEG_DP_PORT GPIOC
#define SEG_DP_PIN 5

#define SEG_DIG1_PORT GPIOC
#define SEG_DIG1_PIN 9
#define SEG_DIG2_PORT GPIOC
#define SEG_DIG2_PIN 6
#define SEG_DIG3_PORT GPIOB
#define SEG_DIG3_PIN 15
#define SEG_DIG4_PORT GPIOA
#define SEG_DIG4_PIN 6

#define INT_PORT GPIOA
#define INT_PIN 15

#define TRIAC_PORT GPIOA
#define TRIAC_PIN 12

#define FIRE_LENGTH_us 600

extern "C" void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

const uint16_t wattage_delay_lookup[] = {8640, 8064, 7616, 7216, 6864, 6544, 6224, 5920, 5632, 5344, 5056, 4768, 4480, 4192, 3904, 3600, 3264, 2928, 2560, 2128, 1616, 864};

volatile uint16_t firing_delay = 8704;
uint16_t target_firing_delay = 8640;
uint16_t current_wattage = 200;

bool switch_states[] = {false, false, false, false, false, false};
bool previous_switch_states[] = {false, false, false, false, false, false};

TouchButton touch[] = {
    TouchButton(ADC_Channel_0, 1500),
    TouchButton(ADC_Channel_1, 1500),
    TouchButton(ADC_Channel_2, 1500),
    TouchButton(ADC_Channel_3, 1500),
    TouchButton(ADC_Channel_4, 1500),
    TouchButton(ADC_Channel_5, 1500)
};

Display display = Display(SEG_A_PORT, SEG_A_PIN,
                          SEG_B_PORT, SEG_B_PIN,
                          SEG_C_PORT, SEG_C_PIN,
                          SEG_D_PORT, SEG_D_PIN,
                          SEG_E_PORT, SEG_E_PIN,
                          SEG_F_PORT, SEG_F_PIN,
                          SEG_G_PORT, SEG_G_PIN,
                          SEG_DP_PORT, SEG_DP_PIN,
                          SEG_DIG1_PORT, SEG_DIG1_PIN,
                          SEG_DIG2_PORT, SEG_DIG2_PIN,
                          SEG_DIG3_PORT, SEG_DIG3_PIN,
                          SEG_DIG4_PORT, SEG_DIG4_PIN,
                          COMMON_CATHODE);

uint8_t read_count = 0;

uint16_t wattage_to_delay(uint16_t wattage) {
    return wattage_delay_lookup[wattage / 100 - 1];
}

int main() {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf(__TIMESTAMP__);
    printf("\r\n");
    printf("SystemCoreClock: %d\r\n", SystemCoreClock);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    pinMode(INT_PORT, INT_PIN, INPUT_PULLDOWN);
    pinMode(TRIAC_PORT, TRIAC_PIN, OUTPUT);

    pinMode(KEY1_PORT, KEY1_PIN, INPUT_ANALOG);
    pinMode(KEY2_PORT, KEY2_PIN, INPUT_ANALOG);
    pinMode(KEY3_PORT, KEY3_PIN, INPUT_ANALOG);
    pinMode(KEY4_PORT, KEY4_PIN, INPUT_ANALOG);
    pinMode(KEY5_PORT, KEY5_PIN, INPUT_ANALOG);
    pinMode(KEY6_PORT, KEY6_PIN, INPUT_ANALOG);

    pinMode(SEG_DIG1_PORT, SEG_DIG1_PIN, OUTPUT);
    pinMode(SEG_DIG2_PORT, SEG_DIG2_PIN, OUTPUT);
    pinMode(SEG_DIG3_PORT, SEG_DIG3_PIN, OUTPUT);
    pinMode(SEG_DIG4_PORT, SEG_DIG4_PIN, OUTPUT);
    pinMode(SEG_A_PORT, SEG_A_PIN, OUTPUT);

    digitalWrite(SEG_A_PORT, SEG_A_PIN, HIGH);
    digitalWrite(SEG_DIG1_PORT, SEG_DIG1_PIN, LOW);
    digitalWrite(SEG_DIG2_PORT, SEG_DIG2_PIN, LOW);
    digitalWrite(SEG_DIG3_PORT, SEG_DIG3_PIN, HIGH);
    digitalWrite(SEG_DIG4_PORT, SEG_DIG4_PIN, HIGH);

    printf("Keys init\r\n");

    TKEY_CR |= 0x51000000;
    
    ADC_Cmd(ADC1, ENABLE);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    printf("EXTI Init ");

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    digitalWrite(TRIAC_PORT, TRIAC_PIN, LOW);

//    timer_init();

    printf("Timer init \r\n");

    display.clear();

    display.printNumber(current_wattage);

    printf("Display init \r\n");

    while (true) {
        if (firing_delay != target_firing_delay) {
            if (firing_delay < target_firing_delay) {
                firing_delay += 8;
            }
            else {
                firing_delay -= 8;
            }
        }
        for (int i = 0; i < 16; i++) {
            display.refresh();            
        }
        display.allOff();
        for (uint8_t i = 0; i < 6; i++) {
            switch_states[i] = touch[i].is_pressed();
#ifdef LOG_BUTTONVALS
            printf("%d ", touch[i].read());
#endif
        }
#ifdef LOG_BUTTONVALS
        printf("\r\n");
#endif
        read_count++;
        
        
        if (!memcmp(switch_states, previous_switch_states, 6)) {
            continue;
        }
        memcpy(previous_switch_states, switch_states, 6);
#ifdef LOG_SWITCHES
        printf("change\t");
        for (bool i : switch_states) {
            printf("%d", i);
        }
        printf("\r\n");
#endif  
        if (switch_states[0]) {
            current_wattage -= 200;
            if (current_wattage < 200) {
                current_wattage = 200;
            }
        }
        if (switch_states [5]) {
            current_wattage += 200; 
            if (current_wattage > 2000) {
                current_wattage = 2000;
            }
        }
        display.printNumber(current_wattage);
        target_firing_delay = wattage_to_delay(current_wattage);
#ifdef LOG_SWITCHES
        printf("Wattage: %d\tDelay: %d\r\n", current_wattage, target_firing_delay);
#endif
    }
    return 0;
}

extern "C" void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
        // turn off all digits
        display.allOff();
#ifdef LOG_INTERRUPT
        printf("EXTI15_10_IRQHandler\r\n");
#endif
        delayMicroseconds(firing_delay);
        digitalWrite(TRIAC_PORT, TRIAC_PIN, HIGH);
        delayMicroseconds(FIRE_LENGTH_us);  
        digitalWrite(TRIAC_PORT, TRIAC_PIN, LOW);
        EXTI_ClearITPendingBit(EXTI_Line15); /* Clear Flag */
    }
}