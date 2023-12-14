#include <debug.h>
#include <ch32v10x.h>
#include <string.h>
#include "util.hpp"
#include "ch32v10x_it.h"
#include "tkey.hpp"
#include "display.hpp"

//#define LOG_INTERRUPT
//#define LOG_BUTTONVALS
#define LOG_SWITCHES
#define LOG_ALPHA

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
#define INT_PIN 9

#define TRIAC_PORT GPIOA
#define TRIAC_PIN 8

#define FIRE_LENGTH_us 600

const uint16_t wattage_delay_lookup[] = {8645, 8069, 7615, 7223, 6868, 6538, 6225, 5924, 5631, 5343, 5059, 4774, 4488, 4198, 3901, 3593, 3270, 2927, 2552, 2128, 1615, 866};

uint16_t firing_delay = 8700;
uint16_t target_firing_delay = 8645;
uint16_t current_wattage = 200;

bool switch_states[] = {false, false, false, false, false, false};
bool previous_switch_states[] = {false, false, false, false, false, false};

TouchButton touch[] = {
    TouchButton(ADC_Channel_0, 2200),
    TouchButton(ADC_Channel_1, 1500),
    TouchButton(ADC_Channel_2, 1500),
    TouchButton(ADC_Channel_3, 1500),
    TouchButton(ADC_Channel_4, 1500),
    TouchButton(ADC_Channel_5, 2200)
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

void timer_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_ICInitTypeDef TIM_ICInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = firing_delay + FIRE_LENGTH_us;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = firing_delay;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Trigger);
}


void set_alpha(uint16_t alpha) {
    TIM2->CH3CVR = alpha;
    TIM2->ATRLR = alpha + FIRE_LENGTH_us;
#ifdef LOG_ALPHA
    printf("Alpha: %d\r\n", alpha);
#endif
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

    timer_init();

    printf("Timer init \r\n");

    display.clear();

    display.printNumber(current_wattage);

    printf("Display init \r\n");

    set_alpha(firing_delay);

    while (true) {
        if (firing_delay != target_firing_delay) {
            if (firing_delay < target_firing_delay) {
                firing_delay++;
            }
            else {
                firing_delay--;
            }
            set_alpha(firing_delay);
        }
        display.refresh();

        for (uint8_t i = 0; i < 6; i++) {
            switch_states[i] = touch[i].is_pressed();
#ifdef LOG_BUTTONVALS
            printf("%d ", touch[i].read());
#endif
        }
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
        printf("Wattage: %d\tDelay: %d\r\n", current_wattage, target_firing_delay);
    }
    return 0;
}
