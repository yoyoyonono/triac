#include <ch32v10x.h>
#include <debug.h>
#include <string.h>

#include "eightpindisplay.hpp"
#include "tkey.hpp"
#include "twelvepindisplay.hpp"
#include "util.hpp"

// #define LOG_INTERRUPT
// #define LOG_BUTTONVALS
#define LOG_SWITCHES
// #define LOG_ALPHA
// #define LOG_BUZZER
#define ENABLE_BUZZER
// #define TWELVE_PIN_DISPLAY
#define EIGHT_PIN_DISPLAY

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

#define KEY2_LED_PORT GPIOC
#define KEY2_LED_PIN 14
#define KEY3_LED_PORT GPIOC
#define KEY3_LED_PIN 15
#define KEY4_LED_PORT GPIOB
#define KEY4_LED_PIN 13
#define KEY5_LED_PORT GPIOB
#define KEY5_LED_PIN 12

#define TEMP_PORT GPIOC
#define TEMP_PIN 0
#define TEMP_ADC_CHANNEL ADC_Channel_10
#define VAD_PORT GPIOC
#define VAD_PIN 1
#define VAD_ADC_CHANNEL ADC_Channel_11
#define NTC_PORT GPIOC
#define NTC_PIN 2
#define NTC_ADC_CHANNEL ADC_Channel_12

#ifdef TWELVE_PIN_DISPLAY
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
#elif defined(EIGHT_PIN_DISPLAY)
#define PIN_0_PORT GPIOC
#define PIN_0_PIN 8
#define PIN_1_PORT GPIOC
#define PIN_1_PIN 7
#define PIN_2_PORT GPIOC
#define PIN_2_PIN 6
#define PIN_3_PORT GPIOB
#define PIN_3_PIN 15
#define PIN_4_PORT GPIOB
#define PIN_4_PIN 0
#define PIN_5_PORT GPIOC
#define PIN_5_PIN 5
#define PIN_6_PORT GPIOC
#define PIN_6_PIN 4
#define PIN_7_PORT GPIOA
#define PIN_7_PIN 7
#endif

#define SEG_DIG1_PORT GPIOC
#define SEG_DIG1_PIN 9
#define SEG_DIG2_PORT GPIOC
#define SEG_DIG2_PIN 6
#define SEG_DIG3_PORT GPIOB
#define SEG_DIG3_PIN 15
#define SEG_DIG4_PORT GPIOA
#define SEG_DIG4_PIN 6

#define BUZZER_PORT GPIOB
#define BUZZER_PIN 10

#define INT_PORT GPIOA
#define INT_PIN 15

#define TRIAC_PORT GPIOA
#define TRIAC_PIN 12

#define PERF_TEST_PORT GPIOB
#define PERF_TEST_PIN 8

#define FIRE_LENGTH_us 500

// peripheral usage
// ADC + TKEY: touchkeys
// ADC: NTC, VAD and temp
// RTC: coundown clock
// TIM2: buzzer PWM
// TIM3: triac delay
// TIM4: display refresh
// USART1: debug

extern "C" void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void RTC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void exti_init();
void tim2_init();
void tim3_init();
void tim4_init();
void adc_init();
void rtc_init();

uint16_t wattage_to_delay(uint16_t wattage);
int64_t get_tick();

const uint16_t wattage_delay_lookup[] = {8640, 8064, 7616, 7216, 6864, 6544, 6224, 5920, 5632, 5344, 5056, 4768, 4480, 4192, 3904, 3600, 3264, 2928, 2560, 2128, 1616, 864};

volatile uint16_t firing_delay = 10000;
uint16_t target_firing_delay = 8640;
bool first_fire = false;
uint16_t current_wattage = 200;
uint8_t read_count = 0;
uint64_t last_buzzer = 0;

static uint8_t time_us = 0;
static uint16_t time_ms = 0;

uint16_t buzzer_loop_count = 0;

bool switch_states[] = {false, false, false, false, false, false};
bool previous_switch_states[] = {false, false, false, false, false, false};
const bool all_false_switches[] = {false, false, false, false, false, false};

enum power_states { OFF,
                    ARMED,
                    ON_WATTAGE,
                    TIMER_SET,
                    TIMER_ON,
                    ERROR };

void change_power_state(power_states state);

power_states current_power_state = OFF;

int16_t adc_callibration_value = 0;

uint8_t timer_minutes = 1;
uint8_t timer_seconds = 0;

bool perf_test_state = false;

bool is_locked = false;

TouchButton touch[] = {
    TouchButton(ADC_Channel_2),
    TouchButton(ADC_Channel_1),
    TouchButton(ADC_Channel_0),
    TouchButton(ADC_Channel_5),
    TouchButton(ADC_Channel_4),
    TouchButton(ADC_Channel_3),
};

#ifdef TWELVE_PIN_DISPLAY
TwelvePinDisplay display = TwelvePinDisplay(SEG_A_PORT, SEG_A_PIN,
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
#elif defined(EIGHT_PIN_DISPLAY)
EightPinDisplay display = EightPinDisplay(PIN_0_PORT, PIN_0_PIN,
                                          PIN_1_PORT, PIN_1_PIN,
                                          PIN_2_PORT, PIN_2_PIN,
                                          PIN_3_PORT, PIN_3_PIN,
                                          PIN_4_PORT, PIN_4_PIN,
                                          PIN_5_PORT, PIN_5_PIN,
                                          PIN_6_PORT, PIN_6_PIN,
                                          PIN_7_PORT, PIN_7_PIN);
#endif

int main() {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf(__TIMESTAMP__);
    printf("\r\n");
    const volatile uint32_t* pulUID = static_cast<const volatile uint32_t*>((const void*)0x1FFFF7E8UL);
    printf("Unique ID: %08lX %08lX %08lX\r\n", pulUID[2], pulUID[1], pulUID[0]);
    printf("SystemCoreClock: %d\r\n", (int)SystemCoreClock);

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

    pinMode(KEY2_LED_PORT, KEY2_LED_PIN, OUTPUT);
    pinMode(KEY3_LED_PORT, KEY3_LED_PIN, OUTPUT);
    pinMode(KEY4_LED_PORT, KEY4_LED_PIN, OUTPUT);
    pinMode(KEY5_LED_PORT, KEY5_LED_PIN, OUTPUT);

    pinMode(TEMP_PORT, TEMP_PIN, INPUT_ANALOG);
    pinMode(VAD_PORT, VAD_PIN, INPUT_ANALOG);
    pinMode(NTC_PORT, NTC_PIN, INPUT_ANALOG);

    pinMode(PERF_TEST_PORT, PERF_TEST_PIN, OUTPUT);

    printf("Display init\r\n");

    exti_init();

    digitalWrite(TRIAC_PORT, TRIAC_PIN, HIGH);

    display.setAll();

    digitalWrite(KEY2_LED_PORT, KEY2_LED_PIN, HIGH);
    digitalWrite(KEY3_LED_PORT, KEY3_LED_PIN, HIGH);
    digitalWrite(KEY4_LED_PORT, KEY4_LED_PIN, HIGH);
    digitalWrite(KEY5_LED_PORT, KEY5_LED_PIN, HIGH);

    tim2_init();
    tim3_init();
    tim4_init();

    printf("Timers init\r\n");

    TIM2->CTLR1 |= 0;

    rtc_init();

    printf("RTC init\r\n");

    time_us = SystemCoreClock / 8000000;
    time_ms = (uint16_t)time_us * 1000;
    printf("tick init\r\n");
    printf("ms: %d\r\n", time_ms);
    printf("us: %d\r\n", time_us);

    TIM2->CTLR1 |= 1;

    SysTick->CTLR |= 1;

    delay(1000);

    printf("buzzer\r\n");

    printf("%d", get_tick());

    adc_init();

    ADC_Cmd(ADC1, ENABLE);

    printf("ADC init\r\n");

    TKEY_CR |= 0x51000000;

    delay(1000);

    for (TouchButton& button : touch) {
        printf("%d\r\n", button.callibrate());
    }

    printf("Keys init\r\n");

    change_power_state(OFF);

    while (true) {
        // toggle perf pin
        perf_test_state = !perf_test_state;
        digitalWrite(PERF_TEST_PORT, PERF_TEST_PIN, perf_test_state);
        // update firing delay for smooth change
        if (current_power_state == ON_WATTAGE) {
            if (firing_delay != target_firing_delay) {
                if (firing_delay < target_firing_delay) {
                    firing_delay += 2;
                } else {
                    firing_delay -= 2;
                }
#ifdef LOG_ALPHA
                printf("Firing delay: %d\r\n", firing_delay);
#endif
            }
        }

        // turn off buzzer if too long
        if (buzzer_loop_count == 0) {
            TIM2->ATRLR = 100;
            TIM2->CH3CVR = 80;
#ifdef LOG_BUZZER
            printf("Buzzer off %d\r\n", get_tick());
#endif
        } else {
#ifdef ENABLE_BUZZER
            TIM2->ATRLR = 2000;
            TIM2->CH3CVR = 1000;
#endif
            buzzer_loop_count--;
        }

        // read touchkey
        TKEY_CR |= 0x51000000;
        for (uint8_t i = 0; i < 6; i++) {
            switch_states[i] = touch[i].is_pressed();
#ifdef LOG_BUTTONVALS
            printf("%u:%u\t", touch[i].normal_value_average, touch[i].read());
#endif
        }
#ifdef LOG_BUTTONVALS
        printf("\r\n");
#endif
        read_count++;
        TKEY_CR &= (~0x51000000);

        // enable adc
        ADC1->CTLR2 |= 1;
        // TODO: Read ADC Here
        //        printf("TEMP: %d\r\n", analogRead(ADC1, TEMP_ADC_CHANNEL));
        //        printf("VAD: %d\r\n", analogRead(ADC1, VAD_ADC_CHANNEL));
        //        printf("NTC: %d\r\n", analogRead(ADC1, NTC_ADC_CHANNEL));

        // check if switches changed
        if (memcmp(switch_states, previous_switch_states, 6) == 0) {
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
        if (memcmp(switch_states, all_false_switches, 6) == 0) {
            continue;
        }

        if (buzzer_loop_count > 0) {
            continue;
        }

        uint8_t lowest_index = 0;
        uint32_t largest_difference = 0;
        for (uint8_t i = 0; i < 6; i++) {
            if (switch_states[i]) {
                uint32_t difference = touch[i].normal_value_average - touch[i].last_read_value;
                if (difference > largest_difference) {
                    largest_difference = difference;
                    lowest_index = i;
                }
            }
        }

        for (uint8_t i = 0; i < 6; i++) {
            if (i != lowest_index) {
                switch_states[i] = false;
            }
        }

        if (touch[lowest_index].last_read_value > touch[lowest_index].normal_value_average) {
            continue;
        }


#ifdef LOG_SWITCHES
        printf("filter\t");
        for (bool i : switch_states) {
            printf("%d", i);
        }
        printf(" %d", touch[lowest_index].last_read_value);
        printf("\r\n");
#endif

        pinMode(BUZZER_PORT, BUZZER_PIN, GPIO_Mode_AF_PP);

#ifdef LOG_BUZZER
        printf("Buzzer %d\r\n", get_tick());
#endif
        buzzer_loop_count = 30;


        if (switch_states[0]) {
            if (is_locked) {
                is_locked = false;
                change_power_state(current_power_state);
            } else {
                is_locked = true;
                display.print("LOC ");
            }
        }

        if (is_locked) {
            continue;
        }

        switch (current_power_state) {
            case OFF: {
                if (switch_states[5]) {
                    change_power_state(ARMED);
                }
                break;
            }
            case ARMED: {
                if (switch_states[5]) {
                    change_power_state(OFF);
                }
                if (switch_states[4]) {
                    change_power_state(ON_WATTAGE);
                }
                break;
            }
            case ON_WATTAGE:
                if (switch_states[2]) {
                    current_wattage -= 200;
                    if (current_wattage < 200) {
                        current_wattage = 200;
                    }
                }
                if (switch_states[3]) {
                    current_wattage += 200;
                    if (current_wattage > 2000) {
                        current_wattage = 2000;
                    }
                }

                display.printNumber(current_wattage);
                target_firing_delay = wattage_to_delay(current_wattage);

                if (switch_states[1]) {
                    change_power_state(TIMER_SET);
                }
                if (switch_states[4]) {
                    change_power_state(ARMED);
                }
                if (switch_states[5]) {
                    change_power_state(OFF);
                }
                break;
            case TIMER_SET:
                if (switch_states[2]) {
                    if (timer_seconds != 0) {
                        timer_seconds = 0;
                    } else {
                        timer_minutes--;
                        if (timer_minutes > 99) {
                            timer_minutes = 0;
                        }
                    }
                }
                if (switch_states[3]) {
                    timer_seconds = 0;
                    timer_minutes++;
                    if (timer_minutes > 99) {
                        timer_minutes = 99;
                    }
                }
                display.printTime(timer_minutes, timer_seconds);

                if (switch_states[1]) {
                    change_power_state(TIMER_ON);
                }
                if (switch_states[4]) {
                    change_power_state(ON_WATTAGE);
                }
                break;
            case TIMER_ON:
                if (switch_states[1]) {
                    change_power_state(TIMER_SET);
                }
                if (switch_states[4]) {
                    change_power_state(ON_WATTAGE);
                }
                if (switch_states[5]) {
                    change_power_state(OFF);
                }
                break;
            default:
                break;
        }
    }
    return 0;
}

void adc_init() {
    ADC_InitTypeDef ADC_InitStructure = {0};
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);

    while (ADC_GetCalibrationStatus(ADC1));
    adc_callibration_value = Get_CalibrationValue(ADC1);
}

void tim2_init() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM2->PSC = (SystemCoreClock / 1000000) - 1;
    TIM2->ATRLR = 2000;
    TIM2->CNT = 0;
    TIM2->CHCTLR2 = 0b0000000001100000;
    TIM2->CCER = 0b0000000100000000;
    TIM2->CH3CVR = 1000;
}

void tim3_init() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM3->PSC = (SystemCoreClock / 500000) - 1;
    TIM3->ATRLR = 1000;
    TIM3->CNT = 0;
    TIM3->CHCTLR2 = 0b0000000001100000;
    TIM3->CCER = 0b0000000100000000;
    TIM3->CH3CVR = 500;
    TIM3->DMAINTENR |= 1;

    NVIC_EnableIRQ(TIM3_IRQn);
}

void tim4_init() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM4->PSC = (SystemCoreClock / 1000000) - 1;
    TIM4->ATRLR = 1000;
    TIM4->CNT = 0;
    TIM4->CHCTLR2 = 0b0000000001100000;
    TIM4->CCER = 0b0000000100000000;
    TIM4->CH3CVR = 500;
    TIM4->DMAINTENR |= 1;
    TIM4->CTLR1 |= 1;

    NVIC_EnableIRQ(TIM4_IRQn);
}

void exti_init() {
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    printf("EXTI Init\r\n");

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void rtc_init() {
    // configure rtc and enable interrupt for every second
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RCC_LSICmd(ENABLE);

    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_WaitForLastTask();
    RTC_SetPrescaler(40000);
    RTC_WaitForLastTask();

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

extern "C" void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
        // turn off all digits
        EXTI_ClearITPendingBit(EXTI_Line15); /* Clear Flag */
#ifdef LOG_INTERRUPT
        printf("EXTI15_10_IRQHandler\r\n");
#endif
        if (current_power_state == ON_WATTAGE || current_power_state == TIMER_ON) {
            display.allOff();
            TIM3->ATRLR = firing_delay;
            TIM3->CNT = 0;
            TIM3->CTLR1 |= 1;
        }
    }
}

extern "C" void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
#ifdef LOG_INTERRUPT
        printf("TIM3_IRQHandler\r\n");
#endif
        if (first_fire) {
            first_fire = false;
            digitalWrite(TRIAC_PORT, TRIAC_PIN, LOW);
            TIM3->ATRLR = FIRE_LENGTH_us;
        } else {
            first_fire = true;
            digitalWrite(TRIAC_PORT, TRIAC_PIN, HIGH);
            TIM3->CTLR1 &= (~1);
        }
    }
}

extern "C" void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
#ifdef LOG_INTERRUPT
        printf("TIM4_IRQHandler\r\n");
#endif
        display.refresh();
    }
}

extern "C" void RTC_IRQHandler(void) {
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
        RTC_ClearITPendingBit(RTC_IT_SEC);
#ifdef LOG_INTERRUPT
        printf("RTC_IRQHandler\r\n");
#endif
        if (current_power_state == TIMER_ON) {
            if (timer_seconds == 0) {
                if (timer_minutes == 0) {
                    change_power_state(OFF);
                    buzzer_loop_count = 200;
                    return;
                }
                timer_minutes--;
                timer_seconds = 59;
            } else {
                timer_seconds--;
            }
            display.printTime(timer_minutes, timer_seconds);
        }
    }
}

uint16_t wattage_to_delay(uint16_t wattage) {
    return wattage_delay_lookup[wattage / 100 - 1];
}

int64_t get_tick() {
    return (static_cast<uint64_t>(SysTick->CNTL0)) +
           (static_cast<uint64_t>(SysTick->CNTL1) << 8) +
           (static_cast<uint64_t>(SysTick->CNTL2) << 16) +
           (static_cast<uint64_t>(SysTick->CNTL3) << 24) +
           (static_cast<uint64_t>(SysTick->CNTH0) << 32) +
           (static_cast<uint64_t>(SysTick->CNTH1) << 40) +
           (static_cast<uint64_t>(SysTick->CNTH2) << 48) +
           (static_cast<uint64_t>(SysTick->CNTH3) << 56);
}

uint16_t convert_adc_3V3(int16_t val) {
    int32_t y;
    y = 6 * (val + adc_callibration_value) / 1000 - 12;
    if (val == 0 || val == 4095)
        return val;
    else {
        if ((val + adc_callibration_value - y) < 0)
            return 0;
        if ((adc_callibration_value + val - y) > 4095 || val == 4095)
            return 4095;
        return (val + adc_callibration_value);
    }
}

uint16_t convert_adc(int16_t val) {
    if ((val + adc_callibration_value) < 0)
        return 0;
    if ((adc_callibration_value + val) > 4095 || val == 4095)
        return 4095;
    return (val + adc_callibration_value);
}

void change_power_state(power_states state) {
    current_power_state = state;
    switch (state) {
        case OFF:
            display.print("----");
            digitalWrite(KEY2_LED_PORT, KEY2_LED_PIN, LOW);
            digitalWrite(KEY3_LED_PORT, KEY3_LED_PIN, LOW);
            digitalWrite(KEY4_LED_PORT, KEY4_LED_PIN, LOW);
            digitalWrite(KEY5_LED_PORT, KEY5_LED_PIN, LOW);
            break;
        case ARMED:
            display.print("FUNC");
            digitalWrite(KEY2_LED_PORT, KEY2_LED_PIN, LOW);
            digitalWrite(KEY5_LED_PORT, KEY5_LED_PIN, LOW);
            break;
        case ON_WATTAGE:
            display.printNumber(current_wattage);
            digitalWrite(KEY2_LED_PORT, KEY2_LED_PIN, LOW);
            digitalWrite(KEY5_LED_PORT, KEY5_LED_PIN, HIGH);
            break;
        case TIMER_SET:
            display.printTime(timer_minutes, timer_seconds);
            digitalWrite(KEY2_LED_PORT, KEY2_LED_PIN, LOW);
            digitalWrite(KEY5_LED_PORT, KEY5_LED_PIN, LOW);
            break;
        case TIMER_ON:
            display.printTime(timer_minutes, timer_seconds);
            digitalWrite(KEY2_LED_PORT, KEY2_LED_PIN, HIGH);
            digitalWrite(KEY5_LED_PORT, KEY5_LED_PIN, HIGH);
            break;
        case ERROR:
            display.print("ERR ");
            break;
        default:
            break;
    }
}
