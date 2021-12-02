#include "led.h"
#include "stdio.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

static u16 ledBrightness = 500;
static u16 sysMaxBrightness = 1000;
static u16 sysMinBrightness = 0;
static u16 maxBrightness = 1000;
static u16 minBrightness = 100;
static int8_t ledDirection = 1;

static volatile u8 powerFlag = 0;
static volatile u8 boundaryFlag = 0;
static volatile u8 alertFlag = 0;
static GPIO_InitTypeDef GPIO_InitStructure;
static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
static TIM_OCInitTypeDef TIM_OCInitStructure;

static u16 myClamp(u16 target, u16 min, u16 max) {
    return (target < min) ? min : ((target > max ? max : target));
}

static void LED_RCC_Configure(void) {
    // LED PWM timer clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // LED Pin clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // Alternate Function IO clock enable for PWM on TIM3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

static void LED_GPIO_AF_Configure(void) {
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void LED_GPIO_Normal_Configure(void) {
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void LED_PWM_Apply(u16 pulse) {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

static void LED_TIM_Configure(void) {
    // LED PWM timer
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 1000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    LED_PWM_Apply(ledBrightness);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}

void LED_Init(void) {
    LED_RCC_Configure();
    LED_GPIO_AF_Configure();
    LED_TIM_Configure();
    LED_Off();
}

void LED_On(void) {
    LED_GPIO_AF_Configure();
    LED_PWM_Apply(ledBrightness);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    powerFlag = 1;
    printf("ON\n");
}

void LED_Off(void) {
    TIM_Cmd(TIM3, DISABLE);
    LED_GPIO_Normal_Configure();
    GPIO_ResetBits(GPIOB,GPIO_Pin_0);
    powerFlag = 0;
    printf("OFF\n");
}

void LED_ChangeBrightness(void) {
    ledBrightness = (ledBrightness+1*ledDirection);
    ledBrightness = myClamp(ledBrightness, minBrightness, maxBrightness);
    boundaryFlag = !((ledBrightness - minBrightness) % (maxBrightness-minBrightness));
    alertFlag = boundaryFlag;

    LED_PWM_Apply(ledBrightness);
}

u16 LED_GetBrightness(void) {
    return ledBrightness;
}

u16 LED_GetBrightnessWithPercent(void) {
    return (u16)((float)(ledBrightness-sysMinBrightness)/(float)(sysMaxBrightness-sysMinBrightness)*100);
}

void LED_ToggleDirection(void) {
    if(ledBrightness >= maxBrightness) {
        ledDirection = -1;
    } else if(ledBrightness <=minBrightness) {
        ledDirection = 1;
    } else {
        ledDirection = -ledDirection;
    }
}

int8_t LED_GetDirection(void) {
    return ledDirection;
}

void LED_ResetBoundaryFlag(void) {
    boundaryFlag = 0;
}

u8 LED_GetBoundaryFlag(void) {
    return boundaryFlag;
}

void LED_ResetAlertFlag(void) {
    alertFlag = 0;
}

u8 LED_GetAlertFlag(void) {
    return alertFlag; 
}

u8 LED_GetPowerStatus(void) {
    return powerFlag;
}