#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
<<<<<<< HEAD
#include "stm32f10x_tim.h"
#include "led.h"
=======
#include "lcd.h"
#include "touch.h"
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775


/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);
void TIM_Configure(void);
void EXTI3_IRQHandler(void);
<<<<<<< HEAD
void EXTI9_5_IRQHandler(void);
void TIM2_IRQHandler(void);

typedef enum _LEDStatus {
    DEFAULT = 0,
    TOGGLE_POWER,
    START_ADJUST,
    CHANGE_BRIGHTNESS,
    PEND_ADJUST,
    FINISH_ADJUST
} LEDStatus;

LEDStatus currentStatus = DEFAULT;

uint16_t prescale;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
//---------------------------------------------------------------------------------------------------

void Delay(u32 delay) {
    for(u32 i=0; i < delay; i++) {}
}
=======
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

void changeBrightness(void);
void turnOnLED(void);
void turnOffLED(void);

uint16_t prescale;
uint16_t pwmPulse = 1000;
uint16_t sysMaxBrightness = 1000;
uint16_t sysMinBrightness = 0;
uint16_t ledBrightness = 500;
uint16_t minBrightness = 100;
uint16_t maxBrightness = 1000;
int8_t ledDimDir = 1;
volatile uint8_t boundaryFlag = 0;
volatile uint8_t alertFlag = 0;
int16_t color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

//---------------------------------------------------------------------------------------------------

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775

void RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // TIM2 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

<<<<<<< HEAD
=======
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/* LED pin clock enable */

>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
    /* Alternate Function IO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
<<<<<<< HEAD
    GPIO_InitTypeDef GPIO_InitStructure;
=======
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
    // Brightness touch sensor pin config
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Vibration module pin config
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

<<<<<<< HEAD
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
=======
    // LED with PWM pin config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
}

void EXTI_Configure(void) // stm32f10x_gpio.h 참고
{
    EXTI_InitTypeDef EXTI_InitStructure;

<<<<<<< HEAD
=======
	// TODO: Select the GPIO pin (Joystick, button) used as EXTI Line using function 'GPIO_EXTILineConfig'
	// TODO: Initialize the EXTI using the structure 'EXTI_InitTypeDef' and the function 'EXTI_Init'
	
    /* Joystick Down */
	// GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource1);
    // EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    // EXTI_Init(&EXTI_InitStructure);

>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
    /* Joystick Up */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

<<<<<<< HEAD
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
=======
    /* Button */
    // GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
    // EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    // EXTI_Init(&EXTI_InitStructure);
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
	
	// NOTE: do not select the UART GPIO pin used as EXTI Line here
}

void TIM_Configure(void) {
    // vibration timer
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 10000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 4000-1;         
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 2-1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
    TIM_CtrlPWMOutputs(TIM1,ENABLE);

    // Touch sensor checking timer
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 2000-1;         
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

<<<<<<< HEAD
=======
    // LED PWM timer
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 1000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2000;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    //Should disable below:
    //TIM_ARRPreloadConfig(TIM1, ENABLE);

<<<<<<< HEAD
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
=======
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ledBrightness;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
}

void NVIC_Configure(void) { // misc.h 참고
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // Brightness Touch sensor
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
<<<<<<< HEAD
    // Touch sensor checking timer
=======
    //TIM2
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

<<<<<<< HEAD
    //Sound sensor
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
=======
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
}

void TIM2_IRQHandler(void) {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
<<<<<<< HEAD
        if(currentStatus == PEND_ADJUST) {
            currentStatus = CHANGE_BRIGHTNESS;
        }
=======
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET) {
            if(!boundaryFlag) {
                changeBrightness();                
                // printf("%d\t%d\n", ledBrightness, ledDimDir);
            } else {
                if(alertFlag) {
                    printf("boundary\n");
                    TIM_Cmd(TIM1,ENABLE);
                    alertFlag = 0;
                }
            }
		}
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    }
}

<<<<<<< HEAD
// brightness touch sensor handler
void EXTI3_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET) {
            currentStatus = START_ADJUST;
		} else {
            currentStatus = FINISH_ADJUST;
=======
// Brightness touch
void EXTI3_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET) {
            boundaryFlag = 0;
            // printf("touched\n");
		} else {
            ledDimDir = -ledDimDir;
            printf("brightness: %d\%\n", (int)((float)(ledBrightness-sysMinBrightness)/(float)(sysMaxBrightness-sysMinBrightness)*100));
            // printf("released\n");
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
        }
        EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

<<<<<<< HEAD
void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) != Bit_RESET) {
            currentStatus = TOGGLE_POWER;
        }
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}

=======
void changeBrightness(void)
{
    ledBrightness = (ledBrightness+1*ledDimDir);
    boundaryFlag = !((ledBrightness - minBrightness) % (maxBrightness-minBrightness));
    alertFlag = boundaryFlag;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ledBrightness;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

void turnOnLED(void) {
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    TIM_OCInitStructure.TIM_Pulse = ledBrightness;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

void turnOffLED(void) {
    TIM_Cmd(TIM3, DISABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_0);
    printf("led: %d\n",GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0));
}


>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    EXTI_Configure();
    NVIC_Configure();
<<<<<<< HEAD
    LED_Init();

    while (1) {
        switch (currentStatus) {
        case DEFAULT:
            break;
        case TOGGLE_POWER:
            if (LED_GetPowerStatus() == 1) {
                LED_Off();
                Delay(1000000); // For debouning
            } else {
                LED_On();
                printf("brightness: %d%%\n", LED_GetBrightnessWithPercent());
                Delay(1000000); // For debouncing
            }
            currentStatus = DEFAULT;
            break;
        case START_ADJUST:
            LED_ResetBoundaryFlag();
            currentStatus = PEND_ADJUST;
            break;
        case CHANGE_BRIGHTNESS:
            if(!LED_GetBoundaryFlag()) {
                LED_ChangeBrightness();                
                printf("%d\t%d\n", LED_GetBrightness(), LED_GetDirection());
            } else {
                if(LED_GetAlertFlag()) {
                    printf("boundary\n");
                    TIM_Cmd(TIM1,ENABLE);
                    LED_ResetAlertFlag();
                }
            }
            currentStatus = PEND_ADJUST;
            break;
        case PEND_ADJUST:
            break;
        case FINISH_ADJUST:
            LED_ToggleDirection();
            printf("brightness: %d%%\n", LED_GetBrightnessWithPercent());
            currentStatus = DEFAULT;
            break;
        }
        
=======

    while (1) {
        //printf("%d\n", GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3));
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
    }
    return 0;
}
