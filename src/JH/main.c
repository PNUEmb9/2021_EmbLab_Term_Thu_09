#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "led.h"


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

=======
void TIM2_IRQHandler(void);

>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
uint16_t prescale;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
//---------------------------------------------------------------------------------------------------

<<<<<<< HEAD
void Delay(u32 delay) {
    for(u32 i=0; i < delay; i++) {}
}

=======
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
void RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // TIM2 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    /* Alternate Function IO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;
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
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
}

void EXTI_Configure(void) // stm32f10x_gpio.h 참고
{
    EXTI_InitTypeDef EXTI_InitStructure;

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

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2000;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    //Should disable below:
    //TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
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
    
    //TIM2
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

<<<<<<< HEAD
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
		}
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    }
}

// Brightness touch
void EXTI3_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET) {
<<<<<<< HEAD
            currentStatus = START_ADJUST;
		} else {
            currentStatus = FINISH_ADJUST;
=======
            LED_ResetBoundaryFlag();
            // printf("touched\n");
		} else {
            LED_ToggleDirection();
            printf("brightness: %d\%\n", LED_GetBrightnessWithPercent());
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
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775

int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    EXTI_Configure();
    NVIC_Configure();
    LED_Init();

<<<<<<< HEAD
    while (1) {
        switch (currentStatus) {
        case DEFAULT:
            break;
        case TOGGLE_POWER:
            if (LED_GetPowerStatus() == 1) {
                LED_Off();
                Delay(1000000);
            } else {
                LED_On();
                printf("brightness: %d%%\n", LED_GetBrightnessWithPercent());
                Delay(1000000);
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
