#include <stdio.h>
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
void EXTI1_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM2_IRQHandler(void);
void USARTS_Init(void);
void sendBoundaryMessage(int8_t);
void sendPowerStatusMessage(char);
void sendBrightnessMessage(void);

typedef enum _LEDStatus {
    DEFAULT = 0,
    SEND_MSG,
    TOGGLE_POWER,
    TURN_ON_POWER,
    TURN_OFF_POWER,
    START_ADJUST,
    CHANGE_BRIGHTNESS,
    ALERT_BOUNDARY,
    PEND_ADJUST,
    FINISH_ADJUST
} LEDStatus;

char onFlag = 0;
char offFlag = 0;
char oFlag = 0;
char fFlag = 0;

LEDStatus currentStatus = DEFAULT;

uint16_t prescale;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
//---------------------------------------------------------------------------------------------------

void Delay(u32 delay) {
    for(u32 i=0; i < delay; i++) {}
}

void RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // TIM2 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Bluetooth via usart clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);

    /* Alternate Function IO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // Brightness touch sensor pin config / PD3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Brightness touch sensor pin config / PD3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Vibration module pin config / PA8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Sound sensor pin config / PD7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* UART between bluetooth pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  	//RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void USARTS_Init(void)
{
    USART_InitTypeDef USART_InitStructure;

	// USART2 peripheral
    USART_Cmd(USART2, ENABLE);
	
	/* 
       BaudRate: 9600
       WordLength: 8bits
       Parity: None
       StopBits: 1bit
       Hardware Flow Control: None
     */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = (uint16_t) USART_WordLength_8b;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = (uint16_t) USART_Parity_No;
    USART_InitStructure.USART_StopBits = (uint16_t) USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = (uint16_t) USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void EXTI_Configure(void) // stm32f10x_gpio.h 참고
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Power Touch sensor interrupt */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource1);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /*  */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	
	// NOTE: do not select the UART GPIO pin used as EXTI Line here
}

void TIM_Configure(void) {
    // vibration timer
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 10000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 4000-1;         
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
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
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2000;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    //Should disable below:
    //TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void NVIC_Configure(void) { // misc.h 참고
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    
    // Brightness Touch sensor
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // Power Touch sensor
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //TIM2
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM1_UP_IRQHandler(void) {
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        Delay(500000);
        LED_ResetAlertFlag();
        TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
    }
}

void TIM2_IRQHandler(void) {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        if(currentStatus == PEND_ADJUST) {
            currentStatus = CHANGE_BRIGHTNESS;
        }
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    }
}

// Power touch
void EXTI1_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) == Bit_RESET) {
            currentStatus = TOGGLE_POWER;
		}
        EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

// Brightness touch
void EXTI3_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        if(LED_GetPowerStatus() && !LED_GetAlertFlag()) {
            if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_RESET ) {
                    currentStatus = START_ADJUST;
            } else {
                if(!LED_GetBoundaryStatus()) {
                    currentStatus = FINISH_ADJUST;
                }
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

// Sound Sensor
void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) != Bit_RESET) {
            if(!LED_GetAlertFlag()) {
                currentStatus = TOGGLE_POWER;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}

// Bluetooth
void USART2_IRQHandler() {
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){     
        char temp = USART_ReceiveData(USART2);
        if(temp == '0') {
            currentStatus = TURN_OFF_POWER;
        } else if(temp == '1') {
            currentStatus = TURN_ON_POWER;
        }
    }
}

void sendDataUART2(uint16_t data) {
	USART_SendData(USART2, data);
    while ((USART2->SR & USART_SR_TC) == 0);
}

// LED 인터럽트에 넣기 
void sendPowerStatusMessage(char powerStatus){
    char msg[100];
    char * powerMsg = (powerStatus == 1 ? "On" : "Off");
    snprintf(msg,100,"LED turned %s\r\n", powerMsg);
    char* pmsg = msg;
    while(*pmsg!=0){
        sendDataUART2(*pmsg);
        pmsg++;
    } 
}
void sendBoundaryMessage(int8_t boundaryStatus){
    char msg[50];
    char * boundaryMsg = (boundaryStatus == -1 ? "min" : "max");
    snprintf(msg,50,"LED reaches at %s brightness\r\n", boundaryMsg);
    char* pmsg = msg;
    while(*pmsg!=0){
        sendDataUART2(*pmsg);
        pmsg++;
    } 
}
void sendBrightnessMessage(){
    //getLEDstatus;
    char msg[50];
    uint16_t brightness = LED_GetBrightnessWithPercent();
    snprintf(msg,50,"LED brightness reaches %d%%\r\n", brightness);
    char* pmsg = msg;
    while(*pmsg!=0){
        sendDataUART2(*pmsg);
        pmsg++;
    }
}

int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USARTS_Init();
    TIM_Configure();
    EXTI_Configure();
    NVIC_Configure();
    LED_Init();

    while (1) {
        switch (currentStatus) {
            case DEFAULT: {
                break;
            } case TOGGLE_POWER: {
                if (LED_GetPowerStatus() == 1) {
                    LED_Off();
                    sendPowerStatusMessage(0);
                    Delay(100000);
                } else {
                    LED_On();
                    sendPowerStatusMessage(1);
                    sendBrightnessMessage();
                    Delay(100000);
                }
                currentStatus = DEFAULT;
                break;
            } case TURN_ON_POWER: {
                if (LED_GetPowerStatus() == 0) {
                    LED_On();
                    sendPowerStatusMessage(1);
                    sendBrightnessMessage();
                }
                currentStatus = DEFAULT;
                break;
            } case TURN_OFF_POWER: {
                if (LED_GetPowerStatus() == 1) {
                    LED_Off();
                    sendPowerStatusMessage(0);
                }
                currentStatus = DEFAULT;
                break;
            } case START_ADJUST: {
                LED_ResetBoundaryStatus();
                currentStatus = PEND_ADJUST;
                break;
            } case CHANGE_BRIGHTNESS: {
                if(!LED_GetBoundaryStatus()) {
                    LED_ChangeBrightness();                
                    currentStatus = PEND_ADJUST;
                } else {
                    currentStatus = ALERT_BOUNDARY;
                }
                break;
            } case PEND_ADJUST: {
                break;
            } case ALERT_BOUNDARY: {
                sendBoundaryMessage(LED_GetBoundaryStatus());
                TIM_Cmd(TIM1,ENABLE);
                currentStatus = FINISH_ADJUST;
                break;
            } case FINISH_ADJUST: {
                LED_ToggleDirection();
                sendBrightnessMessage();
                currentStatus = DEFAULT;
                break;
            }
        }
    }
    return 0;
}
