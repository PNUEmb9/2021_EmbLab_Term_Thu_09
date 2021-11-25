#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"


/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);
void TIM_Configure(void);
void change(uint16_t pulse);

void TIM2_IRQHandler(void);

uint16_t prescale;
uint16_t motorFlag = 0;
uint16_t ledPowerFlag = 0;
uint16_t led1ToggleFlag = 0;
uint16_t led2ToggleFlag = 0;
uint16_t led1Counter = 0;
uint16_t led2Counter = 0;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};
char* ledStatus[2] = {"OFF", "ON"};
int motorIdx[3] = {700, 1500, 2600};

//---------------------------------------------------------------------------------------------------

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

void RCC_Configure(void) // stm32f10x_rcc.h 참고
{
    // TIM2 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/* LED pin clock enable */
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void TIM_Configure(void) {
    TIM_TimeBaseStructure.TIM_Period = 10000;         
    TIM_TimeBaseStructure.TIM_Prescaler = 7200;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 20000-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; // us
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void NVIC_Configure(void) { // misc.h 참고
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    //TIM2
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void) {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        led2Counter = (led2Counter+1) % 5; 
        led1ToggleFlag = !led1ToggleFlag;
        led2ToggleFlag = led2ToggleFlag ^(!led2Counter);
        change(motorIdx[motorFlag]);
        motorFlag = (motorFlag +1) % 3;

        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    }
}

void change(uint16_t per)
{
    uint16_t pwm_pulse;
    pwm_pulse = per;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pwm_pulse;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    NVIC_Configure();

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    uint16_t rawTouchX = 0;
    uint16_t rawTouchY = 0;
    uint16_t touchX = 0;
    uint16_t touchY = 0;

    while (1) {
        LCD_ShowString(40, 40, "THU_TEAM09", BLACK, WHITE);
        LCD_Fill(40,60,90,100, WHITE);
        LCD_ShowString(40, 60, ledStatus[led1ToggleFlag & ledPowerFlag], BLACK, WHITE);
        LCD_ShowString(40, 80, ledStatus[led2ToggleFlag & ledPowerFlag], BLACK, WHITE);
        LCD_DrawRectangle(40, 100, 80, 140);
        LCD_ShowString(50, 110, "BTN", BLACK, WHITE);
        GPIO_Write(GPIOD, ((GPIO_Pin_2 * led1ToggleFlag) | (GPIO_Pin_3 * led2ToggleFlag))*ledPowerFlag);

        Touch_GetXY(&rawTouchX, &rawTouchY, 0); //Wait until Touched
        Convert_Pos(rawTouchX, rawTouchY, &touchX, &touchY);
        if(touchX >= 40 && touchX <= 100 && touchY >= 80 && touchY <= 140) {
            ledPowerFlag = !ledPowerFlag;
            touchX = 0;
            touchY = 0;
        }
    }
    return 0;
}
