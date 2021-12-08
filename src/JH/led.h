#ifndef __LED_H_
#define __LED_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_ChangeBrightness(void);
u16 LED_GetBrightness(void);
u16 LED_GetBrightnessWithPercent(void);
void LED_ToggleDirection(void);
int8_t LED_GetDirection(void);
void LED_ResetBoundaryFlag(void);
u8 LED_GetBoundaryFlag(void);
void LED_ResetAlertFlag(void);
u8 LED_GetAlertFlag(void);
u8 LED_GetPowerStatus(void);

#endif