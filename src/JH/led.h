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
<<<<<<< HEAD
void LED_ResetAlertFlag(void);
=======
void LED_ResetAlertflag(void);
>>>>>>> 63e3f975fbc78a6d178948b1ed0a2a1736920775
u8 LED_GetAlertFlag(void);
u8 LED_GetPowerStatus(void);

#endif