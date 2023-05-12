/**
	*	Configure PWM output
	*
	*/

#ifndef BASIC_PWM
#define BASIC_PWM
#include "stm32f4xx_hal.h"

#define PWM_PERIOD 1680 //25kHz signal

void pwm_start(TIM_HandleTypeDef *htim,uint32_t channel);
void pwm_write(TIM_HandleTypeDef *htim,uint32_t channel, uint16_t rate);

#endif
