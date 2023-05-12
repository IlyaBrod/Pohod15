#include "PWM_control.h"

void pwm_start(TIM_HandleTypeDef *htim,uint32_t channel)
{
	HAL_TIM_Base_Start(htim); 
	HAL_TIM_PWM_Start(htim,channel);
}

void pwm_write(TIM_HandleTypeDef *htim,uint32_t channel, uint16_t rate)
{
	__HAL_TIM_GET_AUTORELOAD(htim);
	__HAL_TIM_SET_COMPARE(htim, channel, rate);
}

