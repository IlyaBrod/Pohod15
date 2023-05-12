/**
	*	Configure Input Capture entree
	*
	*/

#ifndef BASIC_IC
#define BASIC_IC
#include "stm32f4xx_hal.h"

//USER FUNCTIONS
void ic_start(TIM_HandleTypeDef *htim,uint32_t channel,uint8_t ID);
uint32_t ic_read(uint8_t ID);

//PRIVATE FUCTIONS
void HAL_TIM_IC_CALL(TIM_HandleTypeDef *htim);
void DMA_IC_to_PWM(TIM_HandleTypeDef *htim,uint8_t ID);
#endif
