/**
	*	Synchronous Serial Interface Control
	*
	*/

#ifndef SSI_CONTROL
#define SSI_CONTROL

#include "stm32f4xx_hal.h"

#define GPIO_CLK		GPIOA
#define GPIO_CS			GPIOA
#define GPIO_DATA 	GPIOB
#define PIN_CLK			GPIO_PIN_6
#define PIN_CS			GPIO_PIN_7
#define PIN_DATA		GPIO_PIN_0


#define PARITY_EVEN 0
#define PARITY_ODD 1

void SSI_Init(TIM_HandleTypeDef *htim);

/**
	*	Read SSi Data MSB first
	* @param buffer Pointer to the output variable
	* @param bits_counts Number of bits to read
	* @param timeout Maximum tranfer time in ms
	*/
void SSI_Read(uint64_t *buffer, uint32_t bits_count);

/**
	*  Interrupt function on every clock ticks
	*/
void SSI_update(void);

/**
	* Approxiamte delay in nanoseconds based on instructions executions speed
	* 5 instructions + access + back => 7 instructions minimum
	* @param delay Time to wait in ns
	*/
void Delay_ns(unsigned int delay);

/**
	* Check the partity of a given word, to avoid errors [64bits word]
	* @param word Data to check
	* @param parityType Type of parity (PARITY_EVEN or PARITY_ODD)
	* @return Bit of parity
	*/
uint8_t get_Parity(unsigned int word, uint8_t parityType);

#endif
