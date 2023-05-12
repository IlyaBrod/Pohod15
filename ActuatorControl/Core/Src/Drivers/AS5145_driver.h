/**
	* AS5145 Magnetic angular sensor driver
	*
	*/


#ifndef AS5145_DRIVER
#define AS5145_DRIVER

#define AS5145_SENSIVITY_DEG (double)(0.0879)
#define AS5145_SENSIVITY_RAD (double)(0.00153398078)
#define AS5145_RESOLUTION (unsigned int)(4096)

#include "stm32f4xx_hal.h"
#include "SSI_control.h"


typedef struct{
	uint64_t buffer;
	
	unsigned short angle;
	unsigned short OCF;
	unsigned short COF;
	unsigned short LIN;
	unsigned short MagInc;
	unsigned short MagDec;
	unsigned short EvenPar;
	
	unsigned short CalcPar; //calculated parity bit
	unsigned short angle_valid; //angle backup
	uint8_t ready; //ready for read
	
}AS5145;


void AS5145_Init(TIM_HandleTypeDef *htim);

/**
	* Read and store SSI bus data
	*/
void AS5145_Read(void);


/**
	*	Update the measure of the sensor with error detection
	* @param passes Number of passes if there is an error of measurement
	*/
void AS5145_update(uint8_t passes);


/**
	* Return mesured value for selected ID joint
	* @param ID
	* @return Sensor anngular value
	*/
uint16_t AS5145_GetValue(uint8_t ID);


#endif
