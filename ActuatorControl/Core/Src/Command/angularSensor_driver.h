

#ifndef ANGULAR_SENSOR
#define ANGULAR_SENSOR

#include "stm32f4xx_hal.h"
#include "../Drivers/I2C_control.h"
#include "../Drivers/TCA_control.h"
#include "LEG_CONFIG.h"

//AS5600 angular sensor
#define ANGLE_RESOLUTION (unsigned int)(4096) //[ticks]
#define ANGLE_SENSIVITY_DEG (double)(0.0879) //[deg/ticks]
#define ANGLE_SENSIVITY_RAD (double)(0.00153398078) //[rad/ticks]

#define ANGLE_I2C_ADDR_0 0x36
#define ANGLE_I2C_REGISTER_HI 0x0C
#define ANGLE_I2C_REGISTER_LO 0x0D



/**
	* Initialization procedure for the sensor
	*/
void angularSensor_Init(I2C_HandleTypeDef *hi2c);


/**
	* Return measured value for selected ID joint
	* @param ID
	* @return Sensor angular value
	*/
uint16_t angleSensor_GetValue(uint8_t ID);


#endif
