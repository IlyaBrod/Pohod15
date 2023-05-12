/**
	*	One joint object
	*
	*/

#ifndef POHOD_JOINT
#define POHOD_JOINT

#include "stm32f4xx_hal.h"
#include "../Drivers/PWM_control.h"
#include "angularSensor_driver.h"
#include "LEG_CONFIG.h"
#include "main.h"

#define CLK 1    //clockwise
#define CCLK 0   //counter clockwise

typedef struct
	{
	//PWM Settings
	TIM_HandleTypeDef *pwmTim;
	uint32_t pwmChannel;
	int CMD; //PWM writing value (Maximum = PWM_PERIOD).

	//I2C sensor
	I2C_HandleTypeDef *hi2c;
	uint16_t ANGLE;
	float ANGLE_DEG;
	float SPEED;			//RPM
	
	//Global Settings
	uint8_t ENABLE;  //1 if ACTIVE, 0 if INACTIVE
	uint8_t STEADY;	 //1 if ACTIVE, 0 if INACTIVE (power safe mode)
	uint8_t ID;		 //Joint unique number
	uint16_t TARGET; //Target angle
	uint8_t DIR; //Direction polarity (defined on initialization only).
	double ERR;
}JOINT;


void jointInit(JOINT *jnt);
void jointPreUpdate(JOINT *jnt); //Update angular information
void jointPostUpdate(JOINT *jnt); //Write PWM data


//PRIVATE FUNCTIONS
void jointSetDirection(uint8_t ID,uint8_t DIR);
#endif
