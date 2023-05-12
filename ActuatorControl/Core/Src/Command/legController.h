/**
	*	Functions to control Pohod Legs
	*
	*
	*							 J1						O  o					J1'
	*							/   \				|	BODY |			/			\
	*						/				J2---J3			J3'---J2'				\
	*				   V 																		 V
	*/

#ifndef CONTROL_MOTOR
#define CONTROL_MOTOR

#include "stm32f4xx_hal.h"
#include "joint.h"
#include "LEG_CONFIG.h"
#include <math.h>

typedef struct{
	TIM_HandleTypeDef *htim; //Interrupt Timer
	double a[3][FILTER_NA+1];
	double b[3][FILTER_NB+1];
	double w[3][FILTER_N+1];  //doit etre w[3][...]
	
	float speed_memory[3];
	float pos_memory[3];
	
	JOINT *joints[3];
	float Tech;
}CONTROLLER;

//Initialization functions
/**
 * Setup controller timers and initialize joints variables.
 * @param htimPWM PWM timer
 * @param hi2c I2C timer for sensors communication
 * @param htimINT Control interruption timer
 */
void LegInit(TIM_HandleTypeDef *htimPWM, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htimINT);
/**
 * Save controller parameters for each joint
 * @param jxb Numerator coefficient of joint x
 * @param jxa Denominator coefficient of joint x
 */
void LegSetupController(double j1b[FILTER_NB+1],double j1a[FILTER_NA+1],double j2b[FILTER_NB+1],double j2a[FILTER_NA+1],double j3b[FILTER_NB+1],double j3a[FILTER_NA+1]);

/**
 * Start controller interruption
 */
void LegControllerStart(void);
/**
 * Stop controller interruption
 */
void LegControllerStop(void);

// Control functions
/**
 * Move to initial angles position.
 */
void LegReset(void);
void LegSet(uint8_t ID, uint16_t POS);
void LegSetAll(uint16_t POS1, uint16_t POS2,uint16_t POS3);
void LegSetWait(uint8_t ID, uint16_t POS);
void LegSetAllWait(uint16_t POS1,uint16_t POS2,uint16_t POS3);
void JointDisable(uint8_t ID);
void JointEnable(uint8_t ID);

void DEBUG_ioStart(void);

// Private functions
void LegUpdate(TIM_HandleTypeDef *htim);
#endif
