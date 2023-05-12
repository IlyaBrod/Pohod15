#include "joint.h"

void jointInit(JOINT *jnt)
{
	jnt->ANGLE = 0;
	jnt->ERR = 0;
	jnt->CMD = 0;
	jnt->ENABLE = 1;
	jnt->STEADY = 1;
	
	pwm_start(jnt->pwmTim,jnt->pwmChannel);
	pwm_write(jnt->pwmTim,jnt->pwmChannel,0);
	
	angularSensor_Init(jnt->hi2c);
}

void jointPreUpdate(JOINT *jnt)
{	
		//AS5145_update(1);
		jnt->ANGLE = angleSensor_GetValue(jnt->ID);
}

void jointPostUpdate(JOINT *jnt)
{
	if(jnt->CMD>=0){
			uint16_t PWM = (jnt->CMD);
			jointSetDirection(jnt->ID,jnt->DIR);
			pwm_write(jnt->pwmTim,jnt->pwmChannel,PWM);
		}
		else{
			uint16_t PWM = -(jnt->CMD);
			jointSetDirection(jnt->ID,(1-(jnt->DIR))&1);
			pwm_write(jnt->pwmTim,jnt->pwmChannel,PWM);
		}
}

//PRIVATE FUNCTIOS
void jointSetDirection(uint8_t ID,uint8_t DIR)
{
	if(DIR==CLK){
		switch(ID){
			case ID1:

				HAL_GPIO_WritePin(M1_IN1_GPIO_Port,M1_IN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port,M1_IN2_Pin,GPIO_PIN_RESET);
				break;
				
			case ID2:
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port,M2_IN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port,M2_IN2_Pin,GPIO_PIN_RESET);
				break;
			
			case ID3:
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port,M3_IN1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port,M3_IN2_Pin,GPIO_PIN_RESET);
				break;
		}		
	}
	else {
		switch(ID){
			case ID1:
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port,M1_IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port,M1_IN2_Pin,GPIO_PIN_SET);
				break;
			
			case ID2:
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port,M2_IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port,M2_IN2_Pin,GPIO_PIN_SET);
				break;
			
			case ID3:
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port,M3_IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port,M3_IN2_Pin,GPIO_PIN_SET);
				break;
		}
	}
}

