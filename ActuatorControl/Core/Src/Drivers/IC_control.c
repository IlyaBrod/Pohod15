#include "IC_control.h"

uint32_t IC_Val1[3] = {0};
uint32_t IC_Val2[3] = {0};
uint32_t Difference[3] = {1,2,3};
uint32_t DifferenceDMA[3] = {1,2,3};

uint8_t Is_First_Captured[3] = {0};  // is the first value captured ?
uint32_t angularValue[3] = {0};

uint32_t DMA_data1[3];
uint32_t DMA_data2=0;
uint32_t DMA_data3=0;



void ic_start(TIM_HandleTypeDef *htim,uint32_t channel,uint8_t ID) {
	HAL_TIM_IC_Start_DMA(htim,channel,DMA_data1,4);
	
	/*switch(ID){
		
		case 0:
			HAL_TIM_IC_Start_DMA(htim,channel,DMA_data1,3);
			break;
		
		case 1:
			HAL_TIM_IC_Start_DMA(htim,channel,&DMA_data2,1);
			break;
		
		case 2:
			HAL_TIM_IC_Start_DMA(htim,channel,&DMA_data3,1);
			break;
	}*/
}

uint32_t ic_read(uint8_t ID)
{
	return angularValue[ID];
}


// PRIVATE FUNCTIONS

void DMA_IC_to_PWM(TIM_HandleTypeDef *htim,uint8_t ID)
{
	DifferenceDMA[ID]+=1;
}



void HAL_TIM_IC_CALL(TIM_HandleTypeDef *htim)
{
	uint8_t ID=3;
	uint32_t CH;
	if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET){ID = 0; CH=TIM_CHANNEL_1;}
	else if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET){ ID = 1; CH=TIM_CHANNEL_2;}
	else if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET){ ID = 2; CH=TIM_CHANNEL_3;}
		
		
	if(ID!=3)
	{
		if (Is_First_Captured[ID]==0) // if the first value is not captured
		{
			IC_Val1[ID] = HAL_TIM_ReadCapturedValue(htim, CH); // read the first value
			Is_First_Captured[ID] = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, CH, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured[ID]==1)   // if the first is already captured
		{
			IC_Val2[ID] = HAL_TIM_ReadCapturedValue(htim, CH);  // read second value
			__HAL_TIM_SET_COUNTER(htim, CH);  // reset the counter

			if (IC_Val2[ID] > IC_Val1[ID])
			{
				//Difference[ID] = IC_Val2[ID]-IC_Val1[ID];
				//angularValue[ID] = 360.0/4096.0 * (float)(Difference[ID]); 
				Difference[ID] +=1;
			}

			Is_First_Captured[ID] = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, CH, TIM_INPUTCHANNELPOLARITY_RISING);

		}
	}

}






