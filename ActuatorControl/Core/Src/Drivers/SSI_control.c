#include "SSI_control.h"


int sysclk_period = 0;
uint32_t counter=0;
uint32_t max_counter=0;
uint64_t localBuffer=0;
uint8_t SSI_state = 0;

TIM_HandleTypeDef *htimLoc;

void SSI_Init(TIM_HandleTypeDef *htim)
{
	//Init ports state
	HAL_GPIO_WritePin(GPIO_CS,PIN_CS,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_CLK,PIN_CLK,GPIO_PIN_SET);
	
	//Clock period calculation
	int sysclk = HAL_RCC_GetSysClockFreq();
	float period = ((float)1)/((float)sysclk);
	period = period*1000000000;
	sysclk_period = (int)(period);
	

	//Init Interrupt
	htimLoc = htim;
	HAL_TIM_Base_Start_IT(htim);
	
	SSI_state=0;
}


void SSI_Read(uint64_t *buffer, uint32_t bits_count)
{
	
	counter = 0;
	max_counter = bits_count-1;
	localBuffer=0;
	
	HAL_GPIO_WritePin(GPIO_CS,PIN_CS,GPIO_PIN_RESET);
	Delay_ns(500);
	HAL_GPIO_WritePin(GPIO_CLK,PIN_CLK,GPIO_PIN_RESET);
	
	SSI_state=1;
	HAL_TIM_Base_Start_IT(htimLoc);
	while(SSI_state) {Delay_ns(100);};
	HAL_TIM_Base_Stop_IT(htimLoc);
	(*buffer)=localBuffer;
	HAL_GPIO_WritePin(GPIO_CS,PIN_CS,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_CLK,PIN_CLK,GPIO_PIN_SET);
	
}


void SSI_update(void)
{
	if(SSI_state) {

		HAL_GPIO_WritePin(GPIO_CLK,PIN_CLK,GPIO_PIN_SET);
		Delay_ns(500); //420 default, under not working
		localBuffer += HAL_GPIO_ReadPin(GPIO_DATA,PIN_DATA);
		
		if(counter==max_counter)
		{
			SSI_state=0;
		}
		else {
			localBuffer = localBuffer<<1;
			counter++;
			HAL_GPIO_WritePin(GPIO_CLK,PIN_CLK,GPIO_PIN_RESET);
		}

		}	
}



void Delay_ns(unsigned int delay)
{	
	unsigned int ns =  (unsigned int)delay/sysclk_period/7;
	for(int i=0;i<=ns;i++);
}

uint8_t get_Parity(unsigned int word, uint8_t parityType) {
	
	/* V1
	uint64_t checksum = 0;
	for(int i=0;i<bits_count;i++) 
	{
		uint64_t bit = word & 0x01<<i;
		checksum = (checksum + bit)%2;
	}
	
	if(checksum == 0) return parityType;
	else return (parityType+1)%2;
	*/
	
	//V2
	int y = word ^ (word >> 1); 
  y = y ^ (y >> 2); 
  y = y ^ (y >> 4); 
  y = y ^ (y >> 8); 
  y = y ^ (y >> 16); 
  
  // Rightmost bit of y holds the parity value 
  // if (y&1) is 1 then parity is odd else even 
	
  if (y & 1) return (parityType ^ 1); //return 1 when even parity
  return parityType;  //return 0
	
}
