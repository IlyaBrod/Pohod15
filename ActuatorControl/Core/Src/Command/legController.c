#include "legController.h"

#ifdef DEBUG_SAVE_IO
uint16_t input_save[1000] = {0};
uint16_t output_save[1000] = {0};
int cmd_save[1000] = {0};
uint16_t index_save = 0;
uint8_t io_start = 0;
#endif

CONTROLLER Leg = {0};
JOINT jnt1;
JOINT jnt2;
JOINT jnt3;
//int sysclk_period =0;

uint16_t AGL_LIMITS[6] = {ID1_AGL_MIN,ID2_AGL_MIN,ID3_AGL_MIN,
						  ID1_AGL_MAX,ID2_AGL_MAX,ID3_AGL_MAX};

uint16_t seuil(uint8_t ID, uint16_t val); //Limits angles values to extremes
void zeros(float * memory,unsigned int size); //Init array to zeros			
void Delay_ns(unsigned int delay);														

														//Init Settings
void LegInit(TIM_HandleTypeDef *htimPWM, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htimINT)
{
	//Clock period calculation
	//int sysclk = HAL_RCC_GetSysClockFreq();
	//float period = ((float)1)/((float)sysclk);
	//period = period*1000000000;
	//sysclk_period = (int)(period);
	
	
	Leg.joints[0] = &jnt1;
	Leg.joints[1] = &jnt2;
	Leg.joints[2] = &jnt3;
	
	Leg.joints[0]->ENABLE = 0;
	Leg.joints[1]->ENABLE = 0;
	Leg.joints[2]->ENABLE = 0;
	
	#ifdef JOINT1
	Leg.joints[0]->ID = ID1;
	Leg.joints[0]->DIR = ID1_POLARITY;
	Leg.joints[0]->pwmTim = htimPWM;
	Leg.joints[0]->TARGET = AGL_LIMITS[ID1];
	Leg.joints[0]->pwmChannel = TIM_CHANNEL_1;
	Leg.joints[0]->hi2c = hi2c;
	jointInit(Leg.joints[0]);
	#endif
	
	#ifdef JOINT2
	Leg.joints[1]->ID = ID2;
	Leg.joints[1]->DIR = ID2_POLARITY;
	Leg.joints[1]->pwmTim = htimPWM;
	Leg.joints[1]->TARGET = AGL_LIMITS[ID2];
	Leg.joints[1]->pwmChannel = TIM_CHANNEL_2;
	Leg.joints[1]->hi2c = hi2c;
	jointInit(Leg.joints[1]);
	#endif
	
	#ifdef JOINT3
	Leg.joints[2]->ID = ID3;
	Leg.joints[2]->DIR = ID3_POLARITY;
	Leg.joints[2]->pwmTim = htimPWM;
	Leg.joints[2]->TARGET = AGL_LIMITS[ID3];
	Leg.joints[2]->pwmChannel = TIM_CHANNEL_3;
	Leg.joints[2]->hi2c = hi2c;
	jointInit(Leg.joints[2]);
	#endif
	
	
	//Init controller parameters
	for(int i=0;i<3;i++) {
		for(int j=0;j<=FILTER_NA;j++) {
			Leg.a[i][j] = 0;
		}
	}
	
	for(int i=0;i<3;i++) {
		for(int j=0;j<=FILTER_NB;j++) {
			Leg.b[i][j] = 0;
		}
	}
	
		for(int i=0;i<3;i++) {
		for(int j=0;j<=FILTER_N;j++) {
			Leg.w[i][j] = 0;
		}
	}
	
	zeros(Leg.speed_memory,3);
	zeros(Leg.pos_memory,3);
	
	//Start interruption on TIMER
	Leg.htim = htimINT;
	
}

void LegControllerStart(void)
{
		HAL_TIM_Base_Start_IT(Leg.htim);
}

void LegControllerStop(void)
{
	HAL_TIM_Base_Stop_IT(Leg.htim);
}

void LegSetupController(double j1b[FILTER_NB+1],double j1a[FILTER_NA+1],double j2b[FILTER_NB+1],double j2a[FILTER_NA+1],double j3b[FILTER_NB+1],double j3a[FILTER_NA+1])
{
	for(int i =0;i<=FILTER_NB;i++){
		Leg.b[0][i] = j1b[i];
	}
	
	for(int i =0;i<=FILTER_NA;i++){
		Leg.a[0][i] = j1a[i];
	}
	
	for(int i =0;i<=FILTER_NB;i++){
		Leg.b[1][i] = j2b[i];
	}
	
	for(int i =0;i<=FILTER_NA;i++){
		Leg.a[1][i] = j2a[i];
	}
	
		for(int i =0;i<=FILTER_NB;i++){
		Leg.b[2][i] = j3b[i];
	}
	
	for(int i =0;i<=FILTER_NA;i++){
		Leg.a[2][i] = j3a[i];
	}


}

//#################################################### LEG USER FUNCTIONS

void LegReset(void)
{
	
	for(int i=0;i<3;i++)
	{
		if(Leg.joints[i]->ENABLE==1) {
			Leg.joints[i]->TARGET = (AGL_LIMITS[i+3]-AGL_LIMITS[i])/2+AGL_LIMITS[i];
		}
	}
}

void LegSet(uint8_t ID, uint16_t POS){
	
	Leg.joints[ID]->TARGET = seuil(ID,POS);
}

void LegSetAll(uint16_t POS1, uint16_t POS2,uint16_t POS3)
{
	LegSet(ID1,POS1);
	LegSet(ID2,POS2);
	LegSet(ID3,POS3);
}

void LegSetWait(uint8_t ID, uint16_t POS)
{
	LegSet(ID,POS);
	while(Leg.joints[ID]->ERR>ERROR_BEFORE_WAIT)
	{
		Delay_ns(77);
	}
}

void LegSetAllWait(uint16_t POS1,uint16_t POS2,uint16_t POS3)
{
	LegSet(ID1,POS1);
	LegSet(ID2,POS2);
	LegSet(ID3,POS3);
	
	
	while(Leg.joints[ID1]->ERR>ERROR_BEFORE_WAIT || Leg.joints[ID2]->ERR>ERROR_BEFORE_WAIT || Leg.joints[ID3]->ERR>ERROR_BEFORE_WAIT)
	{
		Delay_ns(77);
	}
	
}

void JointDisable(uint8_t ID) {

	Leg.joints[ID]->ENABLE = 0;

}
void JointEnable(uint8_t ID) {
	Leg.joints[ID]->ENABLE = 1;
}



//######################################################

// PRIVATE FUNCTION

void zeros(float * memory,unsigned int size) {

	for(int i=0;i<size;i++)
	{
		memory[i]=0;
	}
}

/*void Delay_ns(unsigned int delay)
{
	unsigned int ns =  (unsigned int)delay/sysclk_period/7;
	for(int i=0;i<=ns;i++);
}*/

uint16_t seuil(uint8_t ID, uint16_t val){
	uint16_t valS=val;
	uint16_t max = AGL_LIMITS[ID+3];
	uint16_t min = AGL_LIMITS[ID];
	
	if(val>max) valS = max;
	else if(val<min) valS = min;
	
	return valS;
}


void DEBUG_ioStart(void) {
	io_start = 1;
}

void LegUpdate(TIM_HandleTypeDef *htim)
{	
	 #ifdef DEBUG_TECH
	 uint32_t tic = HAL_GetTick();
	 #endif

	 if(__HAL_TIM_GET_FLAG(Leg.htim,TIM_FLAG_UPDATE) != RESET) {
		 
		 double ctrl=0;

		 for(int i=0; i<3;i++)
		 {
			 if(Leg.joints[i]->ENABLE == 1) {
				 
				 
				 jointPreUpdate(Leg.joints[i]); //Read sensors values


				 #ifdef DEBUG_ANGLE_DEG
				 Leg.joints[i]->ANGLE_DEG =  (float)(Leg.joints[i]->ANGLE * ANGLE_SENSIVITY_DEG);
				 #endif

				 #ifdef DEBUG_SPEED
				 float spd = ((Leg.joints[i]->ANGLE*ANGLE_SENSIVITY_DEG-Leg.pos_memory[i])/0.003f * 60.0f/360.0f);
				 Leg.joints[i]->SPEED = (Leg.speed_memory[i]+ spd)/2.0f;
				 Leg.pos_memory[i] = Leg.joints[i]->ANGLE*ANGLE_SENSIVITY_DEG;
				 Leg.speed_memory[i] = spd;
				 #endif

				 Leg.joints[i]->ERR = (float)(Leg.joints[i]->TARGET-Leg.joints[i]->ANGLE)*ANGLE_SENSIVITY_RAD;
				 Leg.joints[i]->CMD = (int)(Leg.joints[i]->ERR * (30.0f) *VOLT_TO_PWM);

				 /*
				 Leg.w[i][0] = (Leg.joints[i]->TARGET* ANGLE_SENSIVITY_RAD)- (Leg.joints[i]->ANGLE*ANGLE_SENSIVITY_RAD);		 
				 Leg.w[i][0] =  atan2(sin(Leg.w[i][0]),cos(Leg.w[i][0]));
				 Leg.joints[i]->ERR = Leg.w[i][0];
				 
				 
				 for(int k=1;k<=FILTER_NB;k++) {
					 Leg.w[i][0] -= Leg.a[i][k] * Leg.w[i][k];
				 }
				 
				 for(int k=0;k<=FILTER_NA;k++) {
					 ctrl += Leg.b[i][k] * Leg.w[i][k];
				 }
				 
				 for(int k=FILTER_N;k>=1;k--) {
					 Leg.w[i][k] = Leg.w[i][k-1];
				 }
				 
				 ctrl = -1000.1f*ctrl;
				 
				 Leg.joints[i]->CMD = (int)(ctrl*VOLT_TO_PWM);
				*/
				 //COMMAND SATURATION
				 if(Leg.joints[i]->CMD>PWM_PERIOD) {
					 Leg.joints[i]->CMD = PWM_PERIOD;
				 }
				 else if(Leg.joints[i]->CMD<-PWM_PERIOD) {
					 Leg.joints[i]->CMD = -PWM_PERIOD;
				 }
				 
				 jointPostUpdate(Leg.joints[i]); //Update motor direction and command

				 #ifdef DEBUG_SAVE_IO
				 if(index_save<1000 && io_start==1) {
					 input_save[index_save] = Leg.joints[i]->TARGET;
				 	 output_save[index_save] = Leg.joints[i]->ANGLE;
				 	 cmd_save[index_save] = Leg.joints[i]->CMD;
				 	 index_save = index_save +1;
				 }
				 #endif
			 }
     }

		 
		 
		 #ifdef DEBUG_TECH
		 uint32_t toc = HAL_GetTick();
		 Leg.Tech = (float)(toc-tic);
		 Leg.Tech = ((float)(HAL_RCC_GetSysClockFreq()));
		 #endif
	 }
}

