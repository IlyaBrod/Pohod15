#include "AS5145_driver.h"


AS5145 Sensor1 = {0};
AS5145 Sensor2 = {0};
AS5145 Sensor3 = {0};

AS5145 * Sensors[3];
uint64_t buffer;


void AS5145_Init(TIM_HandleTypeDef *htim)
{
	SSI_Init(htim);
	
	Sensors[0] = &Sensor1;
	Sensors[1] = &Sensor2;
	Sensors[2] = &Sensor3;
}

void AS5145_Read(void)
{	
	
	SSI_Read(&buffer,18); //19*3
	//Extract the 3 data sequences with errors bits
	
	for(int i=0;i<3;i++) {
		Sensors[i]->buffer = ( (buffer & (0x7FFFF<<(19*i))) >>(19*i+1)   );
		Sensors[i]->CalcPar = get_Parity(Sensors[i]->buffer,PARITY_EVEN);
		
		//Extract data
		Sensors[i]->angle = Sensors[i]->buffer >> 6;
		Sensors[i]->OCF = (Sensors[i]->buffer & 0x20) >> 5;
		Sensors[i]->COF = (Sensors[i]->buffer & 0x10) >> 4;
		Sensors[i]->LIN = (Sensors[i]->buffer & 0x08) >> 3;
		Sensors[i]->MagInc = (Sensors[i]->buffer & 0x04) >> 2;
		Sensors[i]->MagDec = (Sensors[i]->buffer & 0x02) >> 1;
		Sensors[i]->EvenPar = (Sensors[i]->buffer & 0x01);
		
	}
	
}

void AS5145_update(uint8_t passes)
{
	uint8_t step = 0;
	
	for(int i=0;i<3;i++) Sensors[i]->ready = 0;
	AS5145_Read();
	
	while((Sensors[0]->ready==0 || Sensors[1]->ready==0 || Sensors[2]->ready==0) && step<passes) {
		step++;
		for(int i=0;i<3;i++) {
			
			// Sensor NOT READY
			if(Sensors[i]->ready==0) {
					//Check parity bits
					//if(Sensors[i]->EvenPar == Sensors[i]->CalcPar) {				
							//Check magnet distance
							if(Sensors[i]->LIN == 0 && Sensors[i]->COF == 0) {
								//Check end of offset compensation
									if(Sensors[i]->OCF == 1) {
											Sensors[i]->ready=1;
											Sensors[i]->angle_valid=Sensors[i]->angle;
									//}
							}
					}
			 }
		 }
	 }
}


uint16_t AS5145_GetValue(uint8_t ID)
{
	return Sensors[ID]->angle_valid;
}

