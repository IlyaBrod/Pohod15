#include "angularSensor_driver.h"

I2C_HandleTypeDef *hi2cLoc;

void angularSensor_Init(I2C_HandleTypeDef *hi2c) {
	
	TCA_Init(hi2c);
	hi2cLoc = hi2c;
	
}


uint16_t angleSensor_GetValue(uint8_t ID) {
	
	uint8_t data[2];
	uint16_t output;

	switch(ID) {
	case ID1:
		TCA_open(TCA_CHANNEL_5);
		break;
	case ID2:
		TCA_open(TCA_CHANNEL_6);
		break;
	case ID3:
		TCA_open(TCA_CHANNEL_7);
		break;
	}

	 //Automatic increment angle register
	i2c_read(hi2cLoc,(ANGLE_I2C_ADDR_0),ANGLE_I2C_REGISTER_HI, data,2);
	output = (data[0]<<8) + data[1]; //MSB first [0], so inversion of the buffer
	
	TCA_close();

	return output;
	
}
