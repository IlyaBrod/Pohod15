#include "TCA_control.h"

I2C_HandleTypeDef *hi2cLoc;

void TCA_Init(I2C_HandleTypeDef *hi2c) {
	hi2cLoc = hi2c;
}

void TCA_open(uint8_t CHANNEL) {

	i2c_write(hi2cLoc, TCA_ADDR,CHANNEL);
}


void TCA_close(void) {

	i2c_write(hi2cLoc, TCA_ADDR, TCA_CHANNEL_DISABLE_ALL);

}
