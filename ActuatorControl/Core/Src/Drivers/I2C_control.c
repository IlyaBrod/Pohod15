#include "I2C_control.h"


int i2c_read(I2C_HandleTypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data, uint8_t bytes)
{
	if(HAL_I2C_Master_Transmit(I2Cx,(uint16_t)(device_address<<1), &register_address, 1, 1000)!=HAL_OK) return 1;
	if(HAL_I2C_Master_Receive(I2Cx, (uint16_t)(device_address<<1), data, bytes, 1000)!=HAL_OK) return 1;
	else return 0;
}

int i2c_write_register(I2C_HandleTypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t data)
{
	uint8_t write_Buffer[2];
	write_Buffer[0] = register_address;
	write_Buffer[1] = data;

	if(HAL_I2C_Master_Transmit(I2Cx, (uint16_t)(device_address<<1), (uint8_t *)write_Buffer, 2, 1000)!=HAL_OK) return 1;
	else return 0;
}


int i2c_write(I2C_HandleTypeDef* I2Cx, uint8_t device_address, uint8_t data)
{
	uint8_t write_Buffer[1]; //Update to multiple bytes writing
	write_Buffer[0] = data;
	
	if(HAL_I2C_Master_Transmit(I2Cx, (uint16_t)(device_address<<1), (uint8_t *)write_Buffer, 1, 1000)!=HAL_OK) return 1;
	else return 0;
}
