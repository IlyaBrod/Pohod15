/**
	* Fichier de contrôle i2c.
	* Attention : HAL_I2C_XX(&ic2, Adress <<1 ... Il faut shift l'adresse de 1. D'après la datasheet de la lib.
	* OMG !
	*
*/

#ifndef BSP_I2C
#define BSP_I2C

#include "stm32f4xx_hal.h"

/**
	* Read a register value and store inside buffer[]
	* @param &I2Cx The i2c HAL handler
	* @param device_adress Adress of the I2C slave device
	* @param register_adress Adress of the register to be read
	* @param data Pointer to a data array, to store the readen values.
	* @param bytes Number of bytes to read
	* @return 0 : No error, 1 : Error
	*/
int i2c_read(I2C_HandleTypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data,uint8_t bytes);

/**
	* Write a register value
	* @param &I2Cx The i2c HAL handler
	* @param device_adress Adress of the I2C slave device
	* @param register_adress Adress of the register to be written
	* @param data 8bits value to store in the register.
	* @return 0 : No error, 1 : Error
	*/
int i2c_write_register(I2C_HandleTypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t data);

/**
	* Write device
	* @param &I2Cx The i2c HAL handler
	* @param device_adress Adress of the I2C slave device
	* @param data 8bits value to store in the register.
	* @return 0 : No error, 1 : Error
	*/
int i2c_write(I2C_HandleTypeDef* I2Cx, uint8_t device_address, uint8_t data);


#endif




