#ifndef TCA_CONTROL
#define TCA_CONTROL

#include "stm32f4xx_hal.h"
#include "../Drivers/I2C_control.h"

//TCA4598A I2C multiplexer
#define TCA_ADDR 0x70

#define TCA_CHANNEL_0 0x1
#define TCA_CHANNEL_1 0x2
#define TCA_CHANNEL_2 0x4
#define TCA_CHANNEL_3 0x8
#define TCA_CHANNEL_4 0x10
#define TCA_CHANNEL_5 0x20
#define TCA_CHANNEL_6 0x40
#define TCA_CHANNEL_7 0x80
#define TCA_CHANNEL_DISABLE_ALL 0x00

void TCA_Init(I2C_HandleTypeDef *hi2c);

/*
 * Open channel
 * @param CHANNEL Number of the channel to open (use TCA_CHANNEL_X)
 */
void TCA_open(uint8_t CHANNEL);

/*
 * Close all the TCA channels
 */
void TCA_close(void);



#endif
