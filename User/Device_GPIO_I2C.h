/*
 * Device_GPIO_I2C.h
 *
 *  Created on: Dec 3, 2024
 *      Author: GW
 */

#ifndef USER_DEVICE_GPIO_I2C_H_
#define USER_DEVICE_GPIO_I2C_H_

void Device_GPIO_I2C_Init(uint8_t status);

uint8_t Device_I2C_ReadReg(uint8_t reg_addr);
void Device_I2C_WriteReg(uint8_t reg_addr, uint8_t data);


#endif /* USER_DEVICE_GPIO_I2C_H_ */
