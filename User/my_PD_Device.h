/*
 * my_PD_Device.h
 *
 *  Created on: Apr 17, 2024
 *      Author: GW
 */

#ifndef USER_MY_PD_DEVICE_H_
#define USER_MY_PD_DEVICE_H_

#include "debug.h"

void my_PD_Handle();
void my_TIM_Handle();
void my_VBus_Countrol(uint8_t Vbus);
void GPIO_Toggle_INIT(void);

void set_PD_STATUS_SRC();
void set_PD_STATUS_SNK();

void my_PD_Status_init();
uint8_t get_PD_Status();
void TIM1_Init(uint16_t arr, uint16_t psc);

void my_usb_init_send(uint8_t valve_switch);
void my_usb_send(uint8_t valve_switch,uint8_t valve_check);
#endif /* USER_MY_PD_DEVICE_H_ */
