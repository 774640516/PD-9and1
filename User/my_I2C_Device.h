/*
 * my_I2C_Device.h
 *
 *  Created on: Mar 27, 2024
 *      Author: GW
 */

#ifndef USER_MY_I2C_DEVICE_H_
#define USER_MY_I2C_DEVICE_H_

extern UINT8  Tmr_Ms_Dlt;

typedef struct
{
    uint8_t Status;                     //  10 write    0 read
    uint8_t read_Addr;
    uint8_t write_Addr;
    uint8_t read_Len;
    uint8_t write_Len;
    uint8_t count;
    uint8_t *read_buff;
    uint8_t *write_buff;

    uint8_t control_status;
    uint16_t control_time;
}MY_I2C_CONTROL;

typedef struct
{
    uint8_t Init_Status;                     //  0 write    0read
    uint8_t Out_Status;
    uint16_t Time;
    uint16_t Set_Out_Vbus;
    uint8_t Set_Out_Current;
    uint16_t Out_Vbus;
    uint8_t read_I2C_Buff[10];
    uint8_t write_I2C_Buff[10];
}SC8726_Device;




void IIC_Init(u32 bound, u16 address);
void my_i2c_read_addr(uint8_t addr,uint8_t len,uint8_t *read_buff);
void my_i2c_write_addr(uint8_t addr,uint8_t len,uint8_t *write_buff);
uint8_t my_get_status();


void my_SC8726_Handle();
void my_SC8726_Close_Vbus();
void my_SC8726_Set_Out_Vbus(uint16_t Vbus,uint16_t Current);
void my_SC8726_Init();
uint8_t get_SC8726_Vbus_Out_Status();
#endif /* USER_MY_I2C_DEVICE_H_ */
