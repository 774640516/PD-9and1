/*
 * my_I2C_Device.c
 *
 *  Created on: Mar 27, 2024
 *      Author: GW
 */

#include "debug.h"
#include "my_I2C_Device.h"
#include "Device_GPIO_I2C.h"
volatile MY_I2C_CONTROL my_i2c_control;
SC8726_Device sc8726;

uint8_t Read_Buff[20];
uint8_t Write_Buff[20];

uint8_t my_get_status()
{
    return my_i2c_control.Status;
}

void my_SC8726_Init()
{
    sc8726.Init_Status = 0;
    sc8726.Out_Status = 0;
    sc8726.Set_Out_Vbus = 0;
    sc8726.Out_Vbus = 0;
}

void my_SC8726_Set_Out_Vbus(uint16_t Vbus, uint16_t Current)
{
    if (Vbus < 5000)
    {
        printf("Vbus = %d  error  <5V\r\n", Vbus);
        sc8726.Set_Out_Vbus = 5000;
    }
    else if (Vbus > 12000)
    {
        printf("Vbus = %d  error  >12V\r\n", Vbus);
        sc8726.Set_Out_Vbus = 12000;
    }
    else
        sc8726.Set_Out_Vbus = Vbus;

    sc8726.Set_Out_Current = (uint8_t)(Current * 4 / 5);
    printf("sc8726.Set_Out_Current = %d\r\n", sc8726.Set_Out_Current);

    if (sc8726.Out_Status == 0)
    {
        sc8726.Out_Status = 3;
    }
}
void my_SC8726_Close_Vbus()
{
    if (sc8726.Out_Status == 0)
    {
        sc8726.Out_Status = 1;
    }
    sc8726.Set_Out_Vbus = 0;
}
uint8_t get_SC8726_Vbus_Out_Status()
{
    return sc8726.Out_Status;
}

void my_SC8726_Init_Handle()
{
    uint8_t status, i;
    switch (sc8726.Init_Status)
    {
    case 0:
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
        sc8726.Time = 3;
        sc8726.Init_Status = 1;
        break;
    case 1:
        if (sc8726.Time > Tmr_Ms_Dlt)
            sc8726.Time -= Tmr_Ms_Dlt;
        else
            sc8726.Time = 0;
        if (sc8726.Time == 0)
        {
            sc8726.Init_Status = 2;
            for (uint8_t i = 0; i < 10; i++)
            {
                sc8726.read_I2C_Buff[i] = Device_I2C_ReadReg(i + 1);
            }
        }
        break;
    case 2:
        for (i = 0; i < 10; i++)
        {
            printf("%d  ", sc8726.read_I2C_Buff[i]);
        }
        printf("\r\n");
        sc8726.Init_Status = 3;
        sc8726.write_I2C_Buff[0] = sc8726.read_I2C_Buff[4] | 0x04;
        Device_I2C_WriteReg(0x05, sc8726.write_I2C_Buff[0]);

        break;
    case 3:
        printf("DIS DCDC 1 ok \r\n");
        sc8726.Init_Status = 4;
        break;
    case 4:
        break;
    }
}

void my_SC8726_Control_Handle()
{
    uint8_t status;
    static uint16_t set_Vbus;
    switch (sc8726.Out_Status)
    {
    case 0:
        break;
    case 1: // 关输出
        sc8726.write_I2C_Buff[0] = sc8726.read_I2C_Buff[4] | 0x04;
        Device_I2C_WriteReg(0x05, sc8726.write_I2C_Buff[0]);
        sc8726.Out_Status = 2;
        break;
    case 2:
        printf("DIS DCDC 1 ok \r\n");
        sc8726.Out_Status = 0;
        sc8726.Set_Out_Vbus = 0;
        sc8726.Out_Vbus = 0;
        break;
    case 3:
        set_Vbus = sc8726.Set_Out_Vbus - 4780;
        set_Vbus /= 20;

        sc8726.write_I2C_Buff[0] = sc8726.Set_Out_Current;
        sc8726.write_I2C_Buff[1] = sc8726.read_I2C_Buff[1] | 0x02;
        sc8726.write_I2C_Buff[2] = set_Vbus / 0x04;
        sc8726.write_I2C_Buff[3] = ((sc8726.read_I2C_Buff[3] & 0xe0) | 0x18) + (set_Vbus % 0x04);
        for (uint8_t i = 0; i < 4; i++)
        {
            Device_I2C_WriteReg(i + 0x01, sc8726.write_I2C_Buff[i]);
        }
        set_Vbus = sc8726.Set_Out_Vbus;
        sc8726.Out_Status = 4;
        break;
    case 4:
        printf("Settings Vbus %d mV success\r\n", set_Vbus);
        sc8726.Out_Status = 5;
        if (sc8726.Out_Vbus == 0)
            sc8726.Time = 12;
        else
            sc8726.Time = 1;
        break;
    case 5:
        if (sc8726.Time > Tmr_Ms_Dlt)
            sc8726.Time -= Tmr_Ms_Dlt;
        else
            sc8726.Time = 0;
        if (sc8726.Time == 0)
        {
            sc8726.Out_Status = 6;
            sc8726.write_I2C_Buff[0] = (sc8726.read_I2C_Buff[4] & (~0x04)) | 0x02;
            Device_I2C_WriteReg(0x05, sc8726.write_I2C_Buff[0]);
        }
        break;
    case 6:
        printf("Out Vbus %d mV success\r\n", set_Vbus);
        sc8726.Out_Vbus = set_Vbus;
        if (set_Vbus != sc8726.Set_Out_Vbus)
        {
            if (sc8726.Set_Out_Vbus != 0)
                sc8726.Out_Status = 3;
            else
                sc8726.Out_Status = 1;
        }
        else
        {
            sc8726.Out_Status = 0;
        }
        GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET); // 使能电源输出
        break;
    }
}

void my_SC8726_Handle()
{
    if (sc8726.Init_Status < 4)
    {
        my_SC8726_Init_Handle();
    }
    else if (sc8726.Init_Status == 4)
    {
        my_SC8726_Control_Handle();
    }
}
