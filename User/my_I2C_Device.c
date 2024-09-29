/*
 * my_I2C_Device.c
 *
 *  Created on: Mar 27, 2024
 *      Author: GW
 */

#include "debug.h"
#include "my_I2C_Device.h"

#define RXAdderss 0X62 << 1 + 1
#define TxAdderss 0X62 << 1

volatile MY_I2C_CONTROL my_i2c_control;
SC8726_Device sc8726;

#define TX_ADDR 0x62
#define RX_ADDR 0x62

#define I2C_TIME_STATUS 100
#define I2C_ERROR_STATUS 101

// uint8_t count;

uint8_t Read_Buff[20];
uint8_t Write_Buff[20];

void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void IIC_Init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitTSturcture = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_19;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
}

void my_i2c_read_addr(uint8_t addr, uint8_t len, uint8_t *read_buff)
{
    my_i2c_control.Status = 0;
    my_i2c_control.read_Len = len;
    my_i2c_control.read_Addr = addr;
    my_i2c_control.write_Len = 1;
    my_i2c_control.read_buff = read_buff;

    my_i2c_control.control_status = 0;
}
void my_i2c_write_addr(uint8_t addr, uint8_t len, uint8_t *write_buff)
{
    my_i2c_control.Status = 10;
    my_i2c_control.write_Addr = addr;
    my_i2c_control.write_Len = len;
    my_i2c_control.write_buff = write_buff;

    my_i2c_control.control_status = 0;
}

uint8_t my_get_status()
{
    return my_i2c_control.Status;
}

void I2C1_EV_IRQHandler(void)
{

    if (I2C_GetITStatus(I2C1, I2C_IT_SB) != RESET)
    {
        if (my_i2c_control.Status == 0 || my_i2c_control.Status == 10)
        {
            my_i2c_control.Status += 1;
            I2C_Send7bitAddress(I2C1, TX_ADDR << 1, I2C_Direction_Transmitter);
        }
        else if (my_i2c_control.Status == 3)
        {
            my_i2c_control.Status += 1;
            I2C_Send7bitAddress(I2C1, RX_ADDR << 1, I2C_Direction_Receiver);
        }
    }
    else if (I2C_GetITStatus(I2C1, I2C_IT_ADDR) != RESET)
    {
        // printf("I2C_IT_ADDR\r\n");
        if (my_i2c_control.Status == 1)
        {
            my_i2c_control.Status = 2;
        }
        if (my_i2c_control.Status == 4)
        {
            my_i2c_control.Status = 5;
            if (my_i2c_control.read_Len == 1)
            {
                I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next); // clear ack
                I2C_GenerateSTOP(I2C1, ENABLE);
            }
        }
        if (my_i2c_control.Status == 11)
        {
            my_i2c_control.Status = 12;
        }
        my_i2c_control.count = 0;
        ((void)I2C_ReadRegister(I2C1, I2C_Register_STAR2));
    }
    else if (I2C_GetITStatus(I2C1, I2C_IT_TXE) != RESET)
    {
        // printf("I2C_IT_TXE\r\n");
        if (my_i2c_control.Status == 2)
        {
            if (my_i2c_control.count < 1)
            {
                I2C_SendData(I2C1, my_i2c_control.read_Addr);
                my_i2c_control.count++;
            }
            else
            {
                I2C_GenerateSTART(I2C1, ENABLE);
                my_i2c_control.Status = 3;
                I2C_SendData(I2C1, 0xff);
            }
        }
        else if (my_i2c_control.Status == 12)
        {
            I2C_SendData(I2C1, my_i2c_control.write_Addr);
            my_i2c_control.Status = 13;
        }
        else if (my_i2c_control.Status == 13)
        {
            if (my_i2c_control.count < my_i2c_control.write_Len)
            {
                I2C_SendData(I2C1, my_i2c_control.write_buff[my_i2c_control.count]);
                my_i2c_control.count++;
            }
            else
            {
                I2C_GenerateSTOP(I2C1, ENABLE);
                my_i2c_control.Status = 0xff;
                I2C_SendData(I2C1, 0xff);
            }
        }
    }
    else if (I2C_GetITStatus(I2C1, I2C_IT_RXNE) != RESET)
    {
        // printf("I2C_IT_RXNE\r\n");
        if (my_i2c_control.Status == 5)
        {
            if (my_i2c_control.count < my_i2c_control.read_Len)
            {
                my_i2c_control.read_buff[my_i2c_control.count] = I2C_ReceiveData(I2C1);
                //                   Read_Buff[my_i2c_control.count] = I2C_ReceiveData(I2C1);
                my_i2c_control.count++;
                if (my_i2c_control.count == (my_i2c_control.read_Len - 1))
                {
                    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next); // clear ack
                    I2C_GenerateSTOP(I2C1, ENABLE);
                }
                if (my_i2c_control.count == my_i2c_control.read_Len)
                {
                    my_i2c_control.Status = 0xff;
                }
            }
            else
            {
            }
        }
    }
    else
    {
        printf("unknown i2c event \r\n");
        printf("sr1 %x \nsr2 %x   %d\r\n", I2C1->STAR1, I2C1->STAR2, my_i2c_control.Status);
    }
}

void I2C1_ER_IRQHandler(void)
{
    if (I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
        printf("I2C_IT_AF\r\n");
    }
}

uint8_t get_I2C_Status()
{
    switch (my_i2c_control.control_status)
    {
    case 0:
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == 0)
        {
            my_i2c_control.control_status = 2;
            I2C_GenerateSTART(I2C1, ENABLE);
            my_i2c_control.control_time = 500;
        }
        else
        {
            my_i2c_control.control_time = 500;
            my_i2c_control.control_status = 1;
        }
        break;
    case 1:
        if (my_i2c_control.control_time > Tmr_Ms_Dlt)
            my_i2c_control.control_time -= Tmr_Ms_Dlt;
        else
            my_i2c_control.control_time = 0;
        if (my_i2c_control.control_time == 0)
        {
            my_i2c_control.control_status = 10;
        }
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == 0)
        {
            my_i2c_control.control_status = 2;
            I2C_GenerateSTART(I2C1, ENABLE);
            my_i2c_control.control_time = 500;
        }
        break;
    case 2:
        if (my_i2c_control.control_time > Tmr_Ms_Dlt)
            my_i2c_control.control_time -= Tmr_Ms_Dlt;
        else
            my_i2c_control.control_time = 0;
        if (my_i2c_control.control_time == 0)
        {
            my_i2c_control.control_status = 11;
        }
        if (my_i2c_control.Status == 0xff)
        {
            my_i2c_control.control_status = 3;
        }
        break;
    case 3:
        return 1;
        break;
    case 10:
        printf("i2c start time out\r\n");
        return I2C_TIME_STATUS;
        break;
    case 11:
        printf("i2c error %d\r\n",my_i2c_control.Status);
        return I2C_ERROR_STATUS;
        break;
    }
    return 0;
}

void my_SC8726_Init()
{
    sc8726.Init_Status = 0;
    sc8726.Out_Status = 0;
    sc8726.Set_Out_Vbus = 0;
    sc8726.Out_Vbus = 0;
}

void my_SC8726_Set_Out_Vbus(uint16_t Vbus,uint16_t Current)
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

    // sc8726.Set_Out_Current = Current*10/12.5;
    sc8726.Set_Out_Current = (uint8_t)(Current*4/5);
    printf("sc8726.Set_Out_Current = %d\r\n",sc8726.Set_Out_Current);

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
uint8_t get_SC8726_Vbus_Out_Status(){
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
            my_i2c_read_addr(1, 10, sc8726.read_I2C_Buff);
        }
        break;
    case 2:
        status = get_I2C_Status();
        if (status == 1)
        {
            for (i = 0; i < 10; i++)
            {
                printf("%d  ", sc8726.read_I2C_Buff[i]);
            }
            printf("\r\n");
            sc8726.Init_Status = 3;
            sc8726.write_I2C_Buff[0] = sc8726.read_I2C_Buff[4] | 0x04;
            my_i2c_write_addr(5, 1, sc8726.write_I2C_Buff);
        }
        else if(status)
        {
            sc8726.Init_Status = status;
        }
        break;
    case 3:
        status = get_I2C_Status();
        if (status == 1)
        {
            printf("DIS DCDC 1 ok \r\n");
            sc8726.Init_Status = 4;
        }
        else if(status)
        {
            sc8726.Init_Status = status;
        }
        break;
    case 4:
        break;
    case I2C_TIME_STATUS: // 超时
        break;
    case I2C_ERROR_STATUS: // i2c错误
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
        my_i2c_write_addr(5, 1, sc8726.write_I2C_Buff);
        sc8726.Out_Status = 2;
        break;
    case 2:
        status = get_I2C_Status();
        if (status == 1)
        {
            printf("DIS DCDC 1 ok \r\n");
            sc8726.Out_Status = 0;
            sc8726.Set_Out_Vbus = 0;
            sc8726.Out_Vbus = 0;
        }
        else if(status)
        {
            sc8726.Out_Status = status;
        }
        break;
    case 3:
        set_Vbus = sc8726.Set_Out_Vbus - 4780;
        set_Vbus /= 20;
        
        sc8726.write_I2C_Buff[0] = sc8726.Set_Out_Current;
        sc8726.write_I2C_Buff[1] = sc8726.read_I2C_Buff[1] | 0x02;
        sc8726.write_I2C_Buff[2] = set_Vbus / 0x04;
        sc8726.write_I2C_Buff[3] = ((sc8726.read_I2C_Buff[3] & 0xe0) | 0x18) + (set_Vbus % 0x04);
        my_i2c_write_addr(1, 4, sc8726.write_I2C_Buff);


        set_Vbus = sc8726.Set_Out_Vbus;
        sc8726.Out_Status = 4;
        break;
    case 4:
        status = get_I2C_Status();
        if (status == 1)
        {
            printf("Settings Vbus %d mV success\r\n", set_Vbus);
            sc8726.Out_Status = 5;
            if(sc8726.Out_Vbus == 0)sc8726.Time = 12;
            else sc8726.Time = 1;
        }
        else if(status)
        {
            sc8726.Out_Status = status;
        }
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
            my_i2c_write_addr(5, 1, sc8726.write_I2C_Buff);
        }
        break;
    case 6:
        status = get_I2C_Status();
        if (status == 1)
        {
            printf("Out Vbus %d mV success\r\n", set_Vbus);
            sc8726.Out_Vbus = set_Vbus;
            if(set_Vbus != sc8726.Set_Out_Vbus){
                if(sc8726.Set_Out_Vbus != 0)sc8726.Out_Status = 3;
                else sc8726.Out_Status = 1;
            }else {
                sc8726.Out_Status = 0;
            }
        }
        else if(status)
        {
            sc8726.Out_Status = status;
        }
        break;
    case I2C_TIME_STATUS:
        break;
    case I2C_ERROR_STATUS:
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
