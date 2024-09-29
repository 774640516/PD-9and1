/*
 * my_SPI_Slave.c
 *
 *  Created on: Apr 7, 2024
 *      Author: GW
 */
#include "debug.h"
#include "my_SPI_Slave.h"
#include "my_PD_Device.h"
#include "PD_Process.h"
#include "my_I2C_Device.h"
#define CONTROL_DATA_LEN 8
// #define DEVICE_INFO   0
// #define
#define CONTROL_DEVICE_SIZE 6

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SPI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

uint8_t test_number[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#define CONTROL_DEVICE_WRITE_SIZE 4
const uint8_t control_write_addr[CONTROL_DEVICE_WRITE_SIZE] = {
    2, 3, 4, 6};

volatile MY_SPI_DEVICE my_spi;

extern PD_CONTROL PD_Ctl; /* PD Control Related Structures */

uint8_t send_spi_slave_data = 0xff;
uint8_t receive_data[10];
uint8_t receive_addr = 0;

uint8_t receive_buff[10];
uint8_t receive_buff_size = 0;
uint8_t receive_flag = 0;
void my_SPI_Data_Init()
{
    my_spi.Start = 0;
    my_spi.Tx_TI_Init = 0;
    my_spi.Write_Len = 0;
    my_spi.Read_Len = 0;
    my_spi.Write_Status = 0;
    my_spi.Read_Status = 0;
    my_spi.Control_Addr = 0;
    for (uint8_t i = 0; i < CONTROL_DATA_LEN; i++)
    {
        my_spi.Control_Data[i] = 0;
    }

    my_spi.DIO_Status = 0;
    my_spi.DIO_Time = 0;
}
void my_SPI_Device_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef SPI_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
    //        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
    SPI_I2S_SendData(SPI1, 0xff);

    SPI_Cmd(SPI1, ENABLE);

    //    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void my_SPI_Selve_Init()
{
    my_SPI_Data_Init();
    my_SPI_Device_Init();
}

void my_SPI_ControlData_on(uint8_t addr, uint8_t bit)
{
    my_spi.Control_Data[addr] |= (1 << bit);
}
void my_SPI_ControlData_off(uint8_t addr, uint8_t bit)
{
    my_spi.Control_Data[addr] &= ~(1 << bit);
}
void SPI1_IRQHandler(void)
{
    if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)
    {
        switch (SPI_I2S_ReceiveData(SPI1))
        {
        case 1:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[0]);
            break;
        case 2:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[1]);
            break;
        case 3:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[2]);
            break;
        case 4:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[3]);
            break;
        case 5:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[4]);
            send_spi_slave_data = my_spi.Control_Data[5];
            break;
        case 6:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[6]);
            send_spi_slave_data = my_spi.Control_Data[7];
            break;
        case 7:
            SPI_I2S_SendData(SPI1, my_spi.Control_Data[8]);
            send_spi_slave_data = my_spi.Control_Data[3];
            break;
        default:
            send_spi_slave_data = 0xff;
            break;
        }

        receive_data[receive_addr++] = SPI_I2S_ReceiveData(SPI1);
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
        SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
    }
    if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
    {
        SPI_I2S_SendData(SPI1, send_spi_slave_data);
        SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);
    }
    SPI_Cmd(SPI1, ENABLE);
}

void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4); /* Clear Flag */
        send_spi_slave_data = 0xff;
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
        for (receive_buff_size = 0; receive_buff_size < receive_addr; receive_buff_size++)
        {
            receive_buff[receive_buff_size] = receive_data[receive_buff_size];
            //            printf("%02x  ", receive_data[receive_buff_size]);
        }
        //        printf("\r\n");
        receive_flag = 1;
        receive_addr = 0;
            //    printf("12312312312\r\n");
    }
}

void my_spi_SNK(uint8_t PDO_Len, uint16_t Current) //
{
    if (Current >= 300)
        Current = 300;
    my_spi.Control_Data[0] = 0x10 + 1;
    my_spi.Control_Data[4] = (PDO_Len << 1) + (Current >> 8);
    my_spi.Control_Data[5] = (uint8_t)Current & 0xff;

    my_spi.DIO_Status = 1;
    my_spi.DIO_Time = 50;

    printf("my_spi.Control_Data[0] = %d\r\n", my_spi.Control_Data[0]);
}

void my_spi_SRC(uint8_t PDO_Len, uint16_t Current, uint8_t status) //
{
    if (status == 0)
    {
        my_spi.Control_Data[0] = 0x30 + 2;
    }
    else
    {
        if (PDO_Len < 3)
            my_SPI_ControlData_on(3, 3);
        my_spi.Control_Data[0] = 0x20 + 4;
    }
    printf("my_spi.Control_Data[0] = %d\r\n", my_spi.Control_Data[0]);
    my_spi.Control_Data[4] = (PDO_Len << 1) + (Current >> 8);
    my_spi.Control_Data[5] = (uint8_t)Current & 0xff;

    my_spi.Control_Data[8] = 0xfc;

    my_spi.DIO_Status = 1;
    my_spi.DIO_Time = 50;
}

void my_spi_set_hall(uint8_t hall)
{
    my_spi.Control_Data[8] = 0xfc | hall;
}

void my_spi_disconnect()
{
    my_spi.Control_Data[0] = 0;
    my_spi.Control_Data[1] = 0;
    my_spi.Control_Data[2] = 0;
    my_spi.Control_Data[3] = 0;
    my_spi.Control_Data[8] = 0;
    my_spi.DIO_Status = 0;
    my_spi.DIO_Time = 5;
}

void my_spi_set_valve()
{
    if (my_spi.Control_Data[2])
    {
        my_usb_send(1, 2);
    }
    else
    {
        my_usb_send(my_spi.Control_Data[1], 0);
    }
}

uint8_t my_spi_get()
{
    if (my_spi.Control_Data[2])
    {
        return 2;
    }
    else
    {
        return my_spi.Control_Data[1];
    }
}

uint8_t my_time_tick(uint16_t *data)
{
    if (*data > Tmr_Ms_Dlt)
        *data -= Tmr_Ms_Dlt;
    else
        *data = 0;
    if (*data == 0)
    {
        return 1;
    }

    return 0;
}

void my_spi_handle()
{
    if (my_spi.DIO_Time)
    {
        if (my_spi.DIO_Time >= Tmr_Ms_Dlt)
        {
            my_spi.DIO_Time -= Tmr_Ms_Dlt;
            if (my_spi.DIO_Time == 0)
            {
                printf("set DIO_Status = %d\r\n", my_spi.DIO_Status);
                GPIO_WriteBit(GPIOB, GPIO_Pin_0, my_spi.DIO_Status);
            }
        }
        else
        {
            my_spi.DIO_Time = 0;
            printf("set DIO_Status = %d\r\n", my_spi.DIO_Status);
            GPIO_WriteBit(GPIOB, GPIO_Pin_0, my_spi.DIO_Status);
        }
    }

    if (receive_flag == 1)
    {
        // receive_flag = 0;
        // for (uint8_t i = 0; i < receive_buff_size; i++)
        // {
        //     printf("%02x  ", receive_buff[i]);
        // }
        // printf("size = %d\r\n",receive_buff_size);
        if (receive_buff_size)
        {
            switch (receive_buff[0])
            {
            case 0x82:
                if (receive_buff_size == 2)
                {
                    printf("valve set %d\r\n", receive_buff[1]);
                    my_SPI_ControlData_off(3, receive_buff[1]);
                    my_spi.Control_Data[1] = receive_buff[1];
                    my_spi.Control_Data[2] = 0;
                    // my_usb_send(receive_buff[1], 0);
                    if(receive_buff[1]){
                        my_pd_open_valve();
                    }else {
                        my_pd_close_valve();
                    }
                }else {
                    printf("receive_buff_size = %d error\r\n",receive_buff_size);
                }

                break;
            case 0x83:
                if (receive_buff_size == 2 && my_spi.Control_Data[1] == 1)
                {
                    my_spi.Control_Data[2] = receive_buff[1];
                    my_SPI_ControlData_off(3, 2);
                    printf("valve check start  %d\r\n", receive_buff[1]);
                    // my_usb_send(1, 1);
                    my_pd_check_valve();
                }
                break;
            case 0x84:
                if (receive_buff_size == 2)
                {
                    my_spi.Control_Data[3] = receive_buff[1];
                }
                break;
            case 0x86:
                if (receive_buff_size == 3)
                {
                    printf("set_PD_STATUS_SRC\r\n");
                    my_set_Vbus_Current(receive_buff[1], receive_buff[2]);
                    if (PD_Ctl.Flag.Bit.Connected == 0)
                    {
                        set_PD_STATUS_SRC();
//                        set_sc8726_select();
                    }else {
                        my_switch_state_pd(1);
                    }
                }
                break;
            default:
                break;
            }
        }
        receive_flag = 0;
        for (uint8_t i = 0; i < receive_buff_size; i++)
        {
            printf("%02x  ", receive_buff[i]);
        }
        printf("size = %d\r\n",receive_buff_size);

    }
}
