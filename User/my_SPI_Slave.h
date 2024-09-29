/*
 * my_SPI_Slave.h
 *
 *  Created on: Apr 7, 2024
 *      Author: GW
 */

#ifndef USER_MY_SPI_SLAVE_H_
#define USER_MY_SPI_SLAVE_H_

// typedef struct
// {
//     uint8_t Reserve;
//     uint8_t Device_Info;
//     uint8_t Control_Valve;
//     uint8_t Check_Valve;
//     uint8_t Error_Valve;
//     uint16_t Input_Power;
//     uint16_t Output_Power;
// }MY_PD_CONTROL;

extern UINT8  Tmr_Ms_Dlt;

typedef struct
{
    uint8_t Start;
    uint8_t Tx_TI_Init;

    uint8_t *Write_Buff;
    uint8_t Write_Len;
    uint8_t Write_Status;

    uint8_t *Read_Buff;
    uint8_t Read_Len;
    uint8_t Read_Status;

    uint8_t Control_Addr;
    uint8_t Control_Data[256];

    uint8_t DIO_Status;
    uint16_t DIO_Time;
}MY_SPI_DEVICE;

void my_SPI_Selve_Init();
void my_spi_SNK(uint8_t PDO_Len,uint16_t Current);
void my_spi_SRC(uint8_t PDO_Len, uint16_t Current, uint8_t status);
void my_spi_disconnect();
void my_spi_handle();

void my_SPI_ControlData_on(uint8_t addr,uint8_t bit);
void my_SPI_ControlData_off(uint8_t addr,uint8_t bit);

void my_spi_set_hall(uint8_t hall);

void my_spi_set_valve();
uint8_t my_spi_get();

uint8_t my_time_tick(uint16_t *data);
#endif /* USER_MY_SPI_SLAVE_H_ */
