/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *GPIO routine:
 *PA0 push-pull output.
 *
 ***Only PA0--PA15 and PC16--PC17 support input pull-down.
 */

#include "debug.h"
#include "PD_Process.h"
#include "my_I2C_Device.h"
#include "my_SPI_Slave.h"
#include "my_PD_Device.h"
#include "Device_GPIO_I2C.h"
void ADC_VBUS_CAP_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    ADC_DeInit(ADC1);

    ADC_CLKConfig(ADC1, ADC_CLK_Div6);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);
}

void my_rcc()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                               RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO |
                               RCC_AHBPeriph_USBPD | RCC_APB2Periph_TIM1 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_I2C1 | RCC_AHBPeriph_USBFS,
                           ENABLE);
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
uint16_t set_Vbus;

uint8_t valve_connect_status = 0;
uint8_t valve_status = 0;
uint16_t valve_time;

void my_usb_init(){
    valve_connect_status = 0;
    valve_status = 0;
}

void my_usb_HID_Host_Handle()
{
    switch(valve_status){
        case 0:
            if(valve_connect_status){
                valve_status = 1;
                valve_time = 100;
            }
            break;
        case 1:
            if(valve_time > Tmr_Ms_Dlt)valve_time -= Tmr_Ms_Dlt;
            else {
                valve_status = 2;
                my_usb_init_send(my_spi_get());
                printf("my_usb_init_send\r\n");
                valve_time = 500;
            }
            break;
        case 2:
            if(valve_time >= Tmr_Ms_Dlt){
                valve_time -= Tmr_Ms_Dlt;
                if(valve_time == 0){
                    valve_time = 500;
                    printf("usb connect timeout\r\n");
                }
            }
            break;
        case 3:
            break;
    }
}

uint8_t my_usb_HID_receive_check(uint8_t *buffer, uint8_t buffer_size){
    uint8_t check = 0;
    if (buffer_size != (buffer[5] + 7))
    {
        printf("buffer_size = %d  buffer[5] = %d  size error\r\n", buffer_size, buffer[5]);
        return 0;
    }
    if (buffer[1] != 0x55 || buffer[2] != 0xaa || buffer[3] != 0x01)
    {
        printf("%d  %d  %d Default data error\r\n", buffer[1], buffer[2], buffer[3]);
        return 0;
    }
    if (buffer[4] > 10)
    {
        printf("%d  command error\r\n", buffer[4]);
        return 0;
    }
    for (uint8_t i = 1; i < (buffer_size - 1); i++)
        check += buffer[i];
    if (buffer[buffer_size - 1] != check)
    {
        printf("%d  %d  check error\r\n", check, buffer[buffer_size - 1]);
        return 0;
    }
    printf("usb hid data check ok\r\n");
    return 1;
}

void IWDG_Feed_Init(u16 prer, u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prer);
    IWDG_SetReload(rlr);
    IWDG_ReloadCounter();
    IWDG_Enable();
}
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    my_rcc();
    USART_Printf_Init(921600);
    GPIO_Toggle_INIT();
    my_SPI_Selve_Init();
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    printf("V4.0.3\r\n");
    PD_Init();

    TIM1_Init(999, 48 - 1);

    ADC_VBUS_CAP_Init();
    Device_GPIO_I2C_Init(1);
    my_SC8726_Init();
    
    IWDG_Feed_Init(IWDG_Prescaler_128,4000);
    while (1)
    {
        IWDG_ReloadCounter();
        my_TIM_Handle();
        my_PD_Handle();
        my_SC8726_Handle();
        my_spi_handle();
    }
}
