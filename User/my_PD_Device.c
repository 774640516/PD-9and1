/*
 * my_PD_Device.c
 *
 *  Created on: Apr 17, 2024
 *      Author: GW
 */

#include "my_PD_Device.h"
#include "PD_Process.h"
#include "my_I2C_Device.h"
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

uint8_t PD_STATUS = 0;
UINT8 Tim_Ms_Cnt = 0x00;
extern uint8_t valve_status;
extern uint16_t valve_time;
void TIM1_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    //    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}
uint8_t get_PD_Status() // 0 SNK受电端   1 SRC供电端
{
    return PD_STATUS;
}
void my_PD_Status_init()
{
    if (get_PD_Status() == 0)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
        GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_SET);
    }
    else
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_RESET);
    }
}

void set_PD_STATUS_SRC()
{
    PD_STATUS = 1;
    my_PD_Status_init();
    PD_Init();
}
void set_PD_STATUS_SNK()
{
    PD_STATUS = 0;
    my_PD_Status_init();
    PD_Init();
}
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    my_PD_Status_init();
    //     GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
}

void my_VBus_Countrol(uint8_t Vbus)
{
    if (PD_STATUS)
    {
        if (Vbus == 0)
        {
            my_SC8726_Close_Vbus();
            printf("Vbus off\r\n");
        }
        else if (Vbus == 1)
        {
            my_SC8726_Set_Out_Vbus(5000, get_PD_Vbus_Current(0));
            printf("set 5V sc8726\r\n");
        }
        else if (Vbus == 2)
        {
            my_SC8726_Set_Out_Vbus(9000, get_PD_Vbus_Current(1));
        }
        else if (Vbus == 3)
        {
            my_SC8726_Set_Out_Vbus(12000, get_PD_Vbus_Current(2));
        }
    }
    else
    {
        printf("SNK VBus error\r\n");
    }
}

void my_TIM_Handle()
{
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
    Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}
void my_PD_Handle()
{
    PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
    if (PD_Ctl.Det_Timer > 4)
    {
        PD_Ctl.Det_Timer = 0;
        PD_Det_Proc();
    }
    PD_Main_Proc();
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        Tim_Ms_Cnt++;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
