/********************************** (C) COPYRIGHT *******************************
 * File Name          : PD_process.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file provides all the PD firmware functions.
 *********************************************************************************
 * Copyright (c) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include <string.h>
#include "PD_Process.h"
#include "my_SPI_Slave.h"
#include "my_PD_Device.h"
#include "my_I2C_Device.h"

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

__attribute__((aligned(4))) uint8_t PD_Rx_Buf[34]; /* PD receive buffer */
__attribute__((aligned(4))) uint8_t PD_Tx_Buf[34]; /* PD send buffer */

/******************************************************************************/
UINT8 PD_Ack_Buf[2]; /* PD-ACK buffer */

UINT8 Tmr_Ms_Cnt_Last;    /* System timer millisecond timing final value */
UINT8 Tmr_Ms_Dlt;         /* System timer millisecond timing this interval value */
PD_CONTROL PD_Ctl;        /* PD Control Related Structures */
UINT8 Adapter_SrcCap[30]; /* Contents of the SrcCap message for the adapter */

UINT8 PDO_Len;

/* SrcCap Table */
UINT8 SrcCap_5V3A_Tab[4] = {0X2C, 0X91, 0X01, 0X3E};
UINT8 SrcCap_5V2A_Tab[4] = {0XC8, 0X90, 0X01, 0X3E};
UINT8 SinkCap_5V1A_Tab[4] = {0X64, 0X90, 0X01, 0X36};

UINT8 SrcCap_Tab[12] = {0X2C, 0X91, 0X01, 0X3E,
                        0xDE, 0xD0, 0x02, 0X3E,
                        0xA7, 0xC0, 0x03, 0X3E};
UINT8 CC_Select;
UINT8 Vbus_Select;

uint8_t SrcCap_size = 4;

uint8_t switch_state_pd_flag = 0;
uint8_t switch_state_pd;

uint16_t Vbus_Current = 167;

uint8_t snk_vbus_type;
uint8_t src_vbus_type;
uint16_t snk_vbus_current; // 受电端 电流
uint16_t src_vbus_current; // 供电端 电流

uint8_t pd_power_error = 0;

uint16_t connect_time = 0;

uint8_t valve_connect = 0; // 阀门连接状态

uint8_t pd_valve_status = 0;
uint16_t pd_valve_time = 0;

#define CONNECT_TIMEOUT 3000
/* PD3.0 */
UINT8 SrcCap_Ext_Tab[28] =
    {
        0X18,
        0X80,
        0X63,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X01,
        0X00,
        0X00,
        0X00,
        0X07,
        0X03,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X03,
        0X00,
        0X12,
        0X00,
        0X00,
};

UINT8 Status_Ext_Tab[8] =
    {
        0X06,
        0X80,
        0X16,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
};

void my_PD_Valve_Receive(uint8_t *buff)     //PD接收数据处理函数
{
    if (buff[2] == 0xAA && buff[3] == 0x55)
    {
        if (buff[0] == 0x01) // 错误情况
        {
            switch (buff[1])
            {
            case 0: // 关阀超时  超时时间20S一次
                // my_Valve_Set_close_timeout(1);
                my_SPI_ControlData_on(3, 0);
                break;
            case 1: // 关阀HALL异常  关到位HALL触发并且开到位HALL也触发了
                break;
            case 2: // 开阀超时  超时时间20S一次
                // my_Valve_Set_open_timeout(1);
                my_SPI_ControlData_on(3, 1);
                break;
            case 3: // 开阀HALL异常  开到位HALL触发并且关到位HALL也触发了
                break;
            case 4: // 自检模式开阀门10S后，开到位HALL依然没有被触发
                // my_Valve_Set_check_error(1);
                my_SPI_ControlData_on(3, 2);
                break;
            case 5: // 自检模式开阀10S到位后，关阀3S之后，开到位HALL依然是触发状态
                // my_Valve_Set_check_error(1);
                my_SPI_ControlData_on(3, 2);
                break;
            case 6:
                my_SPI_ControlData_on(3, 2);
                break;
            }
        }
        else if (buff[0] == 0x02) // 开关到位HALL信号
        {
            my_spi_set_hall(buff[1]);
            // my_Valve_Set_Status((buff[1] & 0x02) >> 1,buff[1] & 0x01);
        }
        else if (buff[0] == 0x03) // 阀门自检成功
        {
            switch (buff[1])
            {
            case 0: // 阀门自检成功
                my_SPI_ControlData_off(2, 0);
                break;
            }
        }
    }
    else
    {
        printf("pd data error %d  %d\r\n", buff[2], buff[3]);
    }
}

void my_set_Vbus_Current(uint8_t data_H, uint8_t data_L)
{
    Vbus_Current = ((data_H & 0x01) << 1) + data_L;
    SrcCap_Tab[8] = data_L;
    SrcCap_Tab[9] &= 0xfc;
    SrcCap_Tab[9] |= (data_H & 0x01);

    if ((data_H >> 1) >= 3)
    {
        SrcCap_size = 12;
    }
    else
    {
        SrcCap_size = 4;
    }
}

void my_switch_state_pd(uint8_t flag)
{
    // SrcCap_Tab[8] = data_L;
    // SrcCap_Tab[9] &= 0xfc;
    // SrcCap_Tab[9] |= (data_H & 0x01);
    switch_state_pd_flag = 1;
    switch_state_pd = flag;
}

uint16_t get_PD_Vbus_Current(uint8_t i)
{
    if (i >= 0 && i <= 2)
    {
        return SrcCap_Tab[i * 4] + ((SrcCap_Tab[(i * 4) + 1] & 0x03) << 8);
    }
    return 300;
}

/*********************************************************************
 * @fn      USBPD_IRQHandler
 *
 * @brief   This function handles USBPD interrupt.
 *
 * @return  none
 */
void USBPD_IRQHandler(void)
{
    if (USBPD->STATUS & IF_RX_ACT)
    {
        USBPD->STATUS |= IF_RX_ACT;
        if ((USBPD->STATUS & MASK_PD_STAT) == PD_RX_SOP0)
        {
            if (USBPD->BMC_BYTE_CNT >= 6)
            {
                /* If GOODCRC, do not answer and ignore this reception */
                if ((USBPD->BMC_BYTE_CNT != 6) || ((PD_Rx_Buf[0] & 0x1F) != DEF_TYPE_GOODCRC))
                {
                    Delay_Us(30); /* Delay 30us, answer GoodCRC */
                    if (get_PD_Status() == 0)
                        PD_Ack_Buf[0] = 0x41;
                    else
                        PD_Ack_Buf[0] = 0x61;
                    PD_Ack_Buf[1] = (PD_Rx_Buf[1] & 0x0E) | PD_Ctl.Flag.Bit.Auto_Ack_PRRole;
                    USBPD->CONFIG |= IE_TX_END;
                    PD_Phy_SendPack(0, PD_Ack_Buf, 2, UPD_SOP0);
                }
            }
        }
    }
    if (USBPD->STATUS & IF_TX_END)
    {
        /* Packet send completion interrupt (GoodCRC send completion interrupt only) */
        USBPD->PORT_CC1 &= ~CC_LVE;
        USBPD->PORT_CC2 &= ~CC_LVE;

        /* Interrupts are turned off and can be turned on after the main function has finished processing the data */
        NVIC_DisableIRQ(USBPD_IRQn);
        PD_Ctl.Flag.Bit.Msg_Recvd = 1; /* Packet received flag */
        USBPD->STATUS |= IF_TX_END;
    }
    if (USBPD->STATUS & IF_RX_RESET)
    {
        USBPD->STATUS |= IF_RX_RESET;
        // PD_SINK_Init();
        PD_PHY_Reset();
        printf("IF_RX_RESET\r\n");
    }
}

/*********************************************************************
 * @fn      PD_Rx_Mode
 *
 * @brief   This function uses to enter reception mode.
 *
 * @return  none
 */
void PD_Rx_Mode(void)
{
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    USBPD->DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
    NVIC_EnableIRQ(USBPD_IRQn);
}

/*********************************************************************
 * @fn      PD_SRC_Init
 *
 * @brief   This function uses to initialize SRC mode.
 *
 * @return  none
 */
void PD_SRC_Init()
{
    PD_Ctl.Flag.Bit.PR_Role = 1;         /* SRC mode */
    PD_Ctl.Flag.Bit.Auto_Ack_PRRole = 1; /* Default auto-responder role is SRC */
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PU_330;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PU_330;
}

/*********************************************************************
 * @fn      PD_SINK_Init
 *
 * @brief   This function uses to initialize SNK mode.
 *
 * @return  none
 */
void PD_SINK_Init()
{
    PD_Ctl.Flag.Bit.PR_Role = 0;         /* SINK mode */
    PD_Ctl.Flag.Bit.Auto_Ack_PRRole = 0; /* Default auto-responder role is SINK */
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PD;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PD;
}

/*********************************************************************
 * @fn      PD_PHY_Reset
 *
 * @brief   This function uses to reset PD PHY.
 *
 * @return  none
 */
void PD_PHY_Reset(void)
{
    if (get_PD_Status() == 0)
    {
        PD_SINK_Init();
        PD_Ctl.Flag.Bit.Stop_Det_Chk = 0; /* PD disconnection detection is enabled by default */
        PD_Ctl.PD_State = STA_IDLE;       /* Set idle state */
        PD_Ctl.Flag.Bit.PD_Comm_Succ = 0;
    }
    else
    {
        PD_Ctl.Flag.Bit.Msg_Recvd = 0;
        PD_Ctl.Msg_ID = 0;
        PD_Ctl.Flag.Bit.PD_Version = 1;
        PD_Ctl.Det_Cnt = 0;
        PD_Ctl.Flag.Bit.Connected = 0;
        PD_Ctl.PD_Comm_Timer = 0;
        PD_Ctl.PD_BusIdle_Timer = 0;
        PD_Ctl.Mode_Try_Cnt = 0x80;
        PD_Ctl.Flag.Bit.PD_Role = 1;
        PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
        PD_Ctl.PD_State = STA_IDLE;
        PD_Ctl.Flag.Bit.PD_Comm_Succ = 0;
        PD_SRC_Init();
        PD_Rx_Mode();
    }
}

/*********************************************************************
 * @fn      PD_Init
 *
 * @brief   This function uses to initialize PD registers and states.
 *
 * @return  none
 */
void PD_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    //    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); /* Open PD I/O clock, AFIO clock and PD clock */
    //    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    //    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    /* Initialize all variables */
    memset(&PD_Ctl.PD_State, 0x00, sizeof(PD_CONTROL));
    Adapter_SrcCap[0] = 1;
    memcpy(&Adapter_SrcCap[1], SrcCap_5V3A_Tab, 4);
    PD_PHY_Reset();
    PD_Rx_Mode();
}

/*********************************************************************
 * @fn      PD_Detect
 *
 * @brief   This function uses to detect CC connection.
 *
 * @return  0:No connection; 1:CC1 connection; 2:CC2 connection
 */
UINT8 PD_Detect(void)
{
    UINT8 ret = 0;
    UINT8 cmp_cc1 = 0;
    UINT8 cmp_cc2 = 0;

    if (PD_Ctl.Flag.Bit.Connected) /*Detect disconnection*/
    {
        // USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
        // USBPD->PORT_CC1 |= CC_CMP_22;
        // Delay_Us(2);
        // if (USBPD->PORT_CC1 & PA_CC_AI)
        // {
        //     cmp_cc1 = bCC_CMP_22;
        // }
        // USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
        // USBPD->PORT_CC1 |= CC_CMP_66;

        // USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
        // USBPD->PORT_CC2 |= CC_CMP_22;
        // Delay_Us(2);
        // if (USBPD->PORT_CC2 & PA_CC_AI)
        // {
        //     cmp_cc2 = bCC_CMP_22;
        // }
        // USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
        // USBPD->PORT_CC2 |= CC_CMP_66;

        // if ((GPIOC->INDR & PIN_CC1) != (uint32_t)Bit_RESET)
        // {
        //     cmp_cc1 |= bCC_CMP_220;
        // }
        // if ((GPIOC->INDR & PIN_CC2) != (uint32_t)Bit_RESET)
        // {
        //     cmp_cc2 |= bCC_CMP_220;
        // }

        if (USBPD->PORT_CC1 & CC_PD)
        {
            if (CC_Select == 1)
            {
                USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
                USBPD->PORT_CC1 |= CC_CMP_22;
                Delay_Us(2);
                if (USBPD->PORT_CC1 & PA_CC_AI)
                {
                    cmp_cc1 = bCC_CMP_22;
                }
                USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
                USBPD->PORT_CC1 |= CC_CMP_66;
            }
            else if (CC_Select == 2)
            {
                USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
                USBPD->PORT_CC2 |= CC_CMP_22;
                Delay_Us(2);
                if (USBPD->PORT_CC2 & PA_CC_AI)
                {
                    cmp_cc2 = bCC_CMP_22;
                }
                USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
                USBPD->PORT_CC2 |= CC_CMP_66;
            }
            /* SRC sample code does not handle SNK */
            if (cmp_cc1 & bCC_CMP_22 == bCC_CMP_22)
            {
                ret = 1;
            }
            if (cmp_cc2 & bCC_CMP_22 == bCC_CMP_22)
            {
                if (ret)
                {
                    ret = 1; /* Huawei A to C cable has two pull-up resistors */
                }
                else
                {
                    ret = 2;
                }
            }
        }
        else
        {
            USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
            USBPD->PORT_CC1 |= CC_CMP_22;
            Delay_Us(2);
            if (USBPD->PORT_CC1 & PA_CC_AI)
            {
                cmp_cc1 = bCC_CMP_22;
            }
            USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
            USBPD->PORT_CC1 |= CC_CMP_66;

            USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
            USBPD->PORT_CC2 |= CC_CMP_22;
            Delay_Us(2);
            if (USBPD->PORT_CC2 & PA_CC_AI)
            {
                cmp_cc2 = bCC_CMP_22;
            }
            USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
            USBPD->PORT_CC2 |= CC_CMP_66;

            if ((GPIOC->INDR & PIN_CC1) != (uint32_t)Bit_RESET)
            {
                cmp_cc1 |= bCC_CMP_220;
            }
            if ((GPIOC->INDR & PIN_CC2) != (uint32_t)Bit_RESET)
            {
                cmp_cc2 |= bCC_CMP_220;
            }
            if (USBPD->CONFIG & CC_SEL)
            {
                if ((cmp_cc2 & bCC_CMP_220) == bCC_CMP_220)
                {
                    ret = 0;
                }
                else
                {
                    ret = 2;
                }
            }
            else
            {
                if ((cmp_cc1 & bCC_CMP_220) == bCC_CMP_220)
                {
                    ret = 0;
                }
                else
                {
                    ret = 1;
                }
            }
        }
    }
    else /*Detect insertion*/
    {
        USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
        USBPD->PORT_CC1 |= CC_CMP_22;
        Delay_Us(2);
        if (USBPD->PORT_CC1 & PA_CC_AI)
        {
            cmp_cc1 |= bCC_CMP_22;
        }
        USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
        USBPD->PORT_CC1 |= CC_CMP_66;
        Delay_Us(2);
        if (USBPD->PORT_CC1 & PA_CC_AI)
        {
            cmp_cc1 |= bCC_CMP_66;
        }
        if ((GPIOC->INDR & PIN_CC1) != (uint32_t)Bit_RESET)
        {
            cmp_cc1 |= bCC_CMP_220;
        }

        USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
        USBPD->PORT_CC2 |= CC_CMP_22;
        Delay_Us(2);
        if (USBPD->PORT_CC2 & PA_CC_AI)
        {
            cmp_cc2 |= bCC_CMP_22;
        }
        USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
        USBPD->PORT_CC2 |= CC_CMP_66;
        Delay_Us(2);
        if (USBPD->PORT_CC2 & PA_CC_AI)
        {
            cmp_cc2 |= bCC_CMP_66;
        }
        if ((GPIOC->INDR & PIN_CC2) != (uint32_t)Bit_RESET)
        {
            cmp_cc2 |= bCC_CMP_220;
        }

        if (USBPD->PORT_CC1 & CC_PD)
        {
            /* SRC sample code does not handle SNK */
            if (cmp_cc1 & bCC_CMP_22 == bCC_CMP_22)
            {
                ret = 1;
            }
            if (cmp_cc2 & bCC_CMP_22 == bCC_CMP_22)
            {
                if (ret)
                {
                    ret = 1; /* Huawei A to C cable has two pull-up resistors */
                }
                else
                {
                    ret = 2;
                }
            }
        }
        else
        {
            if ((((cmp_cc1 & bCC_CMP_66) == bCC_CMP_66) & ((cmp_cc1 & bCC_CMP_220) == 0x00)) == 1)
            {
                if ((((cmp_cc2 & bCC_CMP_22) == bCC_CMP_22) & ((cmp_cc2 & bCC_CMP_66) == 0x00)) == 1)
                {
                    ret = 1;
                }
                if ((cmp_cc2 & bCC_CMP_220) == bCC_CMP_220)
                {
                    ret = 1;
                }
            }
            if ((((cmp_cc2 & bCC_CMP_66) == bCC_CMP_66) & ((cmp_cc2 & bCC_CMP_220) == 0x00)) == 1)
            {
                if (ret)
                {
                    ret = 0;
                }
                else
                {
                    if ((((cmp_cc1 & bCC_CMP_22) == bCC_CMP_22) && ((cmp_cc1 & bCC_CMP_66) == 0x00)) == 1)
                    {
                        ret = 2;
                    }
                    if ((cmp_cc1 & bCC_CMP_220) == bCC_CMP_220)
                    {
                        ret = 2;
                    }
                }
            }
        }
    }
    return (ret);
}

/*********************************************************************
 * @fn      PD_Det_Proc
 *
 * @brief   This function uses to process the return value of PD_Detect.
 *
 * @return  none
 */
void PD_Det_Proc(void)
{
    UINT8 status;

    if (PD_Ctl.Flag.Bit.Connected)
    {
        /* PD is connected, detect its disconnection */

        /* According to the usage scenario of PD SNK, whether
         * it is removed or not should be determined by detecting
         * the Vbus voltage, this code only shows the detection
         * and the subsequent communication flow. */
        status = PD_Detect();
        if (status)
        {
            PD_Ctl.Det_Cnt = 0;
        }
        else
        {
            PD_Ctl.Det_Cnt++;
            if (PD_Ctl.Det_Cnt >= 5)
            {
                PD_Ctl.Det_Cnt = 0;
                PD_Ctl.Flag.Bit.Connected = 0;

                if (PD_Ctl.Flag.Bit.Stop_Det_Chk == 0)
                {
                    PD_Ctl.PD_State = STA_DISCONNECT;
                    my_spi_disconnect();
                    my_VBus_Countrol(0);
                    connect_time = 0;
                    if (switch_state_pd_flag == 1)
                    {
                        switch_state_pd_flag = 0;
                        if (switch_state_pd)
                        {
                            set_PD_STATUS_SRC();
                        }
                        else
                        {
                            set_PD_STATUS_SNK();
                        }
                    }
                }
            }
        }
    }
    else
    {
        /* PD disconnected, check connection */
        status = PD_Detect();
        /* Determine connection status */
        if (status == 0)
        {
            PD_Ctl.Det_Cnt = 0;
        }
        else
        {
            PD_Ctl.Det_Cnt++;
        }
        if (PD_Ctl.Det_Cnt >= 5)
        {
            PD_Ctl.Det_Cnt = 0;
            PD_Ctl.Flag.Bit.Connected = 1;
            // set_Connected_delay_time(20);
            if (PD_Ctl.Flag.Bit.Stop_Det_Chk == 0)
            {
                if (get_PD_Status() == 0)
                {
                    if ((USBPD->PORT_CC1 & CC_PD) || (USBPD->PORT_CC2 & CC_PD))
                    {
                        /* Select the corresponding PD channel */
                        if (status == 1)
                        {
                            USBPD->CONFIG &= ~CC_SEL;
                            CC_Select = 1;
                        }
                        else
                        {
                            USBPD->CONFIG |= CC_SEL;
                            CC_Select = 2;
                        }
                        PD_Ctl.PD_State = STA_SRC_CONNECT;
                        printf("CC%d SRC Connect %d\r\n", status, get_PD_Status());
                    }
                }
                else
                {
                    if (status == 1)
                    {
                        USBPD->CONFIG &= ~CC_SEL;
                    }
                    else
                    {
                        USBPD->CONFIG |= CC_SEL;
                    }
                    if ((USBPD->PORT_CC1 & CC_PD) || (USBPD->PORT_CC2 & CC_PD))
                    {
                        PD_Ctl.PD_State = STA_SRC_CONNECT;
                        printf("CC%d SRC Connect\r\n", status);
                    }
                    else
                    {
                        PD_Ctl.PD_State = STA_SINK_CONNECT;
                        printf("CC%d SINK Connect\r\n", status);
                        // my_VBus_Countrol(1);
                        if (connect_time >= CONNECT_TIMEOUT)
                        {
                            connect_time = CONNECT_TIMEOUT - 10;
                        }
                    }
                }

                PD_Ctl.PD_Comm_Timer = 0;
            }
        }
    }
}

/*********************************************************************
 * @fn      PD_Phy_SendPack
 *
 * @brief   This function uses to send PD data.
 *
 * @return  none
 */
void PD_Phy_SendPack(UINT8 mode, UINT8 *pbuf, UINT8 len, UINT8 sop)
{

    if ((USBPD->CONFIG & CC_SEL) == CC_SEL)
    {
        USBPD->PORT_CC2 |= CC_LVE;
    }
    else
    {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;

    USBPD->DMA = (UINT32)(UINT8 *)pbuf;

    USBPD->TX_SEL = sop;

    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;

    /* Determine if you need to wait for the send to complete */
    if (mode)
    {
        /* Wait for the send to complete, this will definitely complete, no need to do a timeout */
        while ((USBPD->STATUS & IF_TX_END) == 0)
            ;
        USBPD->STATUS |= IF_TX_END;
        if ((USBPD->CONFIG & CC_SEL) == CC_SEL)
        {
            USBPD->PORT_CC2 &= ~CC_LVE;
        }
        else
        {
            USBPD->PORT_CC1 &= ~CC_LVE;
        }

        /* Switch to receive ready to receive GoodCRC */
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= ~(PD_ALL_CLR);
        USBPD->CONTROL &= ~(PD_TX_EN);
        USBPD->DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
        USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
        USBPD->CONTROL |= BMC_START;
    }
}

/*********************************************************************
 * @fn      PD_Load_Header
 *
 * @brief   This function uses to load pd header packets.
 *
 * @return  none
 */
void PD_Load_Header(UINT8 ex, UINT8 msg_type)
{
    /* Message Header
       BIT15 - Extended;
       BIT[14:12] - Number of Data Objects
       BIT[11:9] - Message ID
       BIT8 - PortPower Role/Cable Plug  0: SINK; 1: SOURCE
       BIT[7:6] - Revision, 00: V1.0; 01: V2.0; 10: V3.0;
       BIT5 - Port Data Role, 0: UFP; 1: DFP
       BIT[4:0] - Message Type
    */
    PD_Tx_Buf[0] = msg_type;
    if (PD_Ctl.Flag.Bit.PD_Role)
    {
        PD_Tx_Buf[0] |= 0x20;
    }
    if (PD_Ctl.Flag.Bit.PD_Version)
    {
        /* PD3.0 */
        PD_Tx_Buf[0] |= 0x80;
    }
    else
    {
        /* PD2.0 */
        PD_Tx_Buf[0] |= 0x40;
    }

    PD_Tx_Buf[1] = PD_Ctl.Msg_ID & 0x0E;
    if (PD_Ctl.Flag.Bit.PR_Role)
    {
        PD_Tx_Buf[1] |= 0x01;
    }
    if (ex)
    {
        PD_Tx_Buf[1] |= 0x80;
    }
}

/*********************************************************************
 * @fn      PD_Send_Handle
 *
 * @brief   This function uses to handle sending transactions.
 *
 * @return  0:success; 1:fail
 */
UINT8 PD_Send_Handle(UINT8 *pbuf, UINT8 len)
{
    UINT8 pd_tx_trycnt;
    UINT8 cnt;

    if ((len % 4) != 0)
    {
        /* Send failed */
        return (DEF_PD_TX_FAIL);
    }
    if (len > 28)
    {
        /* Send failed */
        return (DEF_PD_TX_FAIL);
    }

    cnt = len >> 2;
    PD_Tx_Buf[1] |= (cnt << 4);
    for (cnt = 0; cnt != len; cnt++)
    {
        PD_Tx_Buf[2 + cnt] = pbuf[cnt];
    }

    pd_tx_trycnt = 4;
    while (--pd_tx_trycnt) /* Maximum 3 executions */
    {
        NVIC_DisableIRQ(USBPD_IRQn);
        PD_Phy_SendPack(0x01, PD_Tx_Buf, (len + 2), UPD_SOP0);

        /* Set receive timeout 750US */
        cnt = 250;
        while (--cnt)
        {
            if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT)
            {
                USBPD->STATUS |= IF_RX_ACT;
                if ((USBPD->BMC_BYTE_CNT == 6) && ((PD_Rx_Buf[0] & 0x1F) == DEF_TYPE_GOODCRC))
                {
                    PD_Ctl.Msg_ID += 2;
                    break;
                }
            }
            Delay_Us(3);
        }
        if (cnt != 0)
        {
            break;
        }
    }

    /* Switch to receive mode */
    PD_Rx_Mode();
    if (pd_tx_trycnt)
    {
        /* Send successful */
        return (DEF_PD_TX_OK);
    }
    else
    {
        /* Send failed */
        return (DEF_PD_TX_FAIL);
    }
}

/*********************************************************************
 * @fn      PDO_Request
 *
 * @brief   This function uses to Send the specified PDO.
 *
 * @return  none
 */
void PDO_Request(UINT8 pdo_index) // 受电端发送请求包
{
    UINT16 Current, Voltage;
    UINT8 status;
    if ((pdo_index > PDO_Len) || (pdo_index == 0))
    {
        while (1)
        {
            printf("pdo_index error!\r\n");
            Delay_Ms(500);
        }
    }
    else
    {
        memcpy(&PD_Rx_Buf[2], &Adapter_SrcCap[4 * (pdo_index - 1) + 1], 4);
        PD_PDO_Analyse(1, &PD_Rx_Buf[2], &Current, &Voltage);
        printf("Request:\r\nCurrent:%d mA\r\nVoltage:%d mV\r\n", Current, Voltage);

        PD_Load_Header(0x00, DEF_TYPE_REQUEST);

        PD_Rx_Buf[5] = 0x03;
        PD_Rx_Buf[5] |= pdo_index << 4;
        PD_Rx_Buf[3] = PD_Rx_Buf[3] & 0x03;
        PD_Rx_Buf[3] |= (PD_Rx_Buf[2] << 2);
        PD_Rx_Buf[4] = PD_Rx_Buf[3];
        PD_Rx_Buf[4] <<= 2;
        PD_Rx_Buf[4] = PD_Rx_Buf[4] & 0x0C;
        PD_Rx_Buf[4] |= (PD_Rx_Buf[2] >> 6);
    }
    status = PD_Send_Handle(&PD_Rx_Buf[2], 4);

    if (status == DEF_PD_TX_OK)
    {
        PD_Ctl.PD_State = STA_RX_ACCEPT_WAIT;
    }
    else
    {
        PD_Ctl.PD_State = STA_TX_SOFTRST;
    }
    PD_Ctl.PD_Comm_Timer = 0;
    PD_Ctl.Flag.Bit.PD_Comm_Succ = 1;
}

/*********************************************************************
 * @fn      PD_Save_Adapter_SrcCap
 *
 * @brief   This function uses to save the adapter SrcCap information.
 *
 * @return  none
 */
void PD_Save_Adapter_SrcCap(void)
{
    UINT8 i, len;

    /* Calculate the number of NDO's (Number of Data Objects) in the Message Header */
    len = ((PD_Rx_Buf[1] >> 4) & 0x07);

    /* Remove the PPS section */
    for (i = 0; i < len; i++)
    {
        if ((PD_Rx_Buf[2 + (i << 2) + 3] & 0xC0) == 0xC0)
        {
            break;
        }
    }

    PDO_Len = i;

    /* Modify SrcCap information */
    /* BIT[31:30] - Fixed Supply */
    /* BIT29 - Dual-Role Power */
    /* BIT28 - USB Suspend Power */
    /* BIT27 - Unconstrained Power */
    /* BIT26 - USB Communications */
    /* BIT25 - Dual-Role Data */
    /* BIT24 - Unchunked Extended Message Supported */
    /* BIT23 - EPR Mode Capable */
    /* BIT22 - Reserved,shall be set to zero */
    /* BIT[21:20] - Peak Current */
    /* BIT[19:10] - Voltage in 50mV units */
    /* BIT[9:0] - Maximum Current in 10mA units */
    PD_Rx_Buf[5] = 0x3E;

    /* Save the adapter's SrcCap information */
    PD_Rx_Buf[1] &= 0x8F;
    PD_Rx_Buf[1] |= i << 4;
    Adapter_SrcCap[0] = i;
    memcpy(&Adapter_SrcCap[1], &PD_Rx_Buf[2], (i << 2));
}

/*********************************************************************
 * @fn      PD_PDO_Analyse
 *
 * @brief   This function uses to analyse PDO's voltage and current.
 *
 * @return  none
 */
void PD_PDO_Analyse(UINT8 pdo_idx, UINT8 *srccap, UINT16 *current, UINT16 *voltage)
{
    UINT32 temp32;

    temp32 = srccap[((pdo_idx - 1) << 2) + 0] +
             ((UINT32)srccap[((pdo_idx - 1) << 2) + 1] << 8) +
             ((UINT32)srccap[((pdo_idx - 1) << 2) + 2] << 16);

    /* Calculation of current values */
    if (current != NULL)
    {
        *current = (temp32 & 0x000003FF) * 10;
    }

    /* Calculation of voltage values */
    if (voltage != NULL)
    {
        temp32 = temp32 >> 10;
        *voltage = (temp32 & 0x000003FF) * 50;
    }
}

/*********************************************************************
 * @fn      PD_Main_Proc
 *
 * @brief   This function uses to process PD status.
 *
 * @return  none
 */
void PD_Main_Proc()
{
    UINT8 status;
    UINT8 pd_header;
    UINT8 var;
    UINT16 Current, Voltage;

    /* Receive idle timer count */
    PD_Ctl.PD_BusIdle_Timer += Tmr_Ms_Dlt;

    /* Status analysis processing */
    switch (PD_Ctl.PD_State)
    {
    case STA_IDLE:
        if (connect_time < CONNECT_TIMEOUT)
        {
            connect_time += Tmr_Ms_Dlt;
        }
        break;
    case STA_DISCONNECT:
        /* Status: Disconnected */
        valve_connect = 0;
        printf("Disconnect\r\n");
        PD_PHY_Reset();
        break;

    case STA_SRC_CONNECT:
        /* Status: SRC access */
        /* If SRC_CAP is received within 1S, reset operation is performed */
        PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
        if (PD_Ctl.PD_Comm_Timer > 1999)
        {
            /* Retry on exception (abort after 5 attempts) */
            PD_Ctl.Err_Op_Cnt++;
            if (PD_Ctl.Err_Op_Cnt > 5)
            {
                PD_Ctl.Err_Op_Cnt = 0;
                PD_Ctl.PD_State = STA_IDLE;
            }
            else
            {
                PD_PHY_Reset();
            }
        }
        break;

    case STA_RX_ACCEPT_WAIT:
        /* Status: waiting to receive ACCEPT */
    case STA_RX_PS_RDY_WAIT:
        /* Status: waiting to receive PS_RDY */
        PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
        if (PD_Ctl.PD_Comm_Timer > 499)
        {
            PD_Ctl.Flag.Bit.Stop_Det_Chk = 0; /* Enable connection detection*/
            PD_Ctl.PD_State = STA_TX_SOFTRST;
            PD_Ctl.PD_Comm_Timer = 0;
        }
        break;

    case STA_RX_PS_RDY:
        /* Status: PS_RDY received */
        PD_Ctl.PD_State = STA_IDLE;
        if (PD_Ctl.PD_State == STA_RX_APD_PS_RDY_WAIT)
        {
            PD_Ctl.PD_State = STA_RX_APD_PS_RDY;
        }
        break;

    case STA_TX_SOFTRST:
        /* Status: send software reset */
        /* Send soft reset, if sent successfully, mode unchanged, count +1 for retry */
        PD_Load_Header(0x00, DEF_TYPE_SOFT_RESET);
        status = PD_Send_Handle(NULL, 0);
        if (status == DEF_PD_TX_OK)
        {
            /* current mode unchanged, jump to initial state of current mode, mode retry count, switch mode if exceeded */
            PD_Ctl.PD_State = STA_IDLE;
        }
        else
        {
            PD_Ctl.PD_State = STA_TX_HRST;
        }
        PD_Ctl.PD_Comm_Timer = 0;
        break;

    case STA_TX_HRST:
        /* Status: Sending a hardware reset */
        /* Sending a hard reset */
        PD_Ctl.Flag.Bit.Stop_Det_Chk = 1;
        PD_Phy_SendPack(0x01, NULL, 0, UPD_HARD_RESET); /* send HRST */
        PD_Rx_Mode();                                   /* switch to rx mode */
        PD_Ctl.PD_State = STA_IDLE;
        PD_Ctl.PD_Comm_Timer = 0;
        break;

    case STA_SINK_CONNECT:
        if (connect_time < CONNECT_TIMEOUT)
        {
            connect_time += Tmr_Ms_Dlt;
            if (connect_time >= CONNECT_TIMEOUT)
            {
                my_VBus_Countrol(1);
            }
        }
        else
        {
            PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;

            if (PD_Ctl.PD_Comm_Timer > 159)
            {
                PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
                PD_Ctl.PD_Comm_Timer = 0;
                PD_Ctl.PD_State = STA_TX_SRC_CAP;
            }
        }

        break;
    case STA_TX_SRC_CAP: // 供电端发送可以请求的电压电流
        PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;

        if (PD_Ctl.PD_Comm_Timer > 159)
        {
            PD_Load_Header(0x00, DEF_TYPE_SRC_CAP);
            if (SrcCap_size == 12)
            {
                status = PD_Send_Handle(SrcCap_Tab, 8);
            }
            else
            {
                status = PD_Send_Handle(SrcCap_Tab, 4);
            }

            if (status == DEF_PD_TX_OK)
            {
                PD_Ctl.PD_State = STA_RX_REQ_WAIT;
                printf("Send Source Cap Successfully\r\n");
            }
            PD_Ctl.PD_Comm_Timer = 0;
        }
        break;
    case STA_RX_REQ_WAIT:
        PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
        if (PD_Ctl.PD_Comm_Timer > 29)
        {
            PD_Ctl.PD_State = STA_TX_HRST;
        }
        break;
    case STA_TX_ACCEPT: // 供电端回应受电端的请求包 Accept
        PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
        if (PD_Ctl.PD_Comm_Timer > 2)
        {
            PD_Load_Header(0x00, DEF_TYPE_ACCEPT);
            status = PD_Send_Handle(NULL, 0);
            if (status == DEF_PD_TX_OK)
            {
                printf("Accept\r\n");
                PD_Ctl.PD_State = STA_TX_PS_RDY;
                PD_Ctl.PD_Comm_Timer = 0;
                my_VBus_Countrol(Vbus_Select); // 提供对应电压电流
            }
            else
            {
                PD_Ctl.PD_State = STA_TX_SOFTRST;
                PD_Ctl.PD_Comm_Timer = 0;
            }
        }
        break;
    case STA_TX_PS_RDY: // 供电端等待电压设置输出完成,并且发包
        PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
        if (get_SC8726_Vbus_Out_Status() == 0)
        {
            PD_Load_Header(0x00, DEF_TYPE_PS_RDY);
            status = PD_Send_Handle(NULL, 0);
            if (status == DEF_PD_TX_OK)
            {
                if (valve_connect)
                {
                    my_spi_SRC(src_vbus_type, src_vbus_current, 1);
                    pd_valve_status = 1;
                    pd_valve_time = 10;
                }
                else
                {
                    my_spi_SRC(src_vbus_type, src_vbus_current, 0);
                }

                printf("PS ready\r\n");
                PD_Ctl.PD_State = STA_IDLE;
                PD_Ctl.PD_Comm_Timer = 0;
            }
            else
            {
                PD_Ctl.PD_State = STA_TX_SOFTRST;
                PD_Ctl.PD_Comm_Timer = 0;
            }
        }

        if (PD_Ctl.PD_Comm_Timer > 5000)
        {
            // PD_Load_Header(0x00, DEF_TYPE_PS_RDY);
            // status = PD_Send_Handle(NULL, 0);
            // if (status == DEF_PD_TX_OK)
            // {
            printf("SC8726 Time Out  PD\r\n");
            PD_Ctl.PD_State = STA_IDLE;
            //     PD_Ctl.PD_Comm_Timer = 0;
            // }
            // else
            // {
            //     PD_Ctl.PD_State = STA_TX_SOFTRST;
            //     PD_Ctl.PD_Comm_Timer = 0;
            // }
        }
        break;

    default:
        break;
    }

    /* Receive message processing */
    if (PD_Ctl.Flag.Bit.Msg_Recvd)
    {
        /* Adapter communication idle timing */
        PD_Ctl.Adapter_Idle_Cnt = 0x00;
        pd_header = PD_Rx_Buf[0] & 0x1F;
        switch (pd_header)
        {
        case DEF_TYPE_SRC_CAP: // 受电端请求电压 （有12V就请求12V，没有就请求最大的电压（9V 5V））
            Delay_Ms(5);
            PD_Ctl.Flag.Bit.Stop_Det_Chk = 0; /* Enable PD disconnection detection */

            PD_Save_Adapter_SrcCap();

            /* Analysis of the voltage and current of each PDO group */
            for (var = 1; var <= PDO_Len; ++var)
            {
                PD_PDO_Analyse(var, &PD_Rx_Buf[2], &Current, &Voltage);
                printf("PDO:%d\r\nCurrent:%d mA\r\nVoltage:%d mV\r\n", var, Current, Voltage);
                //                printf("%d  %d  %d\r\n", PD_Rx_Buf[2 + (var - 1) * 4], PD_Rx_Buf[3 + (var - 1) * 4], PD_Rx_Buf[4 + (var - 1) * 4]);
            }
            printf("\r\n");
            /* Different PDO's for different voltages and currents */
            /* Default application for the first group of PDO, 5V */
            if (PDO_Len >= 3)
            {
                PDO_Request(3);

                snk_vbus_type = 3;
                snk_vbus_current = PD_Rx_Buf[10] + ((PD_Rx_Buf[11] & 0x03) << 8);
            }
            else
            {
                PDO_Request(PDO_Len);

                snk_vbus_type = PDO_Len;
                snk_vbus_current = PD_Rx_Buf[2 + (PDO_Len - 1) * 4] + ((PD_Rx_Buf[3 + (PDO_Len - 1) * 4] & 0x01) << 8);
            }
            break;

        case DEF_TYPE_ACCEPT:
            PD_Ctl.PD_Comm_Timer = 0;
            if (PD_Ctl.PD_State == STA_RX_ACCEPT_WAIT || get_PD_Status() == 0)
            {
                PD_Ctl.PD_State = STA_RX_PS_RDY_WAIT;
            }
            break;
        case DEF_TYPE_REQUEST: // 供电端 收到受电端的请求包
            /* Request is received */
            printf("Handle Request  %d\r\n", PD_Rx_Buf[6]);
            Delay_Ms(2);
            PD_Ctl.ReqPDO_Idx = (PD_Rx_Buf[5] & 0x70) >> 4;
            printf("  Request:\r\n  PDO_Idx:%d\r\n", PD_Ctl.ReqPDO_Idx);
            if ((PD_Ctl.ReqPDO_Idx == 0) || (PD_Ctl.ReqPDO_Idx > 7))
            {
                PD_Ctl.PD_State = STA_TX_HRST;
            }
            else
            {
                
                if (PD_Rx_Buf[6] == 0x55 && PD_Rx_Buf[7] == 0xAA && PD_Rx_Buf[8] == 0x55 && PD_Rx_Buf[9] == 0xAA)
                {
                    if (SrcCap_size == 12)
                    {
                        pd_power_error = 0;
                        Vbus_Select = 3;
                    }
                    else
                    {
                        pd_power_error = 1;
                        Vbus_Select = PD_Ctl.ReqPDO_Idx;
                    }
                    valve_connect = 1;
                }else {
                    Vbus_Select = PD_Ctl.ReqPDO_Idx;
                }
                src_vbus_type = Vbus_Select;
                PD_PDO_Analyse(Vbus_Select, &PD_Rx_Buf[2], &Current, &Voltage);
                src_vbus_current = Current / 10;
                printf("  Current:%d mA\r\n", Current);

                // printf("DEF_TYPE_REQUEST %d  %d\r\n",)
                if ((PD_Rx_Buf[0] & 0xC0) == 0x80)
                {
                    /* PD3.0 */
                    PD_Ctl.Flag.Bit.PD_Version = 1;
                }
                else
                {
                    PD_Ctl.Flag.Bit.PD_Version = 0;
                }

                PD_Ctl.PD_State = STA_TX_ACCEPT; // 跳转到供电端回包Accept，并提供对应电压电流
                PD_Ctl.PD_Comm_Timer = 0;
            }
            break;

        case DEF_TYPE_PS_RDY:
            /* PS_RDY is received */
            my_spi_SNK(snk_vbus_type, snk_vbus_current);
            printf("Success\r\n");
            // PD_Load_Header(0x00, DEF_TYPE_TEST);
            // PD_Send_Handle(NULL, 0);
            PD_Ctl.PD_State = STA_RX_PS_RDY;
            break;

        case DEF_TYPE_WAIT:
            /* WAIT received, many requests may receive WAIT, need specific analysis */
            break;

        case DEF_TYPE_GET_SNK_CAP:
            Delay_Ms(1);
            PD_Load_Header(0x00, DEF_TYPE_SNK_CAP);
            PD_Send_Handle(SinkCap_5V1A_Tab, sizeof(SinkCap_5V1A_Tab));
            break;

        case DEF_TYPE_SOFT_RESET:
            Delay_Ms(1);
            PD_Load_Header(0x00, DEF_TYPE_ACCEPT);
            PD_Send_Handle(NULL, 0);
            break;

        case DEF_TYPE_GET_SRC_CAP_EX:
            Delay_Ms(1);
            PD_Load_Header(0x01, DEF_TYPE_SRC_CAP);
            PD_Send_Handle(SrcCap_Ext_Tab, sizeof(SrcCap_Ext_Tab));
            break;

        case DEF_TYPE_GET_STATUS:
            Delay_Ms(1);
            PD_Load_Header(0x01, DEF_TYPE_GET_STATUS_R);
            PD_Send_Handle(Status_Ext_Tab, sizeof(Status_Ext_Tab));
            break;

        case DEF_TYPE_VCONN_SWAP:
            Delay_Ms(1);
            PD_Load_Header(0x00, DEF_TYPE_REJECT);
            PD_Send_Handle(NULL, 0);
            break;

        case DEF_TYPE_VENDOR_DEFINED:
            /* VDM message handling */
            if ((PD_Rx_Buf[2] & 0xC0) == 0)
            {
                /* REQ */
                Delay_Ms(1);

                /* Data to be sent is cached to PD_Tx_Buf */
                PD_Load_Header(0x00, DEF_TYPE_VENDOR_DEFINED);

                /* Return to NAK */
                if ((PD_Rx_Buf[3] & 0x60) == 0)
                {
                    PD_Ctl.Flag.Bit.VDM_Version = 0;
                }
                else
                {
                    PD_Ctl.Flag.Bit.VDM_Version = 1;
                }
                PD_Rx_Buf[2] |= 0x80;
                PD_Send_Handle(&PD_Rx_Buf[2], 4);
            }
            break;

        case DEF_TYPE_TEST:
            Delay_Ms(1);
            my_PD_Valve_Receive(&PD_Rx_Buf[2]);
            // printf("DEF_TYPE_TEST  %02x  %02x  %02x  %02x\r\n", PD_Rx_Buf[2], PD_Rx_Buf[3], PD_Rx_Buf[4], PD_Rx_Buf[5]);
            break;

        default:
            printf("Unsupported Command\r\n");
            break;
        }

        /* Message has been processed, interrupt reception is turned on again */
        PD_Rx_Mode();
        PD_Ctl.Flag.Bit.Msg_Recvd = 0; /* Clear the received flag */
        PD_Ctl.PD_BusIdle_Timer = 0;   /* Idle time cleared */
    }
}


uint8_t pd_valve_send_buff[16];

void my_pd_connect_valve() // PD 通讯测试
{
    switch (pd_valve_status)
    {
    case 0:
        // if (valve_connect == 1)
        // {
        //     printf("PD Valve Connect\r\n");
        //     pd_valve_status = 1;
        //     pd_valve_time = 2000;
        // }
        break;
    case 1:
        if (my_time_tick(&pd_valve_time))
        {
            printf("PD Valve Test Send\r\n");
            pd_valve_status = 2;
            pd_valve_send_buff[0] = 0x01;
            pd_valve_send_buff[1] = 0x00;
            pd_valve_send_buff[2] = 0x00;
            pd_valve_send_buff[3] = 0x00;

            PD_Load_Header(0x00, DEF_TYPE_TEST);
            PD_Send_Handle(pd_valve_send_buff, 4);
            // my_Valve_Connect_PD(0);
            // my_Valve_Set_power_error(pd_power_error);
        }
        break;
    case 2:
        if (valve_connect == 0)
        {
            pd_valve_status = 0;
            // my_Valve_Disconnect();
        }
        break;
    }
}

void my_pd_close_valve()
{
    pd_valve_send_buff[0] = 0x03;
    pd_valve_send_buff[1] = 0x00;
    pd_valve_send_buff[2] = 0x00;
    pd_valve_send_buff[3] = 0x00;

    PD_Load_Header(0x00, DEF_TYPE_TEST);
    PD_Send_Handle(pd_valve_send_buff, 4);
}
void my_pd_open_valve()
{
    pd_valve_send_buff[0] = 0x02;
    pd_valve_send_buff[1] = 0x00;
    pd_valve_send_buff[2] = 0x00;
    pd_valve_send_buff[3] = 0x00;

    PD_Load_Header(0x00, DEF_TYPE_TEST);
    PD_Send_Handle(pd_valve_send_buff, 4);
}
void my_pd_check_valve()
{
    pd_valve_send_buff[0] = 0x04;
    pd_valve_send_buff[1] = 0x00;
    pd_valve_send_buff[2] = 0x00;
    pd_valve_send_buff[3] = 0x00;

    PD_Load_Header(0x00, DEF_TYPE_TEST);
    PD_Send_Handle(pd_valve_send_buff, 4);
}
