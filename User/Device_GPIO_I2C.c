/*
 * Device_GPIO_I2C.c
 *
 *  Created on: Dec 3, 2024
 *      Author: GW
 */

#include "debug.h"
#include "Device_GPIO_I2C.h"

#define SC8726_RxAdderss 0xC5  // 0X62 << 1 + 1
#define SC8726_TxAdderss 0xC4  // 0X62 << 1

// PC18   SCL
// PC19   SDA

void Device_GPIO_I2C_Init(uint8_t status)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
//    GPIO_PinRemapConfig(GPIO_Remap_PIOC, ENABLE);
    // GPIO_Remap_PIOC

    if(status == 0)
    {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_19;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }else {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_19;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }

}

void Device_GPIO_I2C_SDA_INPUT()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_19;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void Device_GPIO_I2C_SDA_OUTPUT()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_19;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Device_I2C_W_SCL(uint8_t BitValue) // 拉低SCL或者释放SCL(拉高)
{
    GPIO_WriteBit(GPIOC, GPIO_Pin_18, (BitAction)BitValue);
    Delay_Us(10);
}

void Device_I2C_W_SDA(uint8_t BitValue) // 给SDA写数据（拉低SDA/释放SDA(拉高)）
{
    GPIO_WriteBit(GPIOC, GPIO_Pin_19, (BitAction)BitValue);
    Delay_Us(10);
}

void Device_I2C_Start(void) // 起始信号
{
    //	SCL(1);//释放SCL
    Device_I2C_W_SDA(1); // 释放SDA
    Device_I2C_W_SCL(1); // 释放SCL
    Device_I2C_W_SDA(0); // 拉低SDA
    Device_I2C_W_SCL(0); // 拉低SCL
}
void Device_I2C_Stop(void)
{
    Device_I2C_W_SCL(0);
    Device_I2C_W_SDA(0);
    Device_I2C_W_SCL(1);
    Device_I2C_W_SDA(1);
}

void Device_I2C_SendByte(uint8_t Byte) // 主机向从机发送一个字节，高位先行
{
    for (uint8_t i = 0; i < 8; i++)
    {
        Device_I2C_W_SDA(Byte & (0x80 >> i)); // 给SDA写入数据,只要Byte不是0，那么写入的就是1。因为BitAction
        Device_I2C_W_SCL(1);                  // 释放SCL，从机读取数据
        Device_I2C_W_SCL(0);                  // 拉低SCL，主机准备给SDA写入字节的次高位数据
    }
}
uint8_t Device_I2C_R_SDA(void) // 主机读SDA数据，即判断引脚的电平
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_19);
    Delay_Us(10);
    return BitValue;
}
uint8_t Device_I2C_ReceiveByte(void)
{
    uint8_t Byte = 0x00; // 接收数据的变量
    Device_I2C_W_SDA(1); // 主机释放SDA，让控制权给从机
    Device_GPIO_I2C_SDA_INPUT();
    for (uint8_t i = 0; i < 8; i++)
    {
        Device_I2C_W_SCL(1); // 主机拉高SCL，准备开始读取SDA上面的数据
        if (Device_I2C_R_SDA() == 1)
        {
            Byte |= (0x80 >> i);
        }
        Device_I2C_W_SCL(0); // 主机拉低SCL，准备让从机给SDA写入数据
    }
    Device_GPIO_I2C_SDA_OUTPUT();
    return Byte;
}

uint8_t Device_I2C_ReceiveACK(void) // 主机接收应答
{
    /*
        主机发送完一个字节后，SCL为低电平，
        等待从机给SDA上面写入数据，如果拉低则代表接收成功
    */
    uint8_t ACKBit;
    Device_I2C_W_SDA(1); // 主机释放SDA，让控制权给从机
    Device_I2C_W_SCL(1); // 主机释放SCL，准备开始读取SDA上面的数据
    ACKBit = Device_I2C_R_SDA();
    Device_I2C_W_SCL(0); // 主机拉低SCL，进入下一个时序单元，主机准备给SDA写入字节的数据
    return ACKBit;
}

void Device_I2C_SendACK(uint8_t ACKBit) // 主机发送应答
{
    /*
        从机发送完一个字节后，SCL为低电平，
        等待主机给SDA上面写数据，如果拉低代表接收成功。
    */
    Device_I2C_W_SDA(ACKBit); // 主机给SDA写入应答信号，0为应答，1为非应答
    Device_I2C_W_SCL(1);      // 主机拉高SCL，让从机读取应答信号
    Device_I2C_W_SCL(0);      // 主机拉低SCL，准备写入数据
}

// 指定地址读寄存器
uint8_t Device_I2C_ReadReg(uint8_t reg_addr)
{
    uint8_t data;
    Device_I2C_Start();             // 起始位
    Device_I2C_SendByte(SC8726_TxAdderss); // 从机地址+读
    Device_I2C_ReceiveACK();        // 接收应答，这里有应答位可以判断从机有没有接收到数据

    // 寻址找到从机之后就可以发送下一个字节了
    Device_I2C_SendByte(reg_addr); // 用来指定我写的地址，这个地址就写到从机的地址指针中了
    // 从机接收到reg_addr之后，它的寄存器指针就指向了0x19这个位置
    Device_I2C_ReceiveACK(); // 接收应答

    Device_I2C_Start();              // 再来一个起始条件
    Device_I2C_SendByte(SC8726_RxAdderss);  // 再次指定设备的ID号，这次我们读
    Device_I2C_ReceiveACK();         // 主机接收应答
    data = Device_I2C_ReceiveByte(); // 指针指向的寄存器的数据
    Device_I2C_SendACK(1);           // 主机接收后，发送应答
    Device_I2C_Stop();               // 结束

    return data;
}
// 指定地址写寄存器
void Device_I2C_WriteReg(uint8_t reg_addr, uint8_t data)
{
    Device_I2C_Start();             // 起始位
    Device_I2C_SendByte(SC8726_TxAdderss); // 从机地址+写
    Device_I2C_ReceiveACK();        // 接收应答，这里有应答位可以判断从机有没有接收到数据

    Device_I2C_SendByte(reg_addr); // //用来指定我写的寄存器地址
    Device_I2C_ReceiveACK();       // 接收应答
    Device_I2C_SendByte(data);     // 写入的数据
    Device_I2C_ReceiveACK();       // 接收应答
    Device_I2C_Stop();             // 结束
}
