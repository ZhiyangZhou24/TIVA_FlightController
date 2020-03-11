/*
 * NRF24L01.c
 *
 *  Created on: 2018年6月27日
 *      Author: zhiyang zhou
 */
#include <NRF24L01.h>
#include "time.h"

unsigned char sta=0; //状态标识
#define RX_DR (sta&0x0040)>>6
#define TX_DS (sta&0x0020)>>5
#define MAX_RT (sta&0x0010)>>4
#define TX_FULL sta&0x0001
//*********************************************NRF24L01*************************************
unsigned char RF_channel=0x63;     //无线信道取值0-127     公共信道
unsigned char RF_channel_Tx=0x63;  //无线信道取值0-127
unsigned char RF_Speed=0x06;       //(发射模式下)无线速率 1Mbps-0x06   2Mbps-0x0E  接收模式要+1把LNA给打开
unsigned char TX_PLOAD_WIDTH=32;
unsigned char RX_PLOAD_WIDTH=32;
unsigned char TX_ADDRESS[5]= {0x19,0x97,0x07,0x28,0x01};    //NRF24L01发射地址  公共地址
unsigned char RX_ADDRESS[5]= {0x19,0x98,0x08,0x28,0x00};    //NRF24L01接收地址

//unsigned char RF_channel=0x66;     //无线信道取值0-127     公共信道
//unsigned char RF_channel_Tx=0x66;  //无线信道取值0-127
//unsigned char RF_Speed=0x06;       //(发射模式下)无线速率 1Mbps-0x06   2Mbps-0x0E  接收模式要+1把LNA给打开
//unsigned char TX_PLOAD_WIDTH=32;
//unsigned char RX_PLOAD_WIDTH=32;
//unsigned char TX_ADDRESS[5]= {0x19,0x97,0x07,0x28,0x01};    //NRF24L01发射地址  公共地址
//unsigned char RX_ADDRESS[5]= {0x19,0x97,0x11,0x01,0x00};    //NRF24L01接收地址

/* 下面五个地址公用四个字节的地址  */
unsigned char RX_ADDRESS_P1[5]={0x19,0x95,0x08,0x18,0x00}; //Data pipe 1地址,5字节有效地址,不过不能与Data pipe 0的高四字节相同
unsigned char RX_ADDRESS_P2[5]={0x19,0x95,0x08,0x18,0x11}; //Data pipe 2地址,与Data pipe 1共享高四字节地址
unsigned char RX_ADDRESS_P3[5]={0x19,0x95,0x08,0x18,0x22}; //Data pipe 3地址,与Data pipe 1共享高四字节地址
unsigned char RX_ADDRESS_P4[5]={0x19,0x95,0x08,0x18,0x33}; //Data pipe 4地址,与Data pipe 1共享高四字节地址
unsigned char RX_ADDRESS_P5[5]={0x19,0x95,0x08,0x18,0x44}; //Data pipe 5地址,与Data pipe 1共享高四字节地址


bool NRF_CRC=true;  //false=8bit CRC   true(1)6bit CRC
//***************************************NRF24L01寄存器指令*******************************************************
#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define READ_REG        0x00    // 读寄存器指令
#define WRITE_REG       0x20      // 写寄存器指令
#define RD_RX_PLOAD     0x61    // 读取接收数据指令
#define WR_TX_PLOAD     0xA0    // 写待发数据指令
#define FLUSH_TX        0xE1      // 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2    // 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3    // 定义重复装载数据指令
#define NOP             0xFF    // 保留

#define MAX_TX      0x10  //达到最大发送次数中断
#define TX_OK       0x20  //TX发送完成中断
#define RX_OK       0x40  //接收到数据中断

//*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG2         0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

static void delay_60ns(void)
{
//	__nop();__nop();__nop();__nop();__nop();
//	__nop();__nop();__nop();__nop();__nop();
//	__nop();__nop();__nop();__nop();__nop();
//	__nop();__nop();__nop();__nop();__nop();

}

unsigned char init_NRF24L01_TX(void)
{
    unsigned short i=0;
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
		GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);
		GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5);
		HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	  HWREG(GPIO_PORTC_BASE+GPIO_O_CR)   |= GPIO_PIN_3;
	  HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = 0x0;
	  GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_3);
	  HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	  HWREG(GPIO_PORTC_BASE+GPIO_O_CR)   |= GPIO_PIN_2;
	  HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK) = 0x0;
	  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_2);
  	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
	
	  TX_CE(0);    // chip enable
    TX_CSN(1);   // Spi  disable
    TX_SCK(0);

    Delay_ms(10);
		NRF24L01_Check();

    SPI_RW_Reg_TX(WRITE_REG + CONFIG2, 0x0f);                //清空设置寄存器    寄存器 0x00
    SPI_RW_Reg_TX(FLUSH_TX,0xff);                        //冲洗TX FIFO
    SPI_RW_Reg_TX(FLUSH_RX,0xff);                        //冲洗RX FIFO
    SPI_RW_Reg_TX(WRITE_REG + EN_AA, 0x01);                      //频道0应答允许           寄存器 0x01
    SPI_RW_Reg_TX(WRITE_REG + EN_RXADDR, 0x03);                  //通道0允许接收           寄存器 0x02
    SPI_RW_Reg_TX(WRITE_REG + SETUP_AW, 0x03);           //收发地址长度            寄存器 0x03
    SPI_RW_Reg_TX(WRITE_REG + SETUP_RETR, 0x00);                 //设置自动重发间隔时间:750us + 86us;自动重发次数:15次  寄存器 0x04
    SPI_RW_Reg_TX(WRITE_REG + RF_CH, RF_channel);        //工作频率          寄存器 0x05
    SPI_RW_Reg_TX(WRITE_REG + RF_SETUP, RF_Speed+1);     //设置发射速率功率  寄存器 0x06
    SPI_RW_Reg_TX(WRITE_REG + STATUS, 0x7E);             //复位STATUS寄存器以此清除MAX_RT中断标志

    SPI_Write_Buf_TX(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    //写本地地址
    SPI_RW_Reg_TX(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                //字节有效数据长度  寄存器 0x11
    SPI_Write_Buf_TX(WRITE_REG+RX_ADDR_P0,RX_ADDRESS, RX_ADR_WIDTH);    //设置RX_P0节点地址  写接收端地址  寄存器0x0A

    TX_CE(1);
    for(i=0;i<32;i++)
    {
        TxBuf[i]=0;
    }
    return 1;
		
}
//函数：unsigned int SPI_RW(unsigned int uchar)
//功能：NRF24L01的SPI写时序
unsigned char SPI_RW_TX(unsigned char uchar)
{
    unsigned char bit_ctr;
    for(bit_ctr=0;bit_ctr<8;bit_ctr++) // output 8-bit
    {
        if((uchar&0x80)==0x00){TX_MOSI(0);}
        else{TX_MOSI(1);}
        uchar=uchar<<1;
				delay_60ns();
        TX_SCK(1);
				delay_60ns();
        uchar=uchar|READ_MISO;
        TX_SCK(0);
    }
    return(uchar);
}
//函数：uchar SPI_Read(uchar reg)
//功能：NRF24L01的SPI时序
unsigned char SPI_Read_TX(unsigned char reg)
{
    unsigned char reg_val;
    TX_CSN(0);                // CSN low, initialize SPI communication...
    SPI_RW_TX(reg);            // Select register to read from..
    reg_val = SPI_RW_TX(0);    // ..then read registervalue
    TX_CSN(1);                // CSN high, terminate SPI communication
    return(reg_val);        // return register value
}
//功能：NRF24L01读写寄存器函数
unsigned char SPI_RW_Reg_TX(unsigned char reg, unsigned char value)
{
    unsigned char status;
    TX_CSN(0);                   // CSN low, init SPI transaction
    status = SPI_RW_TX(reg);      // select register
    SPI_RW_TX(value);             // ..and write value to it..
    TX_CSN(1);                   // CSN high again
    return(status);            // return nRF24L01 status uchar
}
//函数：unsigned int SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
//功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
unsigned char SPI_Read_Buf_TX(unsigned char reg, unsigned char *pBuf, unsigned char uchars)
{
    unsigned char status,uchar_ctr;
    TX_CSN(0);                          // Set CSN low, init SPI tranaction
    status = SPI_RW_TX(reg);            // Select register to write to and read status uchar
    for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)
        {pBuf[uchar_ctr] = SPI_RW_TX(0);}    //
    TX_CSN(1);
    return(status);                    // return nRF24L01 status uchar
}
//函数：unsigned int SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
//功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
unsigned char SPI_Write_Buf_TX(unsigned char reg, unsigned char *pBuf, unsigned char uchars)
{
    unsigned char status,uchar_ctr;
    TX_CSN(0);            //SPI使能
    status = SPI_RW_TX(reg);
    for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++)
    {SPI_RW_TX(pBuf[uchar_ctr]);}
    TX_CSN(1);           //关闭SPI
    return(status);    //
}
//设置成接收模式
void Set_RX_Mode(void)
{
    SPI_RW_Reg_TX(WRITE_REG+STATUS,0x70);       //清除所有中断标识
    SPI_RW_Reg_TX(FLUSH_RX,0xff);               //冲洗RX FIFO
    TX_CE(0);
//  SPI_RW_Reg_TX(WRITE_REG + RF_CH, RF_channel);
    SPI_RW_Reg_TX(WRITE_REG + CONFIG2, 0x7f);   //设置成接收模式
    TX_CE(1);
    Delay_ms(1);
}
//函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
//功能：数据读取后放如rx_buf接收缓冲区中
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
    unsigned char revale=0;
    sta=SPI_Read_TX(STATUS);    // 读取状态寄存其来判断数据接收状况
    if(RX_DR)               // 判断是否接收到数据
    {
        TX_CE(0);             //SPI使能
        SPI_Read_Buf_TX(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
        revale=1;           //读取数据完成标志
        SPI_RW_Reg_TX(FLUSH_RX,0xff);                        //冲洗RX FIFO
    }
    SPI_RW_Reg_TX(WRITE_REG+STATUS,sta);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
    TX_CE(1);
    return revale;
}
//检查发射是否已经完成并返回值  true:已发送成功  false:未发送成功
bool Check_TX(void)
{
    sta=SPI_Read_TX(STATUS);    // 读取状态寄存其来判断数据接收状况
    if(TX_DS)               // 判断是否接收到数据
    {
        return true;
    }
    else
    {
        return false;
    }
}
//NRF24L01自检,如果错误,则自行重启
void Check_Self(void)
{
    while(!NRF24L01_Check())
    {
        init_NRF24L01_TX();  //直接每秒钟复位一次NRF24L01
        Delay_ms(500);
    }

}

//检测24L01是否存在
//返回值:0，成功;1，失败
unsigned char NRF24L01_Check(void)
{
	static unsigned char buf[5]={0};
	unsigned char i;
	SPI_Write_Buf_TX(WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,5);//写入5个字节的地址.
	SPI_Read_Buf_TX(TX_ADDR,buf,5); //读出写入的地址
	for(i=0;i<5;i++)
	{
		if(buf[i]!=TX_ADDRESS[i]){break;}
	}
	if(i!=5)
		return 0;    //没检测24L01错误
	return 1;        //检测到24L01
}

//函数：void nRF24L01_TxPacket(unsigned char * tx_buf)
//功能：发送 tx_buf中数据
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	SPI_RW_Reg_TX(WRITE_REG+STATUS,0x70);//清除所有中断标识
	SPI_RW_Reg_TX(FLUSH_TX,0xff); //清空发送FIFO指令
	TX_CE(0);
	SPI_Write_Buf_TX(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	SPI_RW_Reg_TX(WRITE_REG + CONFIG2, 0x5E);    //16位CRC
	TX_CE(1);
	//CE高电平大于10us才能进入发射模式 130us后开始发射数据 37us后发送一字节
}

