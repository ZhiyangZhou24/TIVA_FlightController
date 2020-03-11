/*
 * NRF24L01.c
 *
 *  Created on: 2018��6��27��
 *      Author: zhiyang zhou
 */
#include <NRF24L01.h>
#include "time.h"

unsigned char sta=0; //״̬��ʶ
#define RX_DR (sta&0x0040)>>6
#define TX_DS (sta&0x0020)>>5
#define MAX_RT (sta&0x0010)>>4
#define TX_FULL sta&0x0001
//*********************************************NRF24L01*************************************
unsigned char RF_channel=0x63;     //�����ŵ�ȡֵ0-127     �����ŵ�
unsigned char RF_channel_Tx=0x63;  //�����ŵ�ȡֵ0-127
unsigned char RF_Speed=0x06;       //(����ģʽ��)�������� 1Mbps-0x06   2Mbps-0x0E  ����ģʽҪ+1��LNA����
unsigned char TX_PLOAD_WIDTH=32;
unsigned char RX_PLOAD_WIDTH=32;
unsigned char TX_ADDRESS[5]= {0x19,0x97,0x07,0x28,0x01};    //NRF24L01�����ַ  ������ַ
unsigned char RX_ADDRESS[5]= {0x19,0x98,0x08,0x28,0x00};    //NRF24L01���յ�ַ

//unsigned char RF_channel=0x66;     //�����ŵ�ȡֵ0-127     �����ŵ�
//unsigned char RF_channel_Tx=0x66;  //�����ŵ�ȡֵ0-127
//unsigned char RF_Speed=0x06;       //(����ģʽ��)�������� 1Mbps-0x06   2Mbps-0x0E  ����ģʽҪ+1��LNA����
//unsigned char TX_PLOAD_WIDTH=32;
//unsigned char RX_PLOAD_WIDTH=32;
//unsigned char TX_ADDRESS[5]= {0x19,0x97,0x07,0x28,0x01};    //NRF24L01�����ַ  ������ַ
//unsigned char RX_ADDRESS[5]= {0x19,0x97,0x11,0x01,0x00};    //NRF24L01���յ�ַ

/* ���������ַ�����ĸ��ֽڵĵ�ַ  */
unsigned char RX_ADDRESS_P1[5]={0x19,0x95,0x08,0x18,0x00}; //Data pipe 1��ַ,5�ֽ���Ч��ַ,����������Data pipe 0�ĸ����ֽ���ͬ
unsigned char RX_ADDRESS_P2[5]={0x19,0x95,0x08,0x18,0x11}; //Data pipe 2��ַ,��Data pipe 1��������ֽڵ�ַ
unsigned char RX_ADDRESS_P3[5]={0x19,0x95,0x08,0x18,0x22}; //Data pipe 3��ַ,��Data pipe 1��������ֽڵ�ַ
unsigned char RX_ADDRESS_P4[5]={0x19,0x95,0x08,0x18,0x33}; //Data pipe 4��ַ,��Data pipe 1��������ֽڵ�ַ
unsigned char RX_ADDRESS_P5[5]={0x19,0x95,0x08,0x18,0x44}; //Data pipe 5��ַ,��Data pipe 1��������ֽڵ�ַ


bool NRF_CRC=true;  //false=8bit CRC   true(1)6bit CRC
//***************************************NRF24L01�Ĵ���ָ��*******************************************************
#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define READ_REG        0x00    // ���Ĵ���ָ��
#define WRITE_REG       0x20      // д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61    // ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0    // д��������ָ��
#define FLUSH_TX        0xE1      // ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2    // ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3    // �����ظ�װ������ָ��
#define NOP             0xFF    // ����

#define MAX_TX      0x10  //�ﵽ����ʹ����ж�
#define TX_OK       0x20  //TX��������ж�
#define RX_OK       0x40  //���յ������ж�

//*************************************SPI(nRF24L01)�Ĵ�����ַ****************************************************
#define CONFIG2         0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������

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

    SPI_RW_Reg_TX(WRITE_REG + CONFIG2, 0x0f);                //������üĴ���    �Ĵ��� 0x00
    SPI_RW_Reg_TX(FLUSH_TX,0xff);                        //��ϴTX FIFO
    SPI_RW_Reg_TX(FLUSH_RX,0xff);                        //��ϴRX FIFO
    SPI_RW_Reg_TX(WRITE_REG + EN_AA, 0x01);                      //Ƶ��0Ӧ������           �Ĵ��� 0x01
    SPI_RW_Reg_TX(WRITE_REG + EN_RXADDR, 0x03);                  //ͨ��0�������           �Ĵ��� 0x02
    SPI_RW_Reg_TX(WRITE_REG + SETUP_AW, 0x03);           //�շ���ַ����            �Ĵ��� 0x03
    SPI_RW_Reg_TX(WRITE_REG + SETUP_RETR, 0x00);                 //�����Զ��ط����ʱ��:750us + 86us;�Զ��ط�����:15��  �Ĵ��� 0x04
    SPI_RW_Reg_TX(WRITE_REG + RF_CH, RF_channel);        //����Ƶ��          �Ĵ��� 0x05
    SPI_RW_Reg_TX(WRITE_REG + RF_SETUP, RF_Speed+1);     //���÷������ʹ���  �Ĵ��� 0x06
    SPI_RW_Reg_TX(WRITE_REG + STATUS, 0x7E);             //��λSTATUS�Ĵ����Դ����MAX_RT�жϱ�־

    SPI_Write_Buf_TX(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    //д���ص�ַ
    SPI_RW_Reg_TX(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                //�ֽ���Ч���ݳ���  �Ĵ��� 0x11
    SPI_Write_Buf_TX(WRITE_REG+RX_ADDR_P0,RX_ADDRESS, RX_ADR_WIDTH);    //����RX_P0�ڵ��ַ  д���ն˵�ַ  �Ĵ���0x0A

    TX_CE(1);
    for(i=0;i<32;i++)
    {
        TxBuf[i]=0;
    }
    return 1;
		
}
//������unsigned int SPI_RW(unsigned int uchar)
//���ܣ�NRF24L01��SPIдʱ��
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
//������uchar SPI_Read(uchar reg)
//���ܣ�NRF24L01��SPIʱ��
unsigned char SPI_Read_TX(unsigned char reg)
{
    unsigned char reg_val;
    TX_CSN(0);                // CSN low, initialize SPI communication...
    SPI_RW_TX(reg);            // Select register to read from..
    reg_val = SPI_RW_TX(0);    // ..then read registervalue
    TX_CSN(1);                // CSN high, terminate SPI communication
    return(reg_val);        // return register value
}
//���ܣ�NRF24L01��д�Ĵ�������
unsigned char SPI_RW_Reg_TX(unsigned char reg, unsigned char value)
{
    unsigned char status;
    TX_CSN(0);                   // CSN low, init SPI transaction
    status = SPI_RW_TX(reg);      // select register
    SPI_RW_TX(value);             // ..and write value to it..
    TX_CSN(1);                   // CSN high again
    return(status);            // return nRF24L01 status uchar
}
//������unsigned int SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
//����: ���ڶ����ݣ�reg��Ϊ�Ĵ�����ַ��pBuf��Ϊ���������ݵ�ַ��uchars���������ݵĸ���
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
//������unsigned int SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
//����: ����д���ݣ�Ϊ�Ĵ�����ַ��pBuf��Ϊ��д�����ݵ�ַ��uchars��д�����ݵĸ���
unsigned char SPI_Write_Buf_TX(unsigned char reg, unsigned char *pBuf, unsigned char uchars)
{
    unsigned char status,uchar_ctr;
    TX_CSN(0);            //SPIʹ��
    status = SPI_RW_TX(reg);
    for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++)
    {SPI_RW_TX(pBuf[uchar_ctr]);}
    TX_CSN(1);           //�ر�SPI
    return(status);    //
}
//���óɽ���ģʽ
void Set_RX_Mode(void)
{
    SPI_RW_Reg_TX(WRITE_REG+STATUS,0x70);       //��������жϱ�ʶ
    SPI_RW_Reg_TX(FLUSH_RX,0xff);               //��ϴRX FIFO
    TX_CE(0);
//  SPI_RW_Reg_TX(WRITE_REG + RF_CH, RF_channel);
    SPI_RW_Reg_TX(WRITE_REG + CONFIG2, 0x7f);   //���óɽ���ģʽ
    TX_CE(1);
    Delay_ms(1);
}
//������unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
//���ܣ����ݶ�ȡ�����rx_buf���ջ�������
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
    unsigned char revale=0;
    sta=SPI_Read_TX(STATUS);    // ��ȡ״̬�Ĵ������ж����ݽ���״��
    if(RX_DR)               // �ж��Ƿ���յ�����
    {
        TX_CE(0);             //SPIʹ��
        SPI_Read_Buf_TX(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
        revale=1;           //��ȡ������ɱ�־
        SPI_RW_Reg_TX(FLUSH_RX,0xff);                        //��ϴRX FIFO
    }
    SPI_RW_Reg_TX(WRITE_REG+STATUS,sta);   //���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�־
    TX_CE(1);
    return revale;
}
//��鷢���Ƿ��Ѿ���ɲ�����ֵ  true:�ѷ��ͳɹ�  false:δ���ͳɹ�
bool Check_TX(void)
{
    sta=SPI_Read_TX(STATUS);    // ��ȡ״̬�Ĵ������ж����ݽ���״��
    if(TX_DS)               // �ж��Ƿ���յ�����
    {
        return true;
    }
    else
    {
        return false;
    }
}
//NRF24L01�Լ�,�������,����������
void Check_Self(void)
{
    while(!NRF24L01_Check())
    {
        init_NRF24L01_TX();  //ֱ��ÿ���Ӹ�λһ��NRF24L01
        Delay_ms(500);
    }

}

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��
unsigned char NRF24L01_Check(void)
{
	static unsigned char buf[5]={0};
	unsigned char i;
	SPI_Write_Buf_TX(WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,5);//д��5���ֽڵĵ�ַ.
	SPI_Read_Buf_TX(TX_ADDR,buf,5); //����д��ĵ�ַ
	for(i=0;i<5;i++)
	{
		if(buf[i]!=TX_ADDRESS[i]){break;}
	}
	if(i!=5)
		return 0;    //û���24L01����
	return 1;        //��⵽24L01
}

//������void nRF24L01_TxPacket(unsigned char * tx_buf)
//���ܣ����� tx_buf������
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	SPI_RW_Reg_TX(WRITE_REG+STATUS,0x70);//��������жϱ�ʶ
	SPI_RW_Reg_TX(FLUSH_TX,0xff); //��շ���FIFOָ��
	TX_CE(0);
	SPI_Write_Buf_TX(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	SPI_RW_Reg_TX(WRITE_REG + CONFIG2, 0x5E);    //16λCRC
	TX_CE(1);
	//CE�ߵ�ƽ����10us���ܽ��뷢��ģʽ 130us��ʼ�������� 37us����һ�ֽ�
}

