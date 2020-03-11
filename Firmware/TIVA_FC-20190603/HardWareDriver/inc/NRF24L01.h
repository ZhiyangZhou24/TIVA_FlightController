/*
 * NRF24L01.h
 *
 *  Created on: 2018Äê6ÔÂ27ÈÕ
 *      Author: zhiyang zhou
 */

#ifndef MY_DRIVER_NRF24L01_H_
#define MY_DRIVER_NRF24L01_H_
#include <include.h>
#include <stdbool.h>
#define   TX_CE(n)    GPIO_PORTD_DATA_R = (n)?(GPIO_PORTD_DATA_R|GPIO_PIN_4):(GPIO_PORTD_DATA_R&(~GPIO_PIN_4)) //CE PD4
#define   TX_CSN(n)   GPIO_PORTD_DATA_R = (n)?(GPIO_PORTD_DATA_R|GPIO_PIN_5):(GPIO_PORTD_DATA_R&(~GPIO_PIN_5)) //CSN PD5
#define   TX_SCK(n)   GPIO_PORTB_DATA_R = (n)?(GPIO_PORTB_DATA_R|GPIO_PIN_1):(GPIO_PORTB_DATA_R&(~GPIO_PIN_1)) //SCK PB1
#define   TX_MOSI(n)  GPIO_PORTC_DATA_R = (n)?(GPIO_PORTC_DATA_R|GPIO_PIN_2):(GPIO_PORTC_DATA_R&(~GPIO_PIN_2)) //MOSI PC2
//#define   READ_MISO   (GPIO_PORTC_DATA_R )
#define   READ_MISO   GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_3)>>3;  //MISO PC3

extern unsigned char TxBuf[32];
extern unsigned char RxBuf[32];

void Check_Self(void);
unsigned char init_NRF24L01_TX(void);
unsigned char SPI_RW_TX(unsigned char uchar);
unsigned char SPI_Read_TX(unsigned char reg);
unsigned char SPI_RW_Reg_TX(unsigned char reg, unsigned char value);
unsigned char SPI_Read_Buf_TX(unsigned char reg, unsigned char *pBuf, unsigned char uchars);
unsigned char SPI_Write_Buf_TX(unsigned char reg, unsigned char *pBuf, unsigned char uchars);
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf);
void nRF24L01_TxPacket(unsigned char * tx_buf);
bool Check_TX(void);
void Set_RX_Mode(void);

unsigned char NRF24L01_Check(void);


#endif /* MY_DRIVER_NRF24L01_H_ */
