#ifndef _MY_SPI_H_
#define _MY_SPI_H_
#include "include.h"

#define	  MPU_CS(n)	  GPIO_PORTA_DATA_R = (n)?(GPIO_PORTA_DATA_R|GPIO_PIN_3):(GPIO_PORTA_DATA_R&(~GPIO_PIN_3))  //PA3
#define	  BARO_CS(n)	GPIO_PORTF_DATA_R = (n)?(GPIO_PORTF_DATA_R|GPIO_PIN_3):(GPIO_PORTF_DATA_R&(~GPIO_PIN_3))  //PF3

void SSI0_Init(void);
void SSI1_Init(void);
unsigned char send_and_receive(unsigned char data);
unsigned char send_and_receive1(unsigned char data);
unsigned char bytewrite_MPU(unsigned char write_addr, unsigned char write_data);
bool byteread_MPU(unsigned char read_addr, unsigned char *read_data);
bool Multiread_MPU(unsigned char read_addr, unsigned char *read_buffer, unsigned short bytes_count);
unsigned char ByteRead_MPU(unsigned char Addr);

unsigned char bytewrite_BARO(unsigned char write_addr, unsigned char write_data);
bool byteread_BARO(unsigned char read_addr, unsigned char *read_data);
unsigned char ByteRead_BARO(unsigned char Addr);
bool Multiread_BARO(unsigned char read_addr, unsigned char *read_buffer, unsigned short bytes_count);
#endif
