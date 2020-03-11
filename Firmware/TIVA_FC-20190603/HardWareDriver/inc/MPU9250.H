#ifndef _MPU9250_H_
#define _MPU9250_H_
#include "include.h"
#include "FC_Struct.h"
#include "math.h"


#define	  SCK(n)   GPIO_PORTD_DATA_R = (n)?(GPIO_PORTD_DATA_R|GPIO_PIN_0):(GPIO_PORTD_DATA_R&(~GPIO_PIN_0))
#define	  MOSI(n)	 GPIO_PORTD_DATA_R = (n)?(GPIO_PORTD_DATA_R|GPIO_PIN_1):(GPIO_PORTD_DATA_R&(~GPIO_PIN_1))
#define	  MISO   	 GPIO_PORTD_DATA_R & GPIO_PIN_2
#define	  CSN(n)	 GPIO_PORTD_DATA_R = (n)?(GPIO_PORTD_DATA_R|GPIO_PIN_3):(GPIO_PORTD_DATA_R&(~GPIO_PIN_3))

void Delay(unsigned int i);
void Delay_8MHz(void);
void Delay_1MHz(void);
bool MPU9250_Init(void);
void Delay_6MHz(void);
unsigned char SPI_RW(unsigned char uchar);
unsigned char SPI_RW_Fast(unsigned char uchar);
unsigned char SPI_ReadReg_Fast(unsigned char reg);
unsigned char SPI_RW_Mid(unsigned char uchar);
unsigned char SSP0_ReadReg(unsigned char reg);
void SSP0_WriteReg(unsigned char reg, unsigned char value);
void SSP0_ReadMulti(unsigned char reg, unsigned char *pBuf, unsigned char uchars);
void AK8963_WriteReg(unsigned char reg, unsigned char value);
void AK8963_Auto(void);
void Get_Motion6(void);
void Get_Motion9(void);

#ifdef __cplusplus
}
#endif

#endif
