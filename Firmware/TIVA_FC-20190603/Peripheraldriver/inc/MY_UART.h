#ifndef _MY_UART_H_
#define _MY_UART_H_
#include "include.h"


void UART0_Init(unsigned int Badurate);
void UARTStringPut(uint32_t ui32Base, unsigned char *p,unsigned char length);
#endif
