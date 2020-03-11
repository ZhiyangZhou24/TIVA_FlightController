#ifndef _APP_RADIOLINK_H_
#define _APP_RADIOLINK_H_

#include "include.h"
#include "my_uart.h"
/*���ջ�����ѡ��*/
enum RF_Mode{
	NRF24L01 = 1,
	DJI_DR16 = 2
};
extern u16 DBUS_Link[6];
extern u16 NRF_Link[6];
void RadioLink_Init(void);
void NRF_Task(void);
#endif
