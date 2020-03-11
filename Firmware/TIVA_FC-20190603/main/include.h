#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "tm4c123gh6pge.h"

#include "hw_gpio.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "interrupt.h"
#include "pin_map.h"
#include "rom.h"
#include "rom_map.h"
#include "sysctl.h"
#include "systick.h"
#include "ssi.h"

#include "uart.h"
#include "uartstdio.h"
#include "eeprom.h"


#include "led_task.h"

#include "MY_UART.h"
#include "MY_TIMER.h"
#include "MY_SPI.h"
#include "MY_PWM.h"
#include "MY_EEPROM.h"

#include "imu.h"

#include "parameter.h"
#include "FC_Config.h"
struct Status_Params
{
	unsigned long long CPU_ADD; //����ͳ��CPUռ����
	float Occupancy_Rate; 			//CPUռ����
	int Touch_X;
	int Touch_Y;
	
	float USB_Capacity;//USB����������ʣ������
	float Rx_Capacity;//���յ������ݻ�����
	float Bat_Voltage;
};
extern struct Status_Params Sys_Status;

extern unsigned long  Systermcoreclock;

#endif
