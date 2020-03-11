#include "MY_TIMER.h"
#include "timer.h"
#include <NRF24L01.h>
//1MS时间戳，DBUS接收专用
void Timer0A_Init(void)
{
	// Enable the peripherals used by this example.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	// Enable processor interrupts.
	IntMasterEnable();
	// Configure the two 32-bit periodic timers.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, Systermcoreclock/1000);  //1mS溢出
	// Setup the interrupts for the timer timeouts.
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	// Enable the timers.
	TimerEnable(TIMER0_BASE, TIMER_A);
}
void Timer1A_Init(void)
{
	// Enable the peripherals used by this example.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	// Enable processor interrupts.
	IntMasterEnable();
	// Configure the two 32-bit periodic timers.
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE, TIMER_A, Systermcoreclock/100000);  //10us溢出
	// Setup the interrupts for the timer timeouts.
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	// Enable the timers.
	TimerEnable(TIMER1_BASE, TIMER_A);
}
unsigned long TimeStamp_1ms = 0;
unsigned long long TimeStamp_10us = 0;
unsigned int Now_PackNum=0;
extern unsigned int Pack_Num;
extern unsigned int  magadd,magnum;
void TIMER0A_Handler(void)
{
	static bool sta=0;
	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterDisable();
	TimeStamp_1ms ++;
	if(TimeStamp_1ms%1000==0)
	{
		sta=!sta;
		/*RadioLink UpDate Rate caculate*/
		Now_PackNum = Pack_Num;
		Pack_Num = 0;
		/*Magmeter UpDate Rate caculate*/
		magnum = magadd;
		magadd = 0;
	}
	IntMasterEnable();
}
void TIMER1A_Handler(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterDisable();
	TimeStamp_10us ++;
	IntMasterEnable();
}

