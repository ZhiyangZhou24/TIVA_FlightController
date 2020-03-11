//*****************************************************************************
#include "include.h"
#include "led_task.h"
#include "time.h"

// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
void LEDTaskInit(void)
{
  // Enable Peripheral Clocks 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  // Configure the GPIO Pin Mux for PB6
	// for GPIO_PB6
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);

  // Configure the GPIO Pin Mux for PB7
	// for GPIO_PB7

	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);

  // Configure the GPIO Pin Mux for PF4
	// for GPIO_PF4
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
	LED_R(0);LED_G(0);LED_B(0);
}

void LED_Display(unsigned char mode,unsigned char fly_ready)
{
	static unsigned int Now_Time = 0,Till_Time=0;;
	static bool LED_STA=0;
	Now_Time = sysTickUptime;
	if(fly_ready==0)
	{
		Till_Time=400;
	}
	else if(fly_ready==1)
	{
		Till_Time=100;
	}
	if(Now_Time%Till_Time==0)
	{
		LED_STA = !LED_STA;
	}
	if(mode ==0)
	{LED_G(LED_STA);LED_R(0);LED_B(0);}
	else if(mode ==1)
	{LED_B(LED_STA);LED_R(0);LED_G(0);}
	else if(mode ==2)
	{LED_R(LED_STA);LED_B(0);LED_G(0);}
	
}
