#include "MY_UART.h"
#include "stdio.h"
#include "data_transfer.h"

/*使用microLib的方法*/
int fputc(int ch, FILE *f)
{
	while(UARTBusy(UART0_BASE));
	UARTCharPut(UART0_BASE,(uint8_t) ch);
   return ch;
}
//无线数传接收函数，调用ANO_DT_Data_Receive_Prepare函数进行解包
void UART0_Handler(void)
{
	static unsigned char res;
	//获取中断标志 原始中断状态 屏蔽中断标志
	uint32_t flag = UARTIntStatus(UART0_BASE,1);
	//清除中断标志
	UARTIntClear(UART0_BASE,flag);
	if(flag&UART_INT_RX)
	{
		if(UARTCharsAvail(UART0_BASE))//接收到数据
		{
			res = UARTCharGet(UART0_BASE); //读出寄存器数据
			ANO_DT_Data_Receive_Prepare(res);
		}
	}
}

void UART0_Init(unsigned int Badurate)
{
	// Enable the GPIO Peripheral used by the UART.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	// Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	// Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	// Initialize the UART for console I/O.
	UARTConfigSetExpClk(UART0_BASE, 16000000, Badurate,(UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));
	//UARTStdioConfig(0, Badurate, 16000000);
	
	//禁用FIFO后接收1位就产生中断
  UARTFIFODisable(UART0_BASE); 
	//使能UART0中断
	IntEnable(INT_UART0);
	//使能UART0中断
	UARTIntEnable(UART0_BASE, UART_INT_RX);
	//UART中断地址注册
	UARTIntRegister(UART0_BASE, UART0_Handler);
	//使能全局中断
	IntMasterEnable();
}

void UARTStringPut(uint32_t ui32Base, unsigned char *p,unsigned char length)
{ 
	while(length--) 
	{ 
		while(UARTBusy(ui32Base));
		UARTCharPut(ui32Base,*p++);
	}
}

