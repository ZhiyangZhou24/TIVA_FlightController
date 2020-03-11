#include "include.h"
#include "ultrasonic.h"
#include "my_uart.h"
#include "FC_Config.h"

/*超声波接收中断*/
void UART3_Handler(void)
{
  static unsigned char res;
	//获取中断标志 原始中断状态 屏蔽中断标志
	uint32_t flag = UARTIntStatus(UART3_BASE,1);
	//清除中断标志
	UARTIntClear(UART3_BASE,flag);
	if(flag&UART_INT_RX)
	{
		if(UARTCharsAvail(UART3_BASE))//接收到数据
		{
			 res = UARTCharGet(UART3_BASE); //读出寄存器数据
			 Ultra_Get(res);
		}
	}
}
static void UART3_Init(unsigned int Badurate)
{
	// Enable the GPIO Peripheral used by the UART.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	// Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	// Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART3_BASE, UART_CLOCK_PIOSC);
	UARTConfigSetExpClk(UART3_BASE, 16000000, Badurate,(UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));
	UARTEnable(UART3_BASE);
	
	//禁用FIFO后接收1位就产生中断
  UARTFIFODisable(UART3_BASE); 
	//使能UART0中断
	IntEnable(INT_UART3);
	//使能UART0中断
	UARTIntEnable(UART3_BASE, UART_INT_RX);
	//UART中断地址注册
	UARTIntRegister(UART3_BASE, UART3_Handler);
	//使能全局中断
	IntMasterEnable();
}


void Ultrasonic_Init()
{
  UART3_Init(9600);			//串口3初始化，函数参数为波特率
}

s8 ultra_start_f;
u8 ultra_time;
u8 ultra_ok = 0;
/*超声波触发测距*/
void Ultra_Duty()
{
	u8 temp[3];

	ultra_time++;
	ultra_time = ultra_time%2;
	
	if( ultra_time == 0 )  //100ms//改用发送中断，节省时间。
	{
/*//////////////////////////////////////////////
		UART5->DR = 0xe8;   //ks103地址（可设置）
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
		
		UART5->DR = 0x02;   //++++
		while( (UART5->SR & USART_FLAG_TXE) == 0 );

		UART5->DR = 0xbc;  //70ms,带温度补偿
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
//////////////////////////////////////////////*/	
	#if defined(USE_KS103)
		temp[0] = 0xe8;
		temp[1] = 0x02;
		temp[2] = 0xbc;
		UARTStringPut(UART3_BASE,temp ,3);
	#elif defined(USE_US100)
		temp[0] = 0x55;
		UARTStringPut(UART3_BASE,temp ,1);
	#endif
///////////////////////////////////////////////
		ultra_start_f = 1;
	}
}

u16 ultra_distance,ultra_distance_old;
s16 ultra_delta;
/*超声波接收函数，在串口接收中断内调用*/
void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		ultra_distance = (ultra_tmp<<8) + com_data;
		ultra_start_f = 0;
		ultra_ok = 1;
	}
	
	ultra_delta = ultra_distance - ultra_distance_old;
	
	ultra_distance_old = ultra_distance;
}

