#include "MY_UART.h"
#include "stdio.h"
#include "data_transfer.h"

/*ʹ��microLib�ķ���*/
int fputc(int ch, FILE *f)
{
	while(UARTBusy(UART0_BASE));
	UARTCharPut(UART0_BASE,(uint8_t) ch);
   return ch;
}
//�����������պ���������ANO_DT_Data_Receive_Prepare�������н��
void UART0_Handler(void)
{
	static unsigned char res;
	//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־
	uint32_t flag = UARTIntStatus(UART0_BASE,1);
	//����жϱ�־
	UARTIntClear(UART0_BASE,flag);
	if(flag&UART_INT_RX)
	{
		if(UARTCharsAvail(UART0_BASE))//���յ�����
		{
			res = UARTCharGet(UART0_BASE); //�����Ĵ�������
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
	
	//����FIFO�����1λ�Ͳ����ж�
  UARTFIFODisable(UART0_BASE); 
	//ʹ��UART0�ж�
	IntEnable(INT_UART0);
	//ʹ��UART0�ж�
	UARTIntEnable(UART0_BASE, UART_INT_RX);
	//UART�жϵ�ַע��
	UARTIntRegister(UART0_BASE, UART0_Handler);
	//ʹ��ȫ���ж�
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

