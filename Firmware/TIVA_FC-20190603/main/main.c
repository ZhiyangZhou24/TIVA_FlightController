/**
  ******************************************************************************
  * @�ļ���     �� main.c
  * @����       �� SWUST Tom
  * @�汾       �� V1.0.0
  * @����       �� 2019��3��20��
  * @ժҪ       �� TIVA_FC
  ******************************************************************************/
/******ϵͳͷ�ļ�*******/
#include "include.h"
#include "my_timer.h"
#include "my_uart.h"
#include "time.h"
#include "my_i2c.h"
#include "ultrasonic.h"
#include "icm_20602.h"
#include "my_uart.h"
#include "scheduler_fc.h"
#include "ist8310.h"
#include "ms5611.h"
#include "nrf24l01.h"
#include "App_RadioLink.h"

/*********ϵͳ��ز���**********/
unsigned long  Systermcoreclock;
struct Status_Params Sys_Status;
/*******************************/

void SysHardware_Init(void)
{
	/**********************************��ϵͳ��Ƶ��80MHZ************************************/
	/****************************************12MHZ ����*************************************/
	SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	Systermcoreclock = SysCtlClockGet();
	/***************************************************************************************/
	
	UART0_Init(115200);      //���������ӿڣ�����������λ���������ݣ����ߵ���
	/*�򿪸������㵥Ԫ*/
	FPUEnable();             //��FPU
  FPULazyStackingEnable(); 
	SysTick_Configuration();//ϵͳʱ�����ʼ����������ʱ����
	LEDTaskInit();          //LED��ʼ��
	EEPROM_Init();          //EEPROM��ʼ��
	Ultrasonic_Init();      //�������ӿ�  USART3
	SSI0_Init();            //�ڲ�SPI�������߳�ʼ��==>>ICM20602
	ICM_20602_Init();       //����ICM20602
	
	IIC1_Init();            //�ڲ�IIC�������߳�ʼ��==>>MS5611&IST8310
	IST8310_Init();         //IST8310������ ���� ��ʼ��У׼����
	MS5611_Init();          //MS5611��ѹ�� ����
	RadioLink_Init();       //��ʼ������ģ�� ��ѡ��NRF24L01 ���� ��DR16���ջ�(DBUS protocol)
	Para_Init();            //������У׼�����Լ�PID������ȡ����ʼ��
	PWM_Init();             //���PWM����ӿڳ�ʼ��(����ע����˳��һ��Ҫ��PWM����Ŷ�Ӧ)
}

int main(void)
{
	/*ϵͳӲ����ʼ��*/
	SysHardware_Init();
	while(1)
	{
		Duty_Loop();
	}
}


