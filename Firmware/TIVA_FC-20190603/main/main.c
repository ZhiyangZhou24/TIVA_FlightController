/**
  ******************************************************************************
  * @文件名     ： main.c
  * @作者       ： SWUST Tom
  * @版本       ： V1.0.0
  * @日期       ： 2019年3月20日
  * @摘要       ： TIVA_FC
  ******************************************************************************/
/******系统头文件*******/
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

/*********系统相关参数**********/
unsigned long  Systermcoreclock;
struct Status_Params Sys_Status;
/*******************************/

void SysHardware_Init(void)
{
	/**********************************将系统倍频到80MHZ************************************/
	/****************************************12MHZ 晶振*************************************/
	SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	Systermcoreclock = SysCtlClockGet();
	/***************************************************************************************/
	
	UART0_Init(115200);      //无线数传接口，可用匿名上位机接收数据，在线调参
	/*打开浮点运算单元*/
	FPUEnable();             //打开FPU
  FPULazyStackingEnable(); 
	SysTick_Configuration();//系统时间戳初始化，包括延时函数
	LEDTaskInit();          //LED初始化
	EEPROM_Init();          //EEPROM初始化
	Ultrasonic_Init();      //超声波接口  USART3
	SSI0_Init();            //内部SPI高速总线初始化==>>ICM20602
	ICM_20602_Init();       //配置ICM20602
	
	IIC1_Init();            //内部IIC高速总线初始化==>>MS5611&IST8310
	IST8310_Init();         //IST8310磁力计 配置 初始化校准矩阵
	MS5611_Init();          //MS5611气压计 配置
	RadioLink_Init();       //初始化无线模块 可选择NRF24L01 或者 大疆DR16接收机(DBUS protocol)
	Para_Init();            //传感器校准参数以及PID参数读取并初始化
	PWM_Init();             //电机PWM输出接口初始化(这里注意电机顺序一定要与PWM输出脚对应)
}

int main(void)
{
	/*系统硬件初始化*/
	SysHardware_Init();
	while(1)
	{
		Duty_Loop();
	}
}


