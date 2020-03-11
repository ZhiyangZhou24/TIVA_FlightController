#include "time.h"
#include "include.h"
#include "parameter.h"
#include "systick.h"
#include "scheduler_fc.h"

volatile uint32_t sysTickUptime = 0;

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

uint32_t Timestamp_1us=0;
void SysTick_Handler()
{
	sysTickUptime++;
	sys_time();
	
}
/*系统主时间戳初始化，滴答定时器溢出率：1000HZ*/
void  SysTick_Configuration(void)
{
	SysTickEnable();
	/*定时周期：1000HZ*/
	SysTickPeriodSet(SysCtlClockGet()/TICK_PER_SECOND);
	SysTickIntEnable();
	SysTickIntRegister(SysTick_Handler);
}

uint32_t GetSysTime_us(void) 
{
	register uint32_t ms;
	u32 value;
	ms = sysTickUptime;
	value = ms * TICK_US + (SysTickPeriodGet() - SysTickValueGet()) * TICK_US / SysTickPeriodGet();
	return value;
}

void Delay_us(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while ((GetSysTime_us() - now) < us);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
      Delay_us(1000);
}

int time_1h,time_1m,time_1s,time_1ms;

void sys_time()
{ 

  if(time_1ms < 999)
	{
    time_1ms++;

		Loop_check();
	}
	else
	{
		
    time_1ms =0;
	  if(time_1s<59)
	  {
      time_1s++;
		}
		else
		{
			time_1s = 0;
			if(time_1m<59)
			{
				time_1m++;
			}
			else
			{
				time_1m = 0;
				if(time_1h<23)
				{
					time_1h++;
				}
				else
				{
					time_1h = 0;
				}
			}
		}
	}
}

volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = (( Cycle_T[item][NOW] - Cycle_T[item][OLD] ));//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/



