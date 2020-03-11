#include <include.h>
#include "mpu9250.h"
#define UART_TASK_PRIO          2        //任务优先级
#define UART_STK_SIZE           100      //任务堆栈大小
unsigned char n=0;
static void UartDebugTask(void *pvParameters)
{
	TickType_t xLastWakeTime;         //用于精准定时的变量
	while (1) 
	{
		xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值
		//printf("\r\nCPU_OccupyRate: %4.2f",Sys_Status.Occupancy_Rate);
		vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/1);
	}
}

void UartDebugTaskInit(void)
{
	xTaskCreate(UartDebugTask,            //任务函数
							"UartDebugTask", //任务名称 
							UART_STK_SIZE,           // 任务堆栈大小 
							NULL,       
							UART_TASK_PRIO,
							NULL );
}
