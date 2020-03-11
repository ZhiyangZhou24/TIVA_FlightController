#include <include.h>
#include "mpu9250.h"
#define UART_TASK_PRIO          2        //�������ȼ�
#define UART_STK_SIZE           100      //�����ջ��С
unsigned char n=0;
static void UartDebugTask(void *pvParameters)
{
	TickType_t xLastWakeTime;         //���ھ�׼��ʱ�ı���
	while (1) 
	{
		xLastWakeTime = xTaskGetTickCount(); //��ȡ��ǰTick����,�Ը�����ʱ������ֵ
		//printf("\r\nCPU_OccupyRate: %4.2f",Sys_Status.Occupancy_Rate);
		vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/1);
	}
}

void UartDebugTaskInit(void)
{
	xTaskCreate(UartDebugTask,            //������
							"UartDebugTask", //�������� 
							UART_STK_SIZE,           // �����ջ��С 
							NULL,       
							UART_TASK_PRIO,
							NULL );
}
