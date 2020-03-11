#include "App_OpticalFlow.h"

/*
֡ͷ   �ֽ���    ��������(xy���ٶ�)    ��У�� ��������  ֡β
0xFE    0x04      ... ... ... ...        SUM    SQUAL   0xAA
*/
#define OPTICAL_FLOW_FRAME_SIZE 4 
static OpticalFlow flow_p;
static CameraFram PosCamera;
unsigned char OPTICAL_RxBuffer[32];
static float lpfVal = 0.15f;			/*��ͨϵ��*/
static float flowPixelCompX=0.f, flowPixelCompY=0.f;/*��ǲ���*/

static float flowDataDx=0.f,  flowDataDy=0.f;		/*2֮֡��λ�Ʊ仯������λm*/
static float flowDataOutX=0.f,flowDataOutY=0.f;	/*��������*/
static float lastOutX=0.f,    lastOutY=0.f;			/*��һ�εĲ�������*/


static uint8_t FlowDataRecv(void)
{
	uint8_t new_flag = 0;
	uint8_t tmp;
	unsigned char sum = 0;
	static uint8_t rcount = 0;
	static uint8_t rvaild = 0;
	
	/*OPTICAL_FLOW data recive*/
	static uint8_t state =0;
	
	tmp = UARTCharGet(UART1_BASE);;
	switch(state)
	{
	case 0:
		if(tmp==0xFE)
		{
			state=1;
			OPTICAL_RxBuffer[rcount ++]=tmp;
		}else state = 0;
	break;
		
	case 1:
		if(tmp==0x04)
		{
			state=2;
			OPTICAL_RxBuffer[rcount ++]=tmp;
		}else state = 0;
	break;
		
	case 2:
		OPTICAL_RxBuffer[rcount ++]=tmp;

		if(rcount==9)
		{
			state = 0;
			rcount = 0;
			sum = OPTICAL_RxBuffer[2] + OPTICAL_RxBuffer[3] + OPTICAL_RxBuffer[4] + OPTICAL_RxBuffer[5];
			/*����2֮֡������ر仯������ʵ�ʰ�װ������� (pitch:x)  (roll:y)*/
			if(sum == OPTICAL_RxBuffer[6]) //�Ƚ�֡ͷ֡β��һ�����ݽ������
			{
				/*��ȡ����������Ϣ*/
				PosCamera.qual = OPTICAL_RxBuffer[7];
				/*����ԭʼ���ݣ��ٶȣ���λ�����ص�*/
				PosCamera.flow_x = ((int16_t)(OPTICAL_RxBuffer[3]<<8)|OPTICAL_RxBuffer[2]);
				PosCamera.flow_y = ((int16_t)(OPTICAL_RxBuffer[5]<<8)|OPTICAL_RxBuffer[4]);
				/*������λ�ƣ���λ�����ص�*/
				PosCamera.flow_raw_x += PosCamera.flow_x;
				PosCamera.flow_raw_y += PosCamera.flow_y;
				/*�򵥵�ͨ�˲�,�õ��˲�֮���λ��*/		
				PosCamera.flow_raw_x_lpf += (PosCamera.flow_raw_x - PosCamera.flow_raw_x_lpf) * lpfVal;  
				PosCamera.flow_raw_y_lpf += (PosCamera.flow_raw_y - PosCamera.flow_raw_y_lpf) * lpfVal;
				
				/*�����¼���־*/
				new_flag = 1;
			}
			else
			{
				new_flag = 0;
			}	
		}
	break;
		
	default:
		state = 0;
		rcount = 0;
	break;
	}
}

void UART1_IntHandler(void)
{
	//�����ٽ���
	taskENTER_CRITICAL();
	//�жϱ�־�Ķ�ȡ����������
	unsigned long ulStatus;
	//����ȫ���ж�
	IntMasterDisable();
	ulStatus = UARTIntStatus(UART1_BASE, true);
	//���ж�
	UARTIntClear(UART1_BASE, ulStatus);
	//�����жϱ�־��������¼�
	if(ulStatus & UART_INT_RX)
	{
		FlowDataRecv();
	}
	//ʹ��ȫ���ж�
	IntMasterEnable();
	//�˳��ٽ���
	taskEXIT_CRITICAL();
}