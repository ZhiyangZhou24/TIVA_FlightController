#include "App_OpticalFlow.h"

/*
帧头   字节数    光流数据(xy轴速度)    和校验 环境质量  帧尾
0xFE    0x04      ... ... ... ...        SUM    SQUAL   0xAA
*/
#define OPTICAL_FLOW_FRAME_SIZE 4 
static OpticalFlow flow_p;
static CameraFram PosCamera;
unsigned char OPTICAL_RxBuffer[32];
static float lpfVal = 0.15f;			/*低通系数*/
static float flowPixelCompX=0.f, flowPixelCompY=0.f;/*倾角补偿*/

static float flowDataDx=0.f,  flowDataDy=0.f;		/*2帧之间位移变化量，单位m*/
static float flowDataOutX=0.f,flowDataOutY=0.f;	/*补偿像素*/
static float lastOutX=0.f,    lastOutY=0.f;			/*上一次的补偿像素*/


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
			/*连续2帧之间的像素变化，根据实际安装方向调整 (pitch:x)  (roll:y)*/
			if(sum == OPTICAL_RxBuffer[6]) //比较帧头帧尾，一包数据接收完毕
			{
				/*读取光流质量信息*/
				PosCamera.qual = OPTICAL_RxBuffer[7];
				/*光流原始数据，速度，单位是像素点*/
				PosCamera.flow_x = ((int16_t)(OPTICAL_RxBuffer[3]<<8)|OPTICAL_RxBuffer[2]);
				PosCamera.flow_y = ((int16_t)(OPTICAL_RxBuffer[5]<<8)|OPTICAL_RxBuffer[4]);
				/*积分求位移，单位是像素点*/
				PosCamera.flow_raw_x += PosCamera.flow_x;
				PosCamera.flow_raw_y += PosCamera.flow_y;
				/*简单低通滤波,得到滤波之后的位移*/		
				PosCamera.flow_raw_x_lpf += (PosCamera.flow_raw_x - PosCamera.flow_raw_x_lpf) * lpfVal;  
				PosCamera.flow_raw_y_lpf += (PosCamera.flow_raw_y - PosCamera.flow_raw_y_lpf) * lpfVal;
				
				/*更新事件标志*/
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
	//进入临界区
	taskENTER_CRITICAL();
	//中断标志的读取、处理和清除
	unsigned long ulStatus;
	//禁用全局中断
	IntMasterDisable();
	ulStatus = UARTIntStatus(UART1_BASE, true);
	//清中断
	UARTIntClear(UART1_BASE, ulStatus);
	//根据中断标志处理相关事件
	if(ulStatus & UART_INT_RX)
	{
		FlowDataRecv();
	}
	//使能全局中断
	IntMasterEnable();
	//退出临界区
	taskEXIT_CRITICAL();
}