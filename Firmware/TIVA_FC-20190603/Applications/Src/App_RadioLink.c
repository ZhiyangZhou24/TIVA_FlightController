#include "App_RadioLink.h"
#include "my_timer.h"
#include "NRF24L01.H"
#include "rc.h"
/*            RF接收机选择
*
* NRF24L01 ===>> 板载2.4G 24l01无线收发芯片
* DJI_DR16 ===>> 外接大疆DR16接收机
*
*/
unsigned char RF_Reciver = DJI_DR16;

/****************************************/
/*NRF24L01 发送接收缓冲区*/
unsigned char TxBuf[32];
unsigned char RxBuf[32];
/*DBUS 接收机各通道数据*/
u16 DBUS_Link[6];
u16 NRF_Link[6];
unsigned char USART_RX_BUF[25]; //串口接收缓冲区
unsigned int Pack_Num=0;    //1s时间内收包数
unsigned char Sync_Time=0;  //同步时间，大疆接收机同步时间计算
/************DBUS协议解析*************/
void UART1_Handler(void)
{
  static unsigned char res;
	static uint16_t Data_num=0;
	static uint64_t USART1_timestamp=0;
	static uint64_t DBUS_timestamp=0;
	
	//获取中断标志 原始中断状态 屏蔽中断标志
	uint32_t flag = UARTIntStatus(UART1_BASE,1);
	//清除中断标志
	UARTIntClear(UART1_BASE,flag);
	if(flag&UART_INT_RX)
	{
		if(UARTCharsAvail(UART1_BASE))//接收到数据
		{
			 res = UARTCharGet(UART1_BASE); //读出寄存器数据
			 /* 获取时间戳 判断丢步以及重新连入接收机  实际大疆接收机14ms同步一次数据   这里7ms同步一次数据*/
			 if((TimeStamp_1ms-USART1_timestamp)>7ull)
			 {
				 Data_num=0;
			 }
			 /* 接收数据 */
			 USART_RX_BUF[Data_num++]=res;

			 if(Data_num==17)  //遥控器数据接收完成
			 {
				Sync_Time = TimeStamp_1ms - DBUS_timestamp;
				Pack_Num++;
				//横滚
				DBUS_Link[0]=(((unsigned short)USART_RX_BUF[1]&0x07)<<8)+(unsigned short)USART_RX_BUF[0];
				//俯仰
				DBUS_Link[1]=(((unsigned short)USART_RX_BUF[2]&0x3F)<<5)+(unsigned short)(USART_RX_BUF[1]>>3);
				//偏航
				DBUS_Link[3]=(((unsigned short)USART_RX_BUF[4]&0x01)<<10)+((unsigned short)USART_RX_BUF[3]<<2)+((unsigned short)USART_RX_BUF[2]>>6);
				 //油门
				DBUS_Link[2]=(((unsigned short)USART_RX_BUF[5]&0x0F)<<7)+((unsigned short)USART_RX_BUF[4]>>1);
				//左拨杆 1 3 2
				DBUS_Link[4]=((unsigned short)USART_RX_BUF[5]>>6);
				//右拨杆 1 3 2
				DBUS_Link[5]=(((unsigned short)USART_RX_BUF[5]&0x30)>>4);
				
				Data_num=0;
				DBUS_timestamp=TimeStamp_1ms;
			 }
			 USART1_timestamp=TimeStamp_1ms;
		}
	}
}

//串口1初始化  DR16接收机 采用DBUS 协议
void RadioLink_Init(void)
{
	if(RF_Reciver == DJI_DR16)
	{
		//初始化串口1外设
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		GPIOPinConfigure(GPIO_PB0_U1RX);
		GPIOPinConfigure(GPIO_PB1_U1TX);
		GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		UARTConfigSetExpClk(UART1_BASE,Systermcoreclock, 100000, UART_CONFIG_WLEN_8 |// 100Kbps  默认无流控
									UART_CONFIG_PAR_EVEN | UART_CONFIG_STOP_ONE);  //偶校验  一停止位
		//禁用FIFO后接收1位就产生中断
		UARTFIFODisable(UART1_BASE); 
		//使能UART0中断
		IntEnable(INT_UART1);
		//使能UART0中断
		UARTIntEnable(UART1_BASE, UART_INT_RX);
		//UART中断地址注册
		UARTIntRegister(UART1_BASE, UART1_Handler);
		for(unsigned char i=0;i<CH_NUM;i++)
		{
			MAX_CH[i] = 1684;
			MIN_CH[i] = 364;
		}
	}
	else 
	{
		for(unsigned char i=0;i<CH_NUM;i++)
		{
			MAX_CH[i] = 1000;
			MIN_CH[i] = 0;
		}
		
		if(init_NRF24L01_TX())
		{
			Set_RX_Mode();
		}
	}
	//使能全局中断
	IntMasterEnable();
	//打开时间戳
	Timer0A_Init();
}
unsigned char Check_num,i;
/*解析NRF24L01 数据函数 所用协议为MFELink协议 用户亦可自行定义协议*/
void NRF_Task(void)
{
	static u16 RF_Temp[6];
	if(nRF24L01_RxPacket(RxBuf))
	{
		Check_num=0;                //校验开始
		for(i=0;i<31;i++){Check_num+=RxBuf[i];}
		if(Check_num == RxBuf[31])  //校验通过
		{
			Pack_Num++;               //收报数暂存变量
		}
		//四个通道的摇杆数据   RxBuf[1]-RxBuf[8]
		for(i=0;i<4;i++)
		{ 
			//  Sys 0横滚，1俯仰，2油门，3航向 范围
			//  NRF pitch roll yaw thr
			RF_Temp[i]=(short)((unsigned char)RxBuf[i*2+1]<<8|(unsigned char)RxBuf[i*2+2])+500;//数据组装 取正
			if(NRF_Link[i]>1000){NRF_Link[i]=1000;}                //原始数据限幅
			
		}
		//通道映射
		NRF_Link[0] = RF_Temp[1];
		NRF_Link[1] = RF_Temp[0];
		NRF_Link[2] = RF_Temp[3];
		NRF_Link[3] = RF_Temp[2];
		if     ((RxBuf[30]&0x0F)==0x01){NRF_Link[4]=1;NRF_Link[5]=0;}
		else if((RxBuf[30]&0x0F)==0x02){NRF_Link[4]=3;NRF_Link[5]=0;}
		else if((RxBuf[30]&0x0F)==0x04){NRF_Link[4]=2;NRF_Link[5]=0;}
	}
}

