#include "App_RadioLink.h"
#include "my_timer.h"
#include "NRF24L01.H"
#include "rc.h"
/*            RF���ջ�ѡ��
*
* NRF24L01 ===>> ����2.4G 24l01�����շ�оƬ
* DJI_DR16 ===>> ��Ӵ�DR16���ջ�
*
*/
unsigned char RF_Reciver = DJI_DR16;

/****************************************/
/*NRF24L01 ���ͽ��ջ�����*/
unsigned char TxBuf[32];
unsigned char RxBuf[32];
/*DBUS ���ջ���ͨ������*/
u16 DBUS_Link[6];
u16 NRF_Link[6];
unsigned char USART_RX_BUF[25]; //���ڽ��ջ�����
unsigned int Pack_Num=0;    //1sʱ�����հ���
unsigned char Sync_Time=0;  //ͬ��ʱ�䣬�󽮽��ջ�ͬ��ʱ�����
/************DBUSЭ�����*************/
void UART1_Handler(void)
{
  static unsigned char res;
	static uint16_t Data_num=0;
	static uint64_t USART1_timestamp=0;
	static uint64_t DBUS_timestamp=0;
	
	//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־
	uint32_t flag = UARTIntStatus(UART1_BASE,1);
	//����жϱ�־
	UARTIntClear(UART1_BASE,flag);
	if(flag&UART_INT_RX)
	{
		if(UARTCharsAvail(UART1_BASE))//���յ�����
		{
			 res = UARTCharGet(UART1_BASE); //�����Ĵ�������
			 /* ��ȡʱ��� �ж϶����Լ�����������ջ�  ʵ�ʴ󽮽��ջ�14msͬ��һ������   ����7msͬ��һ������*/
			 if((TimeStamp_1ms-USART1_timestamp)>7ull)
			 {
				 Data_num=0;
			 }
			 /* �������� */
			 USART_RX_BUF[Data_num++]=res;

			 if(Data_num==17)  //ң�������ݽ������
			 {
				Sync_Time = TimeStamp_1ms - DBUS_timestamp;
				Pack_Num++;
				//���
				DBUS_Link[0]=(((unsigned short)USART_RX_BUF[1]&0x07)<<8)+(unsigned short)USART_RX_BUF[0];
				//����
				DBUS_Link[1]=(((unsigned short)USART_RX_BUF[2]&0x3F)<<5)+(unsigned short)(USART_RX_BUF[1]>>3);
				//ƫ��
				DBUS_Link[3]=(((unsigned short)USART_RX_BUF[4]&0x01)<<10)+((unsigned short)USART_RX_BUF[3]<<2)+((unsigned short)USART_RX_BUF[2]>>6);
				 //����
				DBUS_Link[2]=(((unsigned short)USART_RX_BUF[5]&0x0F)<<7)+((unsigned short)USART_RX_BUF[4]>>1);
				//�󲦸� 1 3 2
				DBUS_Link[4]=((unsigned short)USART_RX_BUF[5]>>6);
				//�Ҳ��� 1 3 2
				DBUS_Link[5]=(((unsigned short)USART_RX_BUF[5]&0x30)>>4);
				
				Data_num=0;
				DBUS_timestamp=TimeStamp_1ms;
			 }
			 USART1_timestamp=TimeStamp_1ms;
		}
	}
}

//����1��ʼ��  DR16���ջ� ����DBUS Э��
void RadioLink_Init(void)
{
	if(RF_Reciver == DJI_DR16)
	{
		//��ʼ������1����
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		GPIOPinConfigure(GPIO_PB0_U1RX);
		GPIOPinConfigure(GPIO_PB1_U1TX);
		GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		UARTConfigSetExpClk(UART1_BASE,Systermcoreclock, 100000, UART_CONFIG_WLEN_8 |// 100Kbps  Ĭ��������
									UART_CONFIG_PAR_EVEN | UART_CONFIG_STOP_ONE);  //żУ��  һֹͣλ
		//����FIFO�����1λ�Ͳ����ж�
		UARTFIFODisable(UART1_BASE); 
		//ʹ��UART0�ж�
		IntEnable(INT_UART1);
		//ʹ��UART0�ж�
		UARTIntEnable(UART1_BASE, UART_INT_RX);
		//UART�жϵ�ַע��
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
	//ʹ��ȫ���ж�
	IntMasterEnable();
	//��ʱ���
	Timer0A_Init();
}
unsigned char Check_num,i;
/*����NRF24L01 ���ݺ��� ����Э��ΪMFELinkЭ�� �û�������ж���Э��*/
void NRF_Task(void)
{
	static u16 RF_Temp[6];
	if(nRF24L01_RxPacket(RxBuf))
	{
		Check_num=0;                //У�鿪ʼ
		for(i=0;i<31;i++){Check_num+=RxBuf[i];}
		if(Check_num == RxBuf[31])  //У��ͨ��
		{
			Pack_Num++;               //�ձ����ݴ����
		}
		//�ĸ�ͨ����ҡ������   RxBuf[1]-RxBuf[8]
		for(i=0;i<4;i++)
		{ 
			//  Sys 0�����1������2���ţ�3���� ��Χ
			//  NRF pitch roll yaw thr
			RF_Temp[i]=(short)((unsigned char)RxBuf[i*2+1]<<8|(unsigned char)RxBuf[i*2+2])+500;//������װ ȡ��
			if(NRF_Link[i]>1000){NRF_Link[i]=1000;}                //ԭʼ�����޷�
			
		}
		//ͨ��ӳ��
		NRF_Link[0] = RF_Temp[1];
		NRF_Link[1] = RF_Temp[0];
		NRF_Link[2] = RF_Temp[3];
		NRF_Link[3] = RF_Temp[2];
		if     ((RxBuf[30]&0x0F)==0x01){NRF_Link[4]=1;NRF_Link[5]=0;}
		else if((RxBuf[30]&0x0F)==0x02){NRF_Link[4]=3;NRF_Link[5]=0;}
		else if((RxBuf[30]&0x0F)==0x04){NRF_Link[4]=2;NRF_Link[5]=0;}
	}
}

