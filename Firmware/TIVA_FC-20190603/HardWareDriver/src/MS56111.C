#include "FC_Struct.h"
#include "MS5611.h"

//************************************MS5611�������*********************************
volatile unsigned short Cal_C[7]={0};	      
volatile unsigned int D1_Pres=0,D2_Temp=0;
volatile double	dT=0,TEMP=0;
volatile double OFF_=0,SENS=0;	
volatile double T2=0,Aux=0,OFF2=0,SENS2=0;
double const SeaLevel=101325;   //��ƽ����ѹ101.325kpa
volatile double Pressure=0;		            //��ǰ����ѹ  ֻ��   ��λ��mbar
//*******************************��ѹ�˲�*******************************************
float Pre_X_last=0; //��һʱ�̵����Ž��
float Pre_X_mid=0;  //��ǰʱ�̵�Ԥ����
float Pre_X_now=0;  //��ǰʱ�̵����Ž��
float Pre_P_mid=0;  //��ǰʱ��Ԥ������Э����
float Pre_P_now=0;  //��ǰʱ�����Ž����Э����
float Pre_P_last=0; //��һʱ�����Ž����Э����
float Pre_kg=0;     //����������
float Pre_Kalman_Q=1.0; //ϵͳ����
float Pre_R=500.0;     //��������

void MS5611_Task(void)
{
	 static bool status=0;
	 static unsigned char Fix_Timer=0;
	 static unsigned char Err_Timer=0;
	 static bool Baro_Data_Error=false;
	 //��ѹ��OSR 4096 ת��ʱ�����9.04ms
	 //�¶ȼ�OSR 256  ת��ʱ�����0.60ms
	 //OSR 256  0.60ms
	 //OSR 512  1.17ms
	 //OSR 1024 2.28ms
	 //OSR 2048 4.54ms
	 //OSR 4096 9.04ms
	 status=false;
	 status|=MS5611_GetTemp2();      //�����¶�ת��ָ�� OSR 256
	 vTaskDelay(configTICK_RATE_HZ/1300); //�ӳ�0.77ms
	 status|=MS5611_GetTemp1();      //��ȡ��һ���ڷ��͵��¶�ת��ָ��
	 vTaskDelay(1);
	 status|=MS5611_GetPressure2();  //����ѹ��ת��ָ�� OSR 4096
	 vTaskDelay(configTICK_RATE_HZ/105); //�ӳ�9.52ms
	 status|=MS5611_GetPressure1();  //��ȡ��һ���ڿ�ʼת����ѹ������
	 vTaskDelay(1);
	 if(Cal_C[1]==0x0000||Cal_C[2]==0x0000||Cal_C[3]==0x0000||Cal_C[4]==0x0000||Cal_C[5]==0x0000||Cal_C[6]==0x0000){status=true;}
	 if(status==false)
	 {
		 OFF_=(double)Cal_C[2]*65536.0+((double)Cal_C[4]*dT)/128.0;
	   SENS=(double)Cal_C[1]*32768.0+((double)Cal_C[3]*dT)/256.0;
	   if(TEMP<2000.0)
	   {
		   T2 = (dT*dT) / (double)0x80000000;
		   Aux = (TEMP-2000.0)*(TEMP-2000.0);
		   OFF2 = 2.5*Aux;
		   SENS2 = 1.25*Aux; 
			 if(TEMP<-1500)
			 {
				 double Aux2=(TEMP+1500.0)*(TEMP+1500.0);
				 OFF2+=7.0*Aux2;
				 SENS2+=5.5*Aux2;
			 }
		   TEMP = TEMP - T2;
		   OFF_ = OFF_ - OFF2;
		   SENS = SENS - SENS2;	
	   }
	   Pressure=((double)D1_Pres*(double)SENS/(double)2097152-(double)OFF_)/(double)32768; //�¶��ںϺ�ó���ѹֵ
		 
		 //��ѹ�������쳣���ж�
		 {
			 static double Barod;
			 static unsigned short Baro_Er_Timer=0;
			 static unsigned short Baro_ok_Timer=0;
			 //��ѹ������û�б䶯��
			 if(Barod==Pressure)
			 {
				 if(Baro_ok_Timer>1){Baro_ok_Timer-=1;}
				 Baro_Er_Timer+=1;if(Baro_Er_Timer>10){Baro_Er_Timer=10;}
			 }
			 else
			 {
				 if(Baro_Er_Timer>1){Baro_Er_Timer-=1;}
				 Barod=Pressure;
			 	 Baro_ok_Timer+=1;if(Baro_ok_Timer>50){Baro_ok_Timer=50;}
			 }
			 if(Baro_Er_Timer==10){Baro_Data_Error=true;Take_Off_Protect.flags.Baro_Error=true;}
			 else if(Baro_ok_Timer==50&&Baro_Data_Error==true){Baro_Data_Error=false;}
		 }
	   Baro.Alt_meter=constrain0((float)(44330.0*(1.0-pow((Pressure)/101325.0,0.190295))),-1000.0f,25000.0f);
		 //��ѹ�������쳣�ж�2
		 if(Baro.Alt_meter>30000.0f||Baro.Alt_meter<-500.0f){Baro_Data_Error=true;Take_Off_Protect.flags.Baro_Error=true;}
     Baro.KF_meter=KalmanFilter_Pressure(Baro.Alt_meter); //��λ:��
	   Baro.timestamp=timestamp_data;
		 Baro.Vaild=true;
   }
	 if(status==false)
	 {
		 Fix_Timer++;if(Fix_Timer>10){Fix_Timer=10;}
		 Err_Timer=0;
	 }
	 else if(status==true)
	 {
		 Fix_Timer=0;
		 Err_Timer++;if(Err_Timer>5){Err_Timer=5;}
		 Device.Connect.flags.Baro=false;
		 Take_Off_Protect.flags.Baro_Error=true;
		 Baro.Vaild=false;
	 }
	 if(Fix_Timer==10&&Baro_Data_Error==false)
   {
	   Device.Connect.flags.Baro=true;
	   Take_Off_Protect.flags.Baro_Error=false;
	 }
	 //��������ﵽ����,��������ѹ��
	 if(Err_Timer==5){MS5611_Init();}
}
//��ʼ��MS5611
//return:1�ɹ�  0ʧ��
bool MS5611_Init(void)
{
	Chip_SCU_PinMuxSet(2,3,(SCU_MODE_INACT|SCU_MODE_HIGHSPEEDSLEW_EN|SCU_MODE_INBUFF_EN|SCU_MODE_FUNC4|SCU_MODE_ZIF_DIS));   //MS5611_MISO
	Chip_SCU_PinMuxSet(2,4,(SCU_MODE_INACT|SCU_MODE_HIGHSPEEDSLEW_EN|SCU_MODE_20MA_DRIVESTR|SCU_MODE_FUNC4|SCU_MODE_ZIF_DIS));//MS5611_MOSI
	Chip_SCU_PinMuxSet(3,8,(SCU_MODE_INACT|SCU_MODE_HIGHSPEEDSLEW_EN|SCU_MODE_20MA_DRIVESTR|SCU_MODE_FUNC4|SCU_MODE_ZIF_DIS));//MS5611_SCK 
  Chip_SCU_PinMuxSet(1,9,(SCU_MODE_INACT|SCU_MODE_HIGHSPEEDSLEW_EN|SCU_MODE_20MA_DRIVESTR|SCU_MODE_FUNC0|SCU_MODE_ZIF_DIS));//MS5611_CE 
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,5,11); //MS5611_SCK
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,5,4);  //MS5611_MOSI
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT,1,2);  //MS5611_CE
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT,5,3);  //MS5611_MISO
	MS5611_CE=1;
	MS5611_SCK=0;
	bool status=false;
	status|=MS5611_RESET();
	status|=MS5611_PROM_READ();
	return !status;
}
//SSP0���ٶ�дһ���ֽڵ����� 6MHz
unsigned char MS5611_RW_Fast(unsigned char uchar)
{
	  unsigned char bit_ctr;
   	for(bit_ctr=0;bit_ctr<8;bit_ctr++) // output 8-bit
   	{
			if((uchar&0x80)==0x00){MS5611_MOSI=0;}
			else{MS5611_MOSI=1;}
			uchar=uchar<<1;
			Delay_8MHz();
			MS5611_SCK=1;
			Delay_8MHz();
			uchar|=MS5611_MISO;
			MS5611_SCK=0;                      
   	}
    return(uchar);  
}
//MS5611��16λ����
unsigned short MS5611_RD_16bit(unsigned char reg)
{
	unsigned short i;
	MS5611_RW_Fast(reg);
	i=MS5611_RW_Fast(0)<<8;
	i|=MS5611_RW_Fast(0);
	return i;
}
//��λMS5611
//return:1�ɹ�  0ʧ��
bool MS5611_RESET(void)
{
	volatile unsigned long i;
	MS5611_CE=0;
	MS5611_RW_Fast(CMD_MS5611_RESET);
	vTaskDelay(configTICK_RATE_HZ/50);
	MS5611_CE=1;
	vTaskDelay(1);
	return false;
}
//��MS5611����PROM����
//return:1�ɹ�  0ʧ��
bool MS5611_PROM_READ(void)
{
	MS5611_CE=0;Cal_C[0]=MS5611_RD_16bit(CMD_MS5611_PROM_RD);MS5611_CE=1;vTaskDelay(1);
	MS5611_CE=0;Cal_C[1]=MS5611_RD_16bit(CMD_MS5611_PROM_C1);MS5611_CE=1;vTaskDelay(1);
	MS5611_CE=0;Cal_C[2]=MS5611_RD_16bit(CMD_MS5611_PROM_C2);MS5611_CE=1;vTaskDelay(1);
	MS5611_CE=0;Cal_C[3]=MS5611_RD_16bit(CMD_MS5611_PROM_C3);MS5611_CE=1;vTaskDelay(1);
	MS5611_CE=0;Cal_C[4]=MS5611_RD_16bit(CMD_MS5611_PROM_C4);MS5611_CE=1;vTaskDelay(1);
	MS5611_CE=0;Cal_C[5]=MS5611_RD_16bit(CMD_MS5611_PROM_C5);MS5611_CE=1;vTaskDelay(1);
	MS5611_CE=0;Cal_C[6]=MS5611_RD_16bit(CMD_MS5611_PROM_C6);MS5611_CE=1;vTaskDelay(1);
  if(Cal_C[1]==0xffff||Cal_C[1]==0x0000){return true;}
	if(Cal_C[2]==0xffff||Cal_C[2]==0x0000){return true;}
	if(Cal_C[3]==0xffff||Cal_C[3]==0x0000){return true;}
	if(Cal_C[4]==0xffff||Cal_C[4]==0x0000){return true;}
	if(Cal_C[5]==0xffff||Cal_C[5]==0x0000){return true;}
	if(Cal_C[6]==0xffff||Cal_C[6]==0x0000){return true;}
	return false;
}
//MS5611��ȡѹ�����ݺ���
//return:1�ɹ�  0ʧ��
bool MS5611_GetPressure1(void)
{
	static unsigned long conv1=0,conv2=0,conv3=0; 
	MS5611_CE=0;
  MS5611_RW_Fast(MS5611_ADC);
	conv1=MS5611_RW_Fast(0);
	conv2=MS5611_RW_Fast(0);
	conv3=MS5611_RW_Fast(0);
	MS5611_CE=1;
	//��ȡ��һ������ת���õ���ѹ������
	D1_Pres=(((unsigned int)conv1)<<16)|(((unsigned int)conv2)<<8)|((unsigned int)conv3); 
	if((conv1==0x00&&conv2==0x00&&conv3==0x00)||(conv1==0xff&&conv2==0xff&&conv3==0xff)){return true;}
	else{return false;}
}
//MS5611����ѹ��ת��ָ���
//return:1�ɹ�  0ʧ��
bool MS5611_GetPressure2(void)
{
	MS5611_CE=0;
	MS5611_RW_Fast(0x48); //ѹ��OSR 4096
	MS5611_CE=1;
  return false;
}
//MS5611��ȡ�¶����ݺ���
//return:1�ɹ�  0ʧ��
bool MS5611_GetTemp1(void)
{
	static unsigned long conv1=0,conv2=0,conv3=0; 
	MS5611_CE=0;
  MS5611_RW_Fast(MS5611_ADC);
	conv1=MS5611_RW_Fast(0);
	conv2=MS5611_RW_Fast(0);
	conv3=MS5611_RW_Fast(0);
	MS5611_CE=1;
	//��ȡ��һ������ת���õ����¶�
	D2_Temp=(((unsigned int)conv1)<<16)|(((unsigned int)conv2)<<8)|((unsigned int)conv3);   
	dT=(double)((int)D2_Temp-(int)(((unsigned int)Cal_C[5])<<8));
	TEMP=2000.0+dT*((double)Cal_C[6]/8388608.0);
  Baro.Temp=TEMP/100.0f;
	if((conv1==0x00&&conv2==0x00&&conv3==0x00)||(conv1==0xff&&conv2==0xff&&conv3==0xff)){return true;}
	else{return false;}
}
//MS5611�����¶�ת��ָ���
//return:1�ɹ�  0ʧ��
bool MS5611_GetTemp2(void)
{
	MS5611_CE=0;
	MS5611_RW_Fast(0x50); //�¶�OSR 256
	MS5611_CE=1;
	return false;
}
//MS5611��ѹ���ݿ������˲�
float KalmanFilter_Pressure(float dat)
{
	  if(((Pre_X_last+5)>dat)||((Pre_X_last-5)<dat))
		{
			Pre_X_last=dat;
			Pre_X_now=dat;
			return Pre_X_now;
		}
    Pre_X_mid = Pre_X_last;                    //x(k|k-1) = AX(k-1|k-1)+BU(k)
    Pre_P_mid = Pre_P_last+Pre_Kalman_Q;       //p(k|k-1) = Ap(k-1|k-1)A'+Q
    Pre_kg = Pre_P_mid/(Pre_P_mid+Pre_R);      //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    Pre_X_now=Pre_X_mid+Pre_kg*(dat-Pre_X_mid);//x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    Pre_P_now=(1.0f-Pre_kg)*Pre_P_mid;         //p(k|k) = (I-kg(k)H)P(k|k-1)
    Pre_P_last=Pre_P_now;                      //״̬����
    Pre_X_last=Pre_X_now;
    return Pre_X_now;
}