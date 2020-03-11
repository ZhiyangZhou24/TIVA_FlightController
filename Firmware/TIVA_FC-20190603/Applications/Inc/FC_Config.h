#ifndef _FC_CONFIG_H_
#define _FC_CONFIG_H_
//================ϵͳ===================
#define CTRL_HEIGHT 1       //0ʧ�ܣ�1ʹ��

#define USE_US100           //ʹ��us100�ͺų����� 
// #define USE_KS103					//ʹ��ks103�ͺų�����

#define MAXMOTORS 		(4)		//�������
#define GET_TIME_NUM 	(5)		//���û�ȡʱ�����������
#define CH_NUM 				(6) 	//���ջ�ͨ������

#define USE_TOE_IN_UNLOCK 1 // 0��Ĭ�Ͻ�����ʽ��1����˽�����ʽ
#define ANO_DT_USE_USART2 	//��������2��������
#define ANO_DT_USE_USB_HID	//�����ɿ�USBHID������λ������
//=======================================


//================������===================
#define ACC_ADJ_EN 									//�Ƿ�����У׼���ٶȼ�,(����������)

#define OFFSET_AV_NUM 	50					//У׼ƫ����ʱ��ƽ��������
#define FILTER_NUM 			10					//����ƽ���˲���ֵ����

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//������Y����в���
#define FIX_GYRO_X 			1.02f				//������X����в���

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת����

#define MAX_ACC  4096.0f						//+-8G		���ٶȼ�����
#define TO_DEG_S 500.0f      				//T = 2ms  Ĭ��Ϊ2ms ����ֵ����1/T

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};

// CH_filter[],0�����1������2���ţ�3����		
enum
{
 ROL= 0,
 PIT ,
 THR ,
 YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
};
//=========================================

//================����=====================
#define MAX_CTRL_ANGLE			25.0f										//ң���ܴﵽ�����Ƕ�
#define ANGLE_TO_MAX_AS 		30.0f										//�Ƕ����Nʱ���������ٶȴﵽ��󣨿���ͨ������CTRL_2��Pֵ������
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE		//�⻷���ַ���

#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT����������ƽ��ٶ�
#define MAX_CTRL_YAW_SPEED 	150.0f									//YAW����������ƽ��ٶ�
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED		//�ڻ����ַ���


#define MAX_PWM				100			///%	���PWM���Ϊ100%����
#define MAX_THR       80 			///%	����ͨ�����ռ��80%����20%��������
#define READY_SPEED   20			///%	��������ת��20%����
//=========================================
#endif
