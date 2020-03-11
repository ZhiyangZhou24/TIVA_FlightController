#include "mpu9250.h"
#include "FC_Struct.h"
#include "MY_SPI.h"
void Delay(unsigned int i)
{
	volatile unsigned int j,k;
	for(j=0;j<i;j++)
	{
		for(k=0;k<65534;k++){__nop();}
	}
}
//MPU9250初始化
unsigned char id=0;
bool MPU9250_Init(void)
{
  SSI0_Init();

	bytewrite_MPU(107,0x80); //复位MPU9250
	vTaskDelay(100);
	bytewrite_MPU(107,0x01); //配置MPU9250时钟
	vTaskDelay(1);
	bytewrite_MPU(108,0x00); //启用加速度计与陀螺仪
	vTaskDelay(1);
	bytewrite_MPU(56 ,0x00); //禁用所有中断
	vTaskDelay(1);
	bytewrite_MPU(55, 0x00); //所有中断引脚都无关
	vTaskDelay(1);
	bytewrite_MPU(106,0x30); //开启MPU9250的IIC
	vTaskDelay(1);
	bytewrite_MPU(36 ,0x4D); //MPU9250主机IIC频率400KHz
	vTaskDelay(1);
	bytewrite_MPU(25 ,0x01); //采样率:500Hz
	vTaskDelay(1);
	
	id = ByteRead_MPU(117);
	printf("ID_Is:%d",id);
	
	bytewrite_MPU(26 ,0x03); //陀螺仪滤波
	vTaskDelay(1);
	//0x03 41Hz 5.9ms FS=1KHz
	//0x02 92Hz 3.9ms FS=1KHz
	//0x01 184Hz2.9ms FS=1KHz
	bytewrite_MPU(27 ,0x18); //陀螺仪量程:±2000dps
	vTaskDelay(1);
	bytewrite_MPU(28 ,0x10); //加速度计量程:±8g
	vTaskDelay(1);
	bytewrite_MPU(29 ,0x03); //加速度滤波
	vTaskDelay(10);
	//0x03 41Hz   5.8ms  FS=1KHz
	//0x02 92Hz   7.8ms  FS=1KHz
	//0x01 184Hz  11.9ms FS=1KHz
	AK8963_WriteReg(0x0b,0x01); //AK8963复位
	vTaskDelay(90);
	AK8963_Auto();   //AK8963自动测量并存取
	vTaskDelay(50);
	//随便校验一个寄存器
	if(ByteRead_MPU(26)!=0x03){return false;}
	else{return true;}
}

//AK8963低速写寄存器 500KHz
void AK8963_WriteReg(unsigned char reg, unsigned char value)
{
	bytewrite_MPU(37,0x0c);  //AK8963设备地址
	Delay(1);
	bytewrite_MPU(38, reg);  //AK8963寄存器地址
	Delay(1);
	bytewrite_MPU(99,value);  //data
	Delay(1);
	bytewrite_MPU(39,0x81); //enable
	Delay(10);
	bytewrite_MPU(39,0x00);  //关闭IIC
}
//设置AK8963自动连续测量并存取
void AK8963_Auto(void)
{
	AK8963_WriteReg(0X0A, 0X16);//Continuous measurement mode 2   100Hz
	Delay(1);
	bytewrite_MPU(37, 0x8C);//AK8963 Address READ
	Delay(1);
	bytewrite_MPU(38, 0x02);//AK8963 ST1
	Delay(1);
	bytewrite_MPU(39, 0x88);//enable	  ST1 -- ST2	
	Delay(1);
}
//获取6轴测量数据
void Get_Motion6(void)
{
	unsigned char buffer[14];
	Multiread_MPU(59,buffer,14);
	//±8g 4096LSB/g  最终加速度=原始数据-偏置  低通滤波
	Accel.X_G_NoLPF=((float)(short)((((short)buffer[0])<<8)|buffer[1]))/4096.f;
	Accel.Y_G_NoLPF=((float)(short)((((short)buffer[2])<<8)|buffer[3]))/4096.f;
	Accel.Z_G_NoLPF=((float)(short)((((short)buffer[4])<<8)|buffer[5]))/4096.f;
	Accel.X_G=LPF_Filter(&LP_Ax,Accel.X_G_NoLPF-Accel.X_offset);
	Accel.Y_G=LPF_Filter(&LP_Ay,Accel.Y_G_NoLPF-Accel.Y_offset);
	Accel.Z_G=LPF_Filter(&LP_Az,Accel.Z_G_NoLPF-Accel.Z_offset);
	
	//±2000° 16.4LSB/°/s
	Gyro.X_deg_s_NoLPF=((float)(short)((((short)buffer[8])<<8)|buffer[9]))/16.4f;
	Gyro.Y_deg_s_NoLPF=((float)(short)((((short)buffer[10])<<8)|buffer[11]))/16.4f;
	Gyro.Z_deg_s_NoLPF=((float)(short)((((short)buffer[12])<<8)|buffer[13]))/16.4f;
	Gyro.X_deg_s=LPF_Filter(&LP_Gx,Gyro.X_deg_s_NoLPF-Gyro.X_offset);   //最终角速度=原始数据-偏置
	Gyro.Y_deg_s=LPF_Filter(&LP_Gy,Gyro.Y_deg_s_NoLPF-Gyro.Y_offset);
	Gyro.Z_deg_s=LPF_Filter(&LP_Gz,Gyro.Z_deg_s_NoLPF-Gyro.Z_offset);
}
//获取9轴测量数据   return:传感器数据是否已经更新
void Get_Motion9(void)
{
	unsigned char buffer[22];
	static float Mag_LPF[3]={0};
	float dt=0;
	Multiread_MPU(59,buffer,22);
	//±8g 4096LSB/g  最终加速度=原始数据-偏置  低通滤波
	Accel.X_G_NoLPF=((float)(short)((((short)buffer[0])<<8)|buffer[1]))/4096.f;
	Accel.Y_G_NoLPF=((float)(short)((((short)buffer[2])<<8)|buffer[3]))/4096.f;
	Accel.Z_G_NoLPF=((float)(short)((((short)buffer[4])<<8)|buffer[5]))/4096.f;
	Accel.X_G=LPF_Filter(&LP_Ax,Accel.X_G_NoLPF-Accel.X_offset);
	Accel.Y_G=LPF_Filter(&LP_Ay,Accel.Y_G_NoLPF-Accel.Y_offset);
	Accel.Z_G=LPF_Filter(&LP_Az,Accel.Z_G_NoLPF-Accel.Z_offset);
	//±2000° 16.4LSB/°/s
	Gyro.X_deg_s_NoLPF=((float)(short)((((short)buffer[8])<<8)|buffer[9]))/16.4f;
	Gyro.Y_deg_s_NoLPF=((float)(short)((((short)buffer[10])<<8)|buffer[11]))/16.4f;
	Gyro.Z_deg_s_NoLPF=((float)(short)((((short)buffer[12])<<8)|buffer[13]))/16.4f;
	Gyro.X_deg_s=LPF_Filter(&LP_Gx,Gyro.X_deg_s_NoLPF-Gyro.X_offset);   //最终角速度=原始数据-偏置
	Gyro.Y_deg_s=LPF_Filter(&LP_Gy,Gyro.Y_deg_s_NoLPF-Gyro.Y_offset);
	Gyro.Z_deg_s=LPF_Filter(&LP_Gz,Gyro.Z_deg_s_NoLPF-Gyro.Z_offset);
	// 0.15uT/LSB 1Ga=100uT
	Mag.X_Ga_NoLPF=((float)(short)((((short)buffer[16])<<8)|buffer[15]))*0.15f/100.0f;
	Mag.Y_Ga_NoLPF=((float)(short)((((short)buffer[18])<<8)|buffer[17]))*0.15f/100.0f;
	Mag.Z_Ga_NoLPF=((float)(short)((((short)buffer[20])<<8)|buffer[19]))*0.15f/100.0f;
	Mag.X_Ga_LPF=LPF_Filter(&LP_Mx,Mag.X_Ga_NoLPF);
	Mag.Y_Ga_LPF=LPF_Filter(&LP_My,Mag.Y_Ga_NoLPF);
	Mag.Z_Ga_LPF=LPF_Filter(&LP_Mz,Mag.Z_Ga_NoLPF);
	if(Mag.Xs>0.0f&&Mag.Ys>0.0f&&Mag.Zs>0.0f)
	{
		Mag.X_Ga=(Mag.X_Ga_NoLPF-Mag.X_offset)/Mag.Xs;
	  Mag.Y_Ga=(Mag.Y_Ga_NoLPF-Mag.Y_offset)/Mag.Ys;
	  Mag.Z_Ga=(Mag.Z_Ga_NoLPF-Mag.Z_offset)/Mag.Zs;
	}
	else
	{
		Mag.X_Ga=Mag.X_Ga_NoLPF-Mag.X_offset;
	  Mag.Y_Ga=Mag.Y_Ga_NoLPF-Mag.Y_offset;
	  Mag.Z_Ga=Mag.Z_Ga_NoLPF-Mag.Z_offset;
	}
//	//计算低通滤波dt以及记录磁力计传感器读取数据时间戳
//	dt=(timestamp_data-Mag.timestamp)/1000000.0f;
//	Mag.timestamp=timestamp_data;
	//磁力计低通滤波
	Mag_LPF[0]=Mag_LPF[0]*(1.0f-dt*8.0f)+Mag.X_Ga*dt*8.0f;
	Mag_LPF[1]=Mag_LPF[1]*(1.0f-dt*8.0f)+Mag.Y_Ga*dt*8.0f;
	Mag_LPF[2]=Mag_LPF[2]*(1.0f-dt*8.0f)+Mag.Z_Ga*dt*8.0f;
	//计算磁力计数据最大取值范围
	static float LimitX=0,LimitY=0,LimitZ=0,Limit_Max=0;
	LimitX=fabsf((Mag.Xmax-Mag.Xmin)/2.0f);Limit_Max=LimitX;
	LimitY=fabsf((Mag.Ymax-Mag.Ymin)/2.0f);if(Limit_Max<LimitY){Limit_Max=LimitY;}
	LimitZ=fabsf((Mag.Zmax-Mag.Zmin)/2.0f);if(Limit_Max<LimitZ){Limit_Max=LimitZ;}
//	//磁力计还没有被校准
//	if(Take_Off_Protect.flags.Mag_Need_Calibration==true)
//	{
//		//标记磁力计不可用
//		Mag.Vaild=false;
//	}
//	//传感器正在进行校准
//	else if(Take_Off_Protect.flags.Dof6_Calibration==true)
//	{
//		//标记磁力计不可用
//		Mag.Vaild=false;
//	}
	//磁力计数据错误
//	else if(Mag.X_Ga_NoLPF==0.0f&&Mag.Y_Ga_NoLPF==0.0f&&Mag.Z_Ga_NoLPF==0.0f)
//	{
//		//标记磁力计受干扰
//		Mag.Vaild=false;
//	}
//	//磁力计错误判定	限幅为最大取值范围放大1.25-2.0倍
//	else if(fabsf(Mag_LPF[0])>Limit_Max*1.5f||fabsf(Mag_LPF[1])>Limit_Max*1.5f||fabsf(Mag_LPF[2])>Limit_Max*1.5f)
//  {
//		//标记磁力计受干扰
//		Mag.Vaild=false;
//		Take_Off_Protect.flags.Mag_Interference=true;
//	}
//	else
//	{
//		//标记磁力计正常
//		Mag.Vaild=true;
//		//磁力计已经被标注受到干扰,则复位标识
//		if(Take_Off_Protect.flags.Mag_Interference==true){Take_Off_Protect.flags.Mag_Interference=false;}
//	}
}
