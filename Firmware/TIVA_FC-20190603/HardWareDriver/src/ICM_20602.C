#include "FC_Struct.h"
#include "icm_20602.h"
#include "FC_Struct.h"
#include "MY_SPI.h"
#include "mymath.h"
#include "parameter.h"
#include "time.h"

ICM_STRUCT icm20602;

u8 icm20602_buffer[14];
u8 icm20602_ok;
unsigned char id;

//ICM-20602初始化
bool ICM_20602_Init(void)
{
  bytewrite_MPU(107,0x80); //复位ICM-20602芯片
	Delay_ms(1);
  bytewrite_MPU(104,0x03); //复位ICM-20602加速度计与温度计
	Delay_ms(1);
	bytewrite_MPU(107,0x01); //配置ICM-20602时钟
	Delay_ms(10);
	if(ByteRead_MPU(107)!=0x01){return 1;}
	
	bytewrite_MPU(108,0x00); //启用加速度计与陀螺仪
	Delay_ms(10);
	if(ByteRead_MPU(108)!=0x00){return 1;}
	
	bytewrite_MPU(55, 0x00); //所有中断引脚都无关
	Delay_ms(10);
	if(ByteRead_MPU(55)!=0x00){return 1;}
	
	bytewrite_MPU(25 ,0x00); //陀螺仪采样速率
	Delay_ms(10);
	if(ByteRead_MPU(25)!=0x00){return 1;}
	
	bytewrite_MPU(26 ,0x03); //陀螺仪滤波
	Delay_ms(10);
	if(ByteRead_MPU(26)!=0x03){return 1;}  //0x03 42HZ低通 0x04 20HZ低通
	
	bytewrite_MPU(27 ,0x18); //陀螺仪量程:±2000dps
	Delay_ms(10);
	if(ByteRead_MPU(27)!=0x18){return 1;}
	
	bytewrite_MPU(28 ,0x10); //加速度计量程:±8g
	Delay_ms(10);
	if(ByteRead_MPU(28)!=0x10){return 1;}
	
	bytewrite_MPU(29 ,0x03); //加速度滤波
	Delay_ms(10);
	if(ByteRead_MPU(29)!=0x03){return 1;}
	id = ByteRead_MPU(117);
	if(ByteRead_MPU(117)!=0x12)
	{
		printf("6Dof_Senser_id:0x%x!\n",ByteRead_MPU(117));
		return 1;
	}
  return 0;
}
float constrain0(float val, float min, float max)
{
	if(val<min||val>max){return 0;}
	return val;
}
//获取6轴测量数据
void ICM_Get_Motion6(void)
{
	
}
s32 sum_temp[7]={0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,gyro_sum_cnt = 0;

void icm20602_Data_Offset()
{
#ifdef ACC_ADJ_EN

	if(icm20602.Acc_CALIBRATE == 1)
	{
		if(my_sqrt(my_pow(icm20602.Acc_I16.x)+my_pow(icm20602.Acc_I16.y)+my_pow(icm20602.Acc_I16.z)) < 2500)
		{
			sensor_setup.Offset.mpu_flag = 1;
		}
		else if(my_sqrt(my_pow(icm20602.Acc_I16.x)+my_pow(icm20602.Acc_I16.y)+my_pow(icm20602.Acc_I16.z)) > 2600)
		{
			sensor_setup.Offset.mpu_flag = 0;
		}
						
    acc_sum_cnt++;
		sum_temp[A_X] += icm20602.Acc_I16.x;
		sum_temp[A_Y] += icm20602.Acc_I16.y;
		sum_temp[A_Z] += icm20602.Acc_I16.z - 65536/16;   // +-8G
		sum_temp[TEM] += icm20602.Tempreature;

    if( acc_sum_cnt >= OFFSET_AV_NUM )
		{
			icm20602.Acc_Offset.x = (float)sum_temp[A_X]/OFFSET_AV_NUM;
			icm20602.Acc_Offset.y = (float)sum_temp[A_Y]/OFFSET_AV_NUM;
			icm20602.Acc_Offset.z = (float)sum_temp[A_Z]/OFFSET_AV_NUM;
			icm20602.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			acc_sum_cnt =0;
			if(icm20602.Acc_CALIBRATE == 1)
				Save_Acc_Calibration_Date(&icm20602.Acc_Offset);
			icm20602.Acc_CALIBRATE = 0;
			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
		}	
	}

#endif

	if(icm20602.Gyro_CALIBRATE)
	{
		gyro_sum_cnt++;
		sum_temp[G_X] += icm20602.Gyro_I16.x;
		sum_temp[G_Y] += icm20602.Gyro_I16.y;
		sum_temp[G_Z] += icm20602.Gyro_I16.z;
		sum_temp[TEM] += icm20602.Tempreature;

    if( gyro_sum_cnt >= OFFSET_AV_NUM )
		{
			icm20602.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
			icm20602.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
			icm20602.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
			icm20602.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
			gyro_sum_cnt =0;
			if(icm20602.Gyro_CALIBRATE == 1)
				Save_Gyro_Calibration_Date(&icm20602.Gyro_Offset);
			icm20602.Gyro_CALIBRATE = 0;
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
		}
	}
}
void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;
}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float icm20602_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;

void icm20602_Data_Prepare(float T)
{	
	u8 i;
	s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
//	float auto_offset_temp[3];
  float Gyro_tmp[3];
	Multiread_MPU(59,icm20602_buffer,14);

	icm20602.Acc_I16.x = ((float)(short)((((unsigned short)icm20602_buffer[0])<<8)|(unsigned short)icm20602_buffer[1]));
	icm20602.Acc_I16.y = ((float)(short)((((unsigned short)icm20602_buffer[2])<<8)|(unsigned short)icm20602_buffer[3]));
	icm20602.Acc_I16.z = ((float)(short)((((unsigned short)icm20602_buffer[4])<<8)|(unsigned short)icm20602_buffer[5]));
	
	icm20602.Gyro_I16.x = ((float)(short)((((unsigned short)icm20602_buffer[8])<<8)|(unsigned short)icm20602_buffer[9]));
	icm20602.Gyro_I16.y = ((float)(short)((((unsigned short)icm20602_buffer[10])<<8)|(unsigned short)icm20602_buffer[11]));
	icm20602.Gyro_I16.z = ((float)(short)((((unsigned short)icm20602_buffer[12])<<8)|(unsigned short)icm20602_buffer[13]));
	
  icm20602_Data_Offset(); //校准函数
	Gyro_tmp[0] = icm20602.Gyro_I16.x ;//
  Gyro_tmp[1] = icm20602.Gyro_I16.y ;//
	Gyro_tmp[2] = icm20602.Gyro_I16.z ;//

	icm20602.Tempreature = ((((int16_t)icm20602_buffer[6]) << 8) | icm20602_buffer[7]); //tempreature
	icm20602.TEM_LPF += 2 *3.14f *T *(icm20602.Tempreature - icm20602.TEM_LPF);
	icm20602.Ftempreature = icm20602.TEM_LPF/340.0f + 36.5f;

//======================================================================
	if( ++filter_cnt > FILTER_NUM )	
	{
		filter_cnt = 0;
		filter_cnt_old = 1;
	}
	else
	{
		filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
	}
//10 170 4056
	/* 得出校准后的数据 */
	if(sensor_setup.Offset.mpu_flag == 0)
	{
		icm20602_tmp[A_X] = (icm20602.Acc_I16.x - icm20602.Acc_Offset.x) ;
		icm20602_tmp[A_Y] = (icm20602.Acc_I16.y - icm20602.Acc_Offset.y) ;
		icm20602_tmp[A_Z] = (icm20602.Acc_I16.z - icm20602.Acc_Offset.z) ;
	}
	else
	{
		icm20602_tmp[A_X] = 2*(icm20602.Acc_I16.x - icm20602.Acc_Offset.x) ;
		icm20602_tmp[A_Y] = 2*(icm20602.Acc_I16.y - icm20602.Acc_Offset.y) ;
		icm20602_tmp[A_Z] = 2*(icm20602.Acc_I16.z - icm20602.Acc_Offset.z - 2048) ;
	}
	
	icm20602_tmp[G_X] = Gyro_tmp[0] - icm20602.Gyro_Offset.x ;//
	icm20602_tmp[G_Y] = Gyro_tmp[1] - icm20602.Gyro_Offset.y ;//
	icm20602_tmp[G_Z] = Gyro_tmp[2] - icm20602.Gyro_Offset.z ;//
	

	/* 更新滤波滑动窗口数组 */
	FILT_BUF[A_X][filter_cnt] = icm20602_tmp[A_X];
	FILT_BUF[A_Y][filter_cnt] = icm20602_tmp[A_Y];
	FILT_BUF[A_Z][filter_cnt] = icm20602_tmp[A_Z];
	FILT_BUF[G_X][filter_cnt] = icm20602_tmp[G_X]; 
	FILT_BUF[G_Y][filter_cnt] = icm20602_tmp[G_Y];
	FILT_BUF[G_Z][filter_cnt] = icm20602_tmp[G_Z];

	for(i=0;i<FILTER_NUM;i++)
	{
		FILT_TMP[A_X] += FILT_BUF[A_X][i];
		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}


	mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
	mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
	mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


	mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
	mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
	mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;
	
	
	/*坐标转换*/
	Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&icm20602.Acc.x,&icm20602.Acc.y,&icm20602.Acc.z);
	Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&icm20602.Gyro.x,&icm20602.Gyro.y,&icm20602.Gyro.z);

	icm20602.Gyro_deg.x = icm20602.Gyro.x *TO_ANGLE;
	icm20602.Gyro_deg.y = icm20602.Gyro.y *TO_ANGLE;
	icm20602.Gyro_deg.z = icm20602.Gyro.z *TO_ANGLE;

//======================================================================
}


