#include "include.h"
#include "ctrl.h"
#include "string.h"
#include "height_ctrl.h"
#include "my_eeprom.h"
#include "icm_20602.h"
#include "ist8310.h"
u8 flash_init_error;
sensor_setup_t sensor_setup;
pid_setup_t pid_setup;
/*===参数存储表========      EEPROM扇区号*/
/*Acc_Offset EEPROM扇区号        0
**Gyo_Offset EEPROM扇区号        1
**Mag_Offset EEPROM扇区号        2
**PID1==>>姿态角速度环           3
**PID2==>>姿态角度环             4
**sonar_PID ==>>超声波定高       5
*/
#define Block_Acc_ofst      0
#define Block_Gro_ofst      1
#define Block_Mag_ofst      2

#define Block_PID1_Set      3
#define Block_PID2_Set      4
#define Block_Sonar_PID_Set 5

static void  Param_SetSettingToFC(void) 
{
	memcpy(&icm20602.Acc_Offset,&sensor_setup.Offset.Accel,sizeof(xyz_f_t));
	memcpy(&icm20602.Gyro_Offset,&sensor_setup.Offset.Gyro,sizeof(xyz_f_t));
	memcpy(&ist8310.Mag_Offset,&sensor_setup.Offset.Mag,sizeof(xyz_f_t));
	memcpy(&icm20602.vec_3d_cali,&sensor_setup.Offset.vec_3d_cali,sizeof(xyz_f_t));
	
	icm20602.Acc_Temprea_Offset = sensor_setup.Offset.Acc_Temperature;
	icm20602.Gyro_Temprea_Offset = sensor_setup.Offset.Gyro_Temperature;
  
  memcpy(&ctrl_1.PID[PIDROLL],&pid_setup.groups.ctrl1.roll,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDPITCH],&pid_setup.groups.ctrl1.pitch,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDYAW],&pid_setup.groups.ctrl1.yaw,sizeof(pid_t));
	
	memcpy(&ctrl_2.PID[PIDROLL],&pid_setup.groups.ctrl2.roll,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDPITCH],&pid_setup.groups.ctrl2.pitch,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDYAW],&pid_setup.groups.ctrl2.yaw,sizeof(pid_t));

}

//存储加速度计校准数据扇区1,返回:1成功   0失败
bool Save_Acc_Calibration_Date(xyz_f_t *offset)
{
	
	unsigned int Buffer[16];
	union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	memcpy(&sensor_setup.Offset.Accel, offset,sizeof(xyz_f_t));
	Buffer[0]=0xAAAAAAAA;
	r1.Float_Data=sensor_setup.Offset.Accel.x;Buffer[1]=r1.Uint_Data;
	r1.Float_Data=sensor_setup.Offset.Accel.y;Buffer[2]=r1.Uint_Data;
	r1.Float_Data=sensor_setup.Offset.Accel.z;Buffer[3]=r1.Uint_Data;
	
	//unsigned char EEPOM_Write(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count)
	EEPROM_Write((uint32_t*)Buffer,Block_Acc_ofst,16);
	return 0;
}
//读取加速度计校准数据,返回:0成功   1失败
bool Read_Acc_Calibration_Date(void)
{
	unsigned int Buffer[16];
  union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	//void EEPOM_Read(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count)
	/*0号扇区存储6轴数据零偏*/
	EEPROM_Read((uint32_t*)Buffer,Block_Acc_ofst,16);
	if(Buffer[0]!=0xAAAAAAAA){return 1;}
	{
		r1.Uint_Data=Buffer[1];sensor_setup.Offset.Accel.x = r1.Float_Data;
		r1.Uint_Data=Buffer[2];sensor_setup.Offset.Accel.y = r1.Float_Data;
		r1.Uint_Data=Buffer[3];sensor_setup.Offset.Accel.z = r1.Float_Data;
	}
	return 0;
}
//存储陀螺仪校准数据扇区1,返回:1成功   0失败
bool Save_Gyro_Calibration_Date(xyz_f_t *offset)
{
	
	unsigned int Buffer[16];
	union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	memcpy(&sensor_setup.Offset.Gyro, offset,sizeof(xyz_f_t));
	Buffer[0]=0xAAAAAAAA;
	r1.Float_Data=sensor_setup.Offset.Gyro.x;Buffer[1]=r1.Uint_Data;
	r1.Float_Data=sensor_setup.Offset.Gyro.y;Buffer[2]=r1.Uint_Data;
	r1.Float_Data=sensor_setup.Offset.Gyro.z;Buffer[3]=r1.Uint_Data;
	
	//unsigned char EEPOM_Write(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count)
	EEPROM_Write((uint32_t*)Buffer,Block_Gro_ofst,16);
	return 0;
}
//读取陀螺仪校准数据扇区1,返回:1成功   0失败
bool Read_Gyro_Calibration_Date(void)
{
	unsigned int Buffer[16];
  union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	//void EEPOM_Read(uint32_t *pui32Data, uint32_t block, uint32_t ui32Count)
	/*0号扇区存储6轴数据零偏*/
	EEPROM_Read((uint32_t*)Buffer,Block_Gro_ofst,16);
	if(Buffer[0]!=0xAAAAAAAA){return 1;}
	{
		r1.Uint_Data=Buffer[1];sensor_setup.Offset.Gyro.x = r1.Float_Data;
		r1.Uint_Data=Buffer[2];sensor_setup.Offset.Gyro.y = r1.Float_Data;
		r1.Uint_Data=Buffer[3];sensor_setup.Offset.Gyro.z = r1.Float_Data;
	}
	return 0;
}

//读取磁力计校准数据,返回:0成功   1失败
bool Read_Mag_Calibration_Date(void)
{
	unsigned int Buffer[16];
  union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	/*1号扇区存储磁力计数据*/
	EEPROM_Read((uint32_t*)Buffer,Block_Mag_ofst,16);
	if(Buffer[0]!=0xAAAAAAAA){return 0;}
	{
		r1.Uint_Data=Buffer[1];sensor_setup.Offset.Mag.x=r1.Float_Data;
		r1.Uint_Data=Buffer[2];sensor_setup.Offset.Mag.y=r1.Float_Data;
		r1.Uint_Data=Buffer[3];sensor_setup.Offset.Mag.z=r1.Float_Data;

	}
	return 0;
}
//存储磁力计校准数据扇区2,返回:0成功   1失败
bool Save_Mag_Calibration_Date(xyz_f_t *offset)
{
	unsigned int Buffer[16];
	union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	memcpy(&sensor_setup.Offset.Mag, offset,sizeof(xyz_f_t));	
	Buffer[0]=0xAAAAAAAA;
	r1.Float_Data=sensor_setup.Offset.Mag.x;Buffer[1]=r1.Uint_Data;
	r1.Float_Data=sensor_setup.Offset.Mag.y;Buffer[2]=r1.Uint_Data;
	r1.Float_Data=sensor_setup.Offset.Mag.z;Buffer[3]=r1.Uint_Data;
	EEPROM_Write((uint32_t*)Buffer,Block_Mag_ofst,16);
	return 0;
}

void Param_Save_3d_offset(xyz_f_t *offset)
{
	
}
void Para_ResetToFactorySetup(void)
{
 	/* 加速计默认校准值 */
 	sensor_setup.Offset.Accel.x = 0;
 	sensor_setup.Offset.Accel.y = 0;
 	sensor_setup.Offset.Accel.z = 0;
 	/* 陀螺仪默认校准值 */
 	sensor_setup.Offset.Gyro.x = 0;
 	sensor_setup.Offset.Gyro.y = 0;
 	sensor_setup.Offset.Gyro.z = 0;
 	/* 罗盘默认校准值 */
 	sensor_setup.Offset.Mag.x = 0;		
 	sensor_setup.Offset.Mag.y = 0;		
 	sensor_setup.Offset.Mag.z = 0;	
    /* 温度默认校准值 */	
 	sensor_setup.Offset.Acc_Temperature = 0;
 	sensor_setup.Offset.Gyro_Temperature = 0;
	
  /* PID 默认值 */
	pid_setup.groups.ctrl1.pitch.kp = 0.8;
	pid_setup.groups.ctrl1.roll.kp  = 0.8;	
	pid_setup.groups.ctrl1.yaw.kp   = 1.2;	//1.2
	
	pid_setup.groups.ctrl1.pitch.ki = 0.1;
	pid_setup.groups.ctrl1.roll.ki  = 0.1;	
	pid_setup.groups.ctrl1.yaw.ki   = 1.0;	
	
	pid_setup.groups.ctrl1.pitch.kd = 2.5; //2.0
	pid_setup.groups.ctrl1.roll.kd  = 2.5;	//2.0
	pid_setup.groups.ctrl1.yaw.kd   = 1.0;	
	
	pid_setup.groups.ctrl1.pitch.kdamp = 1;
	pid_setup.groups.ctrl1.roll.kdamp  = 1;	
	pid_setup.groups.ctrl1.yaw.kdamp   = 1;

  pid_setup.groups.ctrl2.pitch.kp = 0.5;
  pid_setup.groups.ctrl2.roll.kp  = 0.5;	
	pid_setup.groups.ctrl2.yaw.kp   = 0.2;	
	
	pid_setup.groups.ctrl2.pitch.ki = 0.05;
	pid_setup.groups.ctrl2.roll.ki  = 0.05;	
	pid_setup.groups.ctrl2.yaw.ki   = 0.05;	
	
	pid_setup.groups.ctrl2.pitch.kd = 0.3;
	pid_setup.groups.ctrl2.roll.kd  = 0.3;
  pid_setup.groups.ctrl2.yaw.kd   = 0.1;
	
	pid_setup.groups.position_ctrl1.kp = 1.0f;
	pid_setup.groups.position_ctrl1.ki = 1.0f;
	pid_setup.groups.position_ctrl1.kd = 1.0f;
	
	pid_setup.groups.position_ctrl2.kp = 1.0f;
	pid_setup.groups.position_ctrl2.ki = 1.0f;
	pid_setup.groups.position_ctrl2.kd = 1.0;
	
	pid_setup.groups.sonar_hc_ctrl1.kp = 1.0f;
	pid_setup.groups.sonar_hc_ctrl1.ki = 1.0f;
	pid_setup.groups.sonar_hc_ctrl1.kd = 1.0f;
	
	pid_setup.groups.sonar_hc_ctrl2.kp = 1.0f;
	pid_setup.groups.sonar_hc_ctrl2.ki = 1.0f;
	pid_setup.groups.sonar_hc_ctrl2.kd = 1.0f;	
	
	Param_SetSettingToFC();
	PID_Para_Init();
}

void PID_Para_Init()
{
	Ctrl_Para_Init();
	WZ_Speed_PID_Init();
	Ultra_PID_Init();
}

void Para_Init()
{
	Read_Acc_Calibration_Date();
	Read_Gyro_Calibration_Date();
	Read_Mag_Calibration_Date();
	PID_Para_Read();
	Param_SetSettingToFC();
	//Para_ResetToFactorySetup();  //第一次刷代码请取消注释本段，将PID及传感器参数设为默认值

	PID_Para_Init();
}
bool PID_Para_Read(void)
{
	unsigned int Buffer[16];
  union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	/*读取PID1存储扇区的数据*/
	EEPROM_Read((uint32_t*)Buffer,Block_PID1_Set,64);
	if(Buffer[0]!=0xAAAAAAAA){return 0;}
	{
		r1.Uint_Data=Buffer[1];pid_setup.groups.ctrl1.roll.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[2];pid_setup.groups.ctrl1.roll.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[3];pid_setup.groups.ctrl1.roll.kd    = r1.Float_Data;
		r1.Uint_Data=Buffer[4];pid_setup.groups.ctrl1.roll.kdamp = r1.Float_Data;
		
		r1.Uint_Data=Buffer[5];pid_setup.groups.ctrl1.pitch.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[6];pid_setup.groups.ctrl1.pitch.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[7];pid_setup.groups.ctrl1.pitch.kd    = r1.Float_Data;
		r1.Uint_Data=Buffer[8];pid_setup.groups.ctrl1.pitch.kdamp = r1.Float_Data;
		
		r1.Uint_Data=Buffer[9]; pid_setup.groups.ctrl1.yaw.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[10];pid_setup.groups.ctrl1.yaw.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[11];pid_setup.groups.ctrl1.yaw.kd    = r1.Float_Data;
		r1.Uint_Data=Buffer[12];pid_setup.groups.ctrl1.yaw.kdamp = r1.Float_Data;
	}
	/*读取PID2存储扇区的数据*/
	EEPROM_Read((uint32_t*)Buffer,Block_PID2_Set,64);
	if(Buffer[0]!=0xAAAAAAAA){return 0;}
	{
		r1.Uint_Data=Buffer[1];pid_setup.groups.ctrl2.roll.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[2];pid_setup.groups.ctrl2.roll.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[3];pid_setup.groups.ctrl2.roll.kd    = r1.Float_Data;
		r1.Uint_Data=Buffer[4];pid_setup.groups.ctrl2.roll.kdamp = r1.Float_Data;
		
		r1.Uint_Data=Buffer[5];pid_setup.groups.ctrl2.pitch.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[6];pid_setup.groups.ctrl2.pitch.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[7];pid_setup.groups.ctrl2.pitch.kd    = r1.Float_Data;
		r1.Uint_Data=Buffer[8];pid_setup.groups.ctrl2.pitch.kdamp = r1.Float_Data;
		
		r1.Uint_Data=Buffer[9]; pid_setup.groups.ctrl2.yaw.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[10];pid_setup.groups.ctrl2.yaw.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[11];pid_setup.groups.ctrl2.yaw.kd    = r1.Float_Data;
		r1.Uint_Data=Buffer[12];pid_setup.groups.ctrl2.yaw.kdamp = r1.Float_Data;
	}
	/*读取超声波定高PID存储扇区的数据*/
	EEPROM_Read((uint32_t*)Buffer,Block_Sonar_PID_Set,64);
	if(Buffer[0]!=0xAAAAAAAA){return 0;}
	{
		r1.Uint_Data=Buffer[1];pid_setup.groups.sonar_hc_ctrl1.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[2];pid_setup.groups.sonar_hc_ctrl1.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[3];pid_setup.groups.sonar_hc_ctrl1.kd    = r1.Float_Data;
		
		r1.Uint_Data=Buffer[4];pid_setup.groups.sonar_hc_ctrl2.kp    = r1.Float_Data;
		r1.Uint_Data=Buffer[5];pid_setup.groups.sonar_hc_ctrl2.ki    = r1.Float_Data;
		r1.Uint_Data=Buffer[6];pid_setup.groups.sonar_hc_ctrl2.kd    = r1.Float_Data;
	}
	
	return 0;
}
void Param_SavePID(void)
{
	unsigned int Buffer[16]={0};
	union result      //共用体,用于float转4个unsigned char
  {
    float Float_Data;
    unsigned int Uint_Data;
  }r1;
	memcpy(&pid_setup.groups.ctrl1.roll,&ctrl_1.PID[PIDROLL],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl1.pitch,&ctrl_1.PID[PIDPITCH],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl1.yaw,&ctrl_1.PID[PIDYAW],sizeof(pid_t));
	
	Buffer[0]=0xAAAAAAAA;
	r1.Float_Data=pid_setup.groups.ctrl1.roll.kp;   Buffer[1]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.roll.ki;   Buffer[2]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.roll.kd;   Buffer[3]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.roll.kdamp;Buffer[4]=r1.Uint_Data;
	
	r1.Float_Data=pid_setup.groups.ctrl1.pitch.kp;   Buffer[5]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.pitch.ki;   Buffer[6]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.pitch.kd;   Buffer[7]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.pitch.kdamp;Buffer[8]=r1.Uint_Data;
	
	r1.Float_Data=pid_setup.groups.ctrl1.yaw.kp;    Buffer[9]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.yaw.ki;    Buffer[10]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.yaw.kd;    Buffer[11]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl1.yaw.kdamp; Buffer[12]=r1.Uint_Data;
	/*存储PID1参数*/
	EEPROM_Write((uint32_t*)Buffer,Block_PID1_Set,64);
	
	memcpy(&pid_setup.groups.ctrl2.roll,&ctrl_2.PID[PIDROLL],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl2.pitch,&ctrl_2.PID[PIDPITCH],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl2.yaw,&ctrl_2.PID[PIDYAW],sizeof(pid_t));
	r1.Float_Data=pid_setup.groups.ctrl2.roll.kp;   Buffer[1]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.roll.ki;   Buffer[2]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.roll.kd;   Buffer[3]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.roll.kdamp;Buffer[4]=r1.Uint_Data;
	
	r1.Float_Data=pid_setup.groups.ctrl2.pitch.kp;   Buffer[5]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.pitch.ki;   Buffer[6]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.pitch.kd;   Buffer[7]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.pitch.kdamp;Buffer[8]=r1.Uint_Data;
	
	r1.Float_Data=pid_setup.groups.ctrl2.yaw.kp;    Buffer[9]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.yaw.ki;    Buffer[10]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.yaw.kd;    Buffer[11]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.ctrl2.yaw.kdamp; Buffer[12]=r1.Uint_Data;

	/*存储PID2参数*/
	EEPROM_Write((uint32_t*)Buffer,Block_PID2_Set,64);
//	pid_setup.groups.sonar_hc_ctrl1.kp = 0.4f;
//	pid_setup.groups.sonar_hc_ctrl1.ki = 0.12f;
//	pid_setup.groups.sonar_hc_ctrl1.kd = 1.4f;
//	
//	pid_setup.groups.sonar_hc_ctrl2.kp = 1.5f;
//	pid_setup.groups.sonar_hc_ctrl2.ki = 0.0f;
//	pid_setup.groups.sonar_hc_ctrl2.kd = 2.5f;
	r1.Float_Data=pid_setup.groups.sonar_hc_ctrl1.kp;   Buffer[1]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.sonar_hc_ctrl1.ki;   Buffer[2]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.sonar_hc_ctrl1.kd;   Buffer[3]=r1.Uint_Data;
	
	r1.Float_Data=pid_setup.groups.sonar_hc_ctrl2.kp;   Buffer[4]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.sonar_hc_ctrl2.ki;   Buffer[5]=r1.Uint_Data;
	r1.Float_Data=pid_setup.groups.sonar_hc_ctrl2.kd;   Buffer[6]=r1.Uint_Data;
	/*存储超声波定高参数*/
	EEPROM_Write((uint32_t*)Buffer,Block_Sonar_PID_Set,64);
}
extern u16 flash_save_en_cnt;

void Parameter_Save()
{
	flash_save_en_cnt++;
	if( flash_save_en_cnt > 100 ) // 20 *60 = 1200ms
	{
		flash_save_en_cnt = 0;
		if( !fly_ready )
		{
			Param_SavePID();
		}
	}
}
