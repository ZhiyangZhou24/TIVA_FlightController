#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "include.h"

/** @addtogroup Exported_types
  * @{
  */
/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32 ;

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;

}pid_t;

typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

typedef struct 
{
  s16 x;
	s16 y;
	s16 z;
}xyz_s16_t;


typedef union
{
	uint8_t raw_data[64];
	struct
	{
		xyz_f_t Accel;
		xyz_f_t Gyro;
		xyz_f_t Mag;
		xyz_f_t vec_3d_cali;
		uint32_t mpu_flag;
		float Acc_Temperature;
		float Gyro_Temperature;
	}Offset;
}sensor_setup_t;

typedef  struct{
	pid_t roll;
	pid_t pitch;	
	pid_t yaw;	
 }pid_group_t;

typedef union
{
 uint8_t raw_data[192];
 struct
 {
	 pid_group_t ctrl1; //姿态内环
	 pid_group_t ctrl2; //姿态外环
 /////////////////////
	 pid_t sonar_hc_ctrl1;     //定高内环  （超声波）
	 pid_t sonar_hc_ctrl2;     //定高外环
	 pid_t position_ctrl1;     //定点内环
	 pid_t position_ctrl2;     //定点外环
 }groups;

}pid_setup_t;

bool Read_Acc_Calibration_Date(void);
bool Save_Acc_Calibration_Date(xyz_f_t *offset);
bool Read_Gyro_Calibration_Date(void);
bool Save_Gyro_Calibration_Date(xyz_f_t *offset);
bool Read_Mag_Calibration_Date(void);
bool Save_Mag_Calibration_Date(xyz_f_t *offset);

extern sensor_setup_t sensor_setup;
extern pid_setup_t pid_setup;

void Senser_Calibrat_Read(void);
bool PID_Para_Read(void);
void Para_Init(void);

void Para_ResetToFactorySetup(void);
void Param_SavePID(void);
void Param_SaveAccelOffset(xyz_f_t *offset);
void Param_SaveGyroOffset(xyz_f_t *offset);
void Param_SaveMagOffset(xyz_f_t *offset);
void Param_Save_3d_offset(xyz_f_t *offset);
void Parameter_Save(void);
void PID_Para_Init(void);

#endif

