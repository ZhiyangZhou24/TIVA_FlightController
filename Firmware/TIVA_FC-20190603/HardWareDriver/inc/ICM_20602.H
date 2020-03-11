#ifndef _ICM_20602_H_
#define _ICM_20602_H_
#include "include.h"
#include "FC_Config.h"
#include "parameter.h"

#define GYRO_DEATH_ZOOM 20   //  20 / 65536

#define OFFSET_AV_NUM 50
#define FILTER_NUM 10


typedef struct 
{
	char Acc_CALIBRATE;
	char Gyro_CALIBRATE;
	char Cali_3d;
  xyz_s16_t Acc_I16;
	xyz_s16_t Gyro_I16;

	xyz_f_t Acc;
	xyz_f_t Gyro;

//	XYZ_STRUCT Acc_deg;
	xyz_f_t Gyro_deg;
	
	xyz_f_t Acc_Offset;
	xyz_f_t Gyro_Offset;
	xyz_f_t Gyro_Auto_Offset;
	xyz_f_t vec_3d_cali;
	float Acc_Temprea_Offset;
	float Gyro_Temprea_Offset;
	
	float Gyro_Temprea_Adjust;
	float ACC_Temprea_Adjust;

	s16 Tempreature;
	float TEM_LPF;
	float Ftempreature;
}ICM_STRUCT;

extern ICM_STRUCT icm20602;
extern float icm20602_tmp[ITEMS];
extern u8 acc_3d_calibrate_f,acc_3d_step;
extern u8 icm20602_ok;

bool ICM_20602_Init(void);
void ICM_Get_Motion6(void);
void icm20602_Data_Prepare(float T);
float constrain0(float val, float min, float max);
	
#ifdef __cplusplus
}
#endif

#endif
