#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "include.h"

typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_height_pid;

void Height_Ctrl(float T,float thr);

void Ultra_PID_Init(void);

void WZ_Speed_PID_Init(void);

void height_speed_ctrl(float T,float thr,float exp_z_speed,float );

void Baro_Ctrl(float T,float thr);

void Ultra_Ctrl(float T,float thr);

extern float ultra_ctrl_out;

extern float height_ctrl_out;

extern u8 baro_ctrl_start;

extern float ultra_speed,wz_speed,baro_height;
#endif

