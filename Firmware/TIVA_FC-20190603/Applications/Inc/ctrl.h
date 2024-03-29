#ifndef _CTRL_H
#define	_CTRL_H

#include "include.h"
#include "FC_Config.h"
#include "rc.h"
#include "parameter.h"
//#include "imu.h"
//#include "icm20602.h"

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
		PID4,
		PID5,
		PID6,

		PIDITEMS
};



typedef struct
{
	xyz_f_t err;
	xyz_f_t err_old;
	xyz_f_t err_i;
	xyz_f_t eliminate_I;
	xyz_f_t err_d;
	xyz_f_t damp;
	xyz_f_t out;
	pid_t 	PID[PIDITEMS];
	xyz_f_t err_weight;
	float FB;

}ctrl_t;

extern ctrl_t ctrl_1;
extern ctrl_t ctrl_2;

void CTRL_2(float);
void CTRL_1(float);
void Ctrl_Para_Init(void);
void Thr_Ctrl(float);
void All_Out(float x,float y,float z);

extern u8 Thr_Low;
extern float Thr_Weight;
extern float motor[MAXMOTORS];

#endif

